//
//  AutoTuner.swift
//  Crazyflie client
//
//  Trial framework + four optimizers, ported from PID-Calibration's AutoTuner.kt.
//  The balance-robot cost (wheel speed, travel, fall) is replaced with the
//  drone's own telemetry (PROTOCOL.md §4 log TOC):
//
//      cost = angle_std
//           + 0.1 × gyro_rms
//           + 0.2 × motor_rms
//           + 0.005 × control_effort
//           + 20 × saturated_share
//
//  Only the four optimizers named in the brief ship here (ML / Twiddle /
//  Nelder-Mead / Random). The other seven of AutoTuner.kt's eleven are
//  deliberately not ported — they need 30+ trials and the flight-risk budget
//  does not justify them.
//
//  SAFETY: this class never starts a trial on its own. The caller must run the
//  hold-to-confirm gate first, must keep the drone secured on a stand or
//  tethered, and must wire `panicStop` to an emergency cut. One axis at a time.
//

import Foundation

// MARK: - Search space

/// Log-scaled box around the starting gains, mapped to the unit cube.
/// Factor 2.0 → the search range is current/2 .. current×2, matching
/// AutoTuner.kt's default `Method.defaultRange`.
struct TuneSpace {
    let names: [String]
    let lo: [Double]
    let hi: [Double]
    let logScaled: [Bool]

    /// - Parameter centre: starting gain values, same order as `names`.
    init(names: [String], centre: [Double], factor: Double) {
        self.names = names
        var lo: [Double] = []
        var hi: [Double] = []
        var logScaled: [Bool] = []
        for c in centre {
            // Gains are multiplicative, so the box is log-scaled. A non-positive
            // start is treated as a tiny positive so the search can still move.
            let base = c > 0 ? c : 0.05
            lo.append(log(max(base / factor, 1e-3)))
            hi.append(log(min(base * factor, 1e6)))
            logScaled.append(true)
        }
        self.lo = lo
        self.hi = hi
        self.logScaled = logScaled
    }

    /// Unit-cube point → real gain values.
    func value(_ k: Int, _ u: Double) -> Double {
        exp(lo[k] + u * (hi[k] - lo[k]))
    }

    /// Real values → unit-cube point, or `nil` when outside the box.
    func toUnit(_ values: [Double]) -> [Double]? {
        var x: [Double] = []
        for (k, v) in values.enumerated() {
            if logScaled[k] && v <= 0 { return nil }
            let u = ((logScaled[k] ? log(v) : v) - lo[k]) / (hi[k] - lo[k])
            if u < -1e-6 || u > 1 + 1e-6 { return nil }
            x.append(min(1, max(0, u)))
        }
        return x
    }
}

// MARK: - Cost

/// Per-trial measurements used to score a candidate.
struct TuneMetrics {
    var angleMean: Double
    var angleStd: Double
    var gyroRms: Double
    var motorRms: Double
    var controlEffort: Double
    var satShare: Double
    var sampleCount: Int

    /// The brief's cost. Drop the wheel-encoder and fall terms — a quad has
    /// neither, and a tip-over is the emergency path, not a cost signal.
    var cost: Double {
        angleStd
            + 0.1 * gyroRms
            + 0.2 * motorRms
            + 0.005 * controlEffort
            + 20 * satShare
    }
}

/// One decoded telemetry row the tuner cares about.
struct TuneSample {
    /// Active axis angle (stabilizer.roll / .pitch / .yaw rate), degrees.
    let angle: Double
    /// Motor output in µs (motor.m1..m4), averaged later.
    let motorMicros: [Double]
    /// Controller output for the active axis (controller.roll / .pitch / .yaw).
    let controller: Double
}

// MARK: - Gaussian process (ML)

/// Zero-mean GP on standardised targets, RBF kernel over unit-cube inputs,
/// marginal-likelihood grid over LENGTHS × NOISES — a direct port of
/// AutoTuner.kt's `GaussianProcess`.
final class GaussianProcess {
    private let xs: [[Double]]
    private let mu: Double
    private let sd: Double
    private let ls: Double
    private let chol: [[Double]]
    private let alpha: [Double]

    static let lengths: [Double] = [0.08, 0.15, 0.25, 0.4, 0.7, 1.2]
    static let noises: [Double] = [0.01, 0.05, 0.15, 0.4]

    init?(xs: [[Double]], y: [Double]) {
        guard !xs.isEmpty, xs.count == y.count else { return nil }
        self.xs = xs
        let n = y.count
        let mean = y.reduce(0, +) / Double(n)
        self.mu = mean
        let sdev = sqrt(y.reduce(0) { $0 + ($1 - mean) * ($1 - mean) } / Double(n))
        let sd = (n > 1 && sdev > 1e-9) ? sdev : 1.0
        self.sd = sd
        // Local `sd` — a closure over `self.sd` here would capture self before
        // ls/chol/alpha are initialised.
        let z = y.map { ($0 - mean) / sd }

        var bestLml = -Double.infinity
        var bestL = 0.25
        var bestChol: [[Double]]?
        var bestAlpha: [Double]?

        for l in GaussianProcess.lengths {
            for noise in GaussianProcess.noises {
                var k = [[Double]](repeating: [Double](repeating: 0, count: n), count: n)
                for i in 0..<n {
                    for j in 0..<n {
                        k[i][j] = GaussianProcess.rbf(xs[i], xs[j], l) + (i == j ? noise : 0)
                    }
                }
                guard let c = GaussianProcess.cholesky(k) else { continue }
                let a = GaussianProcess.backSolve(c, GaussianProcess.forwardSolve(c, z))
                var lml = 0.0
                for i in 0..<n {
                    lml += -0.5 * z[i] * a[i] - log(c[i][i])
                }
                if lml > bestLml {
                    bestLml = lml
                    bestL = l
                    bestChol = c
                    bestAlpha = a
                }
            }
        }
        guard let chol = bestChol, let alpha = bestAlpha else { return nil }
        self.ls = bestL
        self.chol = chol
        self.alpha = alpha
    }

    /// Posterior mean and std of the latent y at `x`.
    func predict(_ x: [Double]) -> (mean: Double, std: Double) {
        let ks = xs.map { GaussianProcess.rbf(x, $0, ls) }
        var mean = 0.0
        for i in ks.indices {
            mean += ks[i] * alpha[i]
        }
        let v = GaussianProcess.forwardSolve(chol, ks)
        let variance = max(1e-12, 1 - v.reduce(0) { $0 + $1 * $1 })
        return (mu + sd * mean, sd * sqrt(variance))
    }

    /// Expected improvement over the best observed `yBest` (in standardised cost).
    func expectedImprovement(_ x: [Double], yBest: Double, xi: Double = 0.01) -> Double {
        let (mean, std) = predict(x)
        let imp = yBest - mean - xi
        let z = imp / std
        return imp * GaussianProcess.normCdf(z)
            + std * exp(-0.5 * z * z) / sqrt(2 * .pi)
    }

    private static func rbf(_ a: [Double], _ b: [Double], _ l: Double) -> Double {
        var s = 0.0
        for i in a.indices {
            let d = a[i] - b[i]
            s += d * d
        }
        return exp(-0.5 * s / (l * l))
    }

    private static func cholesky(_ a: [[Double]]) -> [[Double]]? {
        let n = a.count
        var l = [[Double]](repeating: [Double](repeating: 0, count: n), count: n)
        for i in 0..<n {
            for j in 0...i {
                var s = a[i][j]
                for k in 0..<j {
                    s -= l[i][k] * l[j][k]
                }
                if i == j {
                    if s <= 0 { return nil }
                    l[i][i] = sqrt(s)
                } else {
                    l[i][j] = s / l[j][j]
                }
            }
        }
        return l
    }

    /// L x = b (lower-triangular).
    private static func forwardSolve(_ l: [[Double]], _ b: [Double]) -> [Double] {
        var x = [Double](repeating: 0, count: b.count)
        for i in b.indices {
            var s = b[i]
            for k in 0..<i {
                s -= l[i][k] * x[k]
            }
            x[i] = s / l[i][i]
        }
        return x
    }

    /// Lᵀ x = b.
    private static func backSolve(_ l: [[Double]], _ b: [Double]) -> [Double] {
        var x = [Double](repeating: 0, count: b.count)
        for i in b.indices.reversed() {
            var s = b[i]
            for k in (i + 1)..<b.count {
                s -= l[k][i] * x[k]
            }
            x[i] = s / l[i][i]
        }
        return x
    }

    /// Abramowitz-Stegun 7.1.26 erf, |error| < 1.5e-7.
    private static func normCdf(_ z: Double) -> Double {
        let x = abs(z) / sqrt(2.0)
        let t = 1 / (1 + 0.3275911 * x)
        let erf = 1 - t * (0.254829592
            + t * (-0.284496736
            + t * (1.421413741
            + t * (-1.453152027
            + t * 1.061405429)))) * exp(-x * x)
        return 0.5 * (1 + (z >= 0 ? erf : -erf))
    }
}

// MARK: - Tuner

/// Which axis is under tune. One at a time — the safety gate enforces it.
enum TuneAxis: Int, CaseIterable {
    case roll = 0
    case pitch
    case yaw

    var title: String {
        switch self {
        case .roll: return "Roll"
        case .pitch: return "Pitch"
        case .yaw: return "Yaw"
        }
    }

    /// Parameter TOC ids for kp/ki/kd on this axis (PROTOCOL.md §3).
    var gainParamIds: [UInt8] {
        switch self {
        case .roll: return [3, 4, 5]    // pid.roll_kp, ki, kd
        case .pitch: return [6, 7, 8]
        case .yaw: return [9, 10, 11]
        }
    }

    var gainNames: [String] {
        switch self {
        case .roll: return ["pid.roll_kp", "pid.roll_ki", "pid.roll_kd"]
        case .pitch: return ["pid.pitch_kp", "pid.pitch_ki", "pid.pitch_kd"]
        case .yaw: return ["pid.yaw_kp", "pid.yaw_ki", "pid.yaw_kd"]
        }
    }

    /// Log TOC names for the angle / controller / motor samples.
    var angleLogName: String {
        switch self {
        case .roll: return "stabilizer.roll"
        case .pitch: return "stabilizer.pitch"
        case .yaw: return "stabilizer.yaw"
        }
    }

    var controllerLogName: String {
        switch self {
        case .roll: return "controller.roll"
        case .pitch: return "controller.pitch"
        case .yaw: return "controller.yaw"
        }
    }
}

/// Exactly four optimizers, per the brief.
enum TuneMethod: Int, CaseIterable {
    case ml = 0
    case twiddle
    case nelderMead
    case random

    var title: String {
        switch self {
        case .ml: return "ML (Bayesian)"
        case .twiddle: return "Twiddle"
        case .nelderMead: return "Nelder-Mead"
        case .random: return "Random"
        }
    }

    var hint: String {
        switch self {
        case .ml:
            return "Gaussian process + expected improvement. Fewest trials; the default."
        case .twiddle:
            return "One gain at a time, wins re-measured. Simple, stays close to the start."
        case .nelderMead:
            return "Downhill simplex: local, no model, sensitive to noise."
        case .random:
            return "Latin-hypercube samples: what the others must beat."
        }
    }

    /// Search box as a multiplier on the starting gains.
    var defaultRange: Double {
        self == .twiddle ? 3.0 : 2.0
    }
}

/// Runs the trial loop on its own serial queue. The UI feeds samples in via
/// `ingest(_:)` and reads back via the listener callbacks (always main-queue).
final class AutoTuner {

    struct Config {
        var axis: TuneAxis
        var method: TuneMethod
        var trials: Int
        /// Seconds of telemetry per trial, after the settle window.
        var trialSeconds: Double
        /// Seconds of no-input settle after writing a candidate.
        var settleSeconds: Double
        var range: Double
    }

    protocol Listener: AnyObject {
        func tunerStatus(_ text: String)
        func tunerTrial(_ report: TrialReport)
        func tunerFinished(reason: String, start: [Double]?, result: [Double]?)
    }

    /// What the model looks like after a trial.
    struct TrialReport {
        let n: Int
        let total: Int
        let cost: Double
        let candidate: [Double]
        let best: [Double]
        let bestCost: Double
    }

    // MARK: Tuner state

    private let cfg: Config
    private let client: CrtpClient
    weak var listener: Listener?

    private let queue = DispatchQueue(label: "crtp.autotune")
    private let stateLock = NSLock()

    /// Telemetry ring fed by the log client. Main-thread writers, queue readers.
    private var pendingSamples: [TuneSample] = []
    private var paramEcho: [UInt8: Double] = [:]

    private var stopped = false
    private var running = false

    /// Current gains being held (kp, ki, kd).
    private var params: [Double]
    private var startParams: [Double]?
    private var bestParams: [Double]
    private var bestCost = Double.infinity

    private var space: TuneSpace
    private var xs: [[Double]] = []
    private var ys: [Double] = []            // toY(cost)
    private var own: [Bool] = []
    private var k = 0

    private var rng = SystemRandomNumberGenerator()

    init(config: Config, client: CrtpClient, startGains: [Double]) {
        self.cfg = config
        self.client = client
        self.params = startGains
        self.bestParams = startGains
        self.startParams = startGains
        self.space = TuneSpace(names: config.axis.gainNames,
                               centre: startGains,
                               factor: config.range)
    }

    var isRunning: Bool {
        stateLock.lock()
        defer { stateLock.unlock() }
        return running
    }

    /// Feed decoded telemetry. Safe from any thread.
    func ingest(_ sample: TuneSample) {
        stateLock.lock()
        pendingSamples.append(sample)
        if pendingSamples.count > 1500 {
            pendingSamples.removeFirst(pendingSamples.count - 1500)
        }
        stateLock.unlock()
    }

    /// Feed a param echo (id → value) from `CrtpClient.onParamValue`.
    func ingestParam(id: UInt8, value: Double) {
        stateLock.lock()
        paramEcho[id] = value
        stateLock.unlock()
    }

    /// Begin searching. The caller must already have cleared the safety gate.
    func start() {
        queue.async { [weak self] in
            self?.run()
        }
    }

    /// Halts the tuner after the current step. The caller should also send
    /// `FC_CMD_EMERGENCY` — this class does not, so the panic path is one tap.
    func stop() {
        stateLock.lock()
        stopped = true
        stateLock.unlock()
    }

    // MARK: - Robot I/O

    private var now: Date { Date() }

    private func checkStop() throws {
        stateLock.lock()
        let s = stopped
        stateLock.unlock()
        if s { throw StopError.stopped }
    }

    private enum StopError: Error {
        case stopped
    }

    private func status(_ s: String) {
        DispatchQueue.main.async { [weak self] in
            self?.listener?.tunerStatus(s)
        }
    }

    /// Writes one gain and waits for the drone's echo. Retries 3× — the
    /// client's own 500 ms retransmission already covers the wire, this covers
    /// a lost reply.
    private func setGain(paramId: UInt8, _ value: Double) throws {
        for _ in 0..<3 {
            stateLock.lock()
            paramEcho[paramId] = nil
            stateLock.unlock()

            client.writeParam(id: paramId, value: .float(Float(value)))

            let deadline = now.addingTimeInterval(0.5)
            while now < deadline {
                stateLock.lock()
                let echo = paramEcho[paramId]
                stateLock.unlock()
                if echo != nil { return }
                Thread.sleep(forTimeInterval: 0.01)
            }
        }
        throw StopError.stopped
    }

    /// Applies the candidate, writing only what actually changed.
    private func apply(_ p: [Double]) throws {
        let ids = cfg.axis.gainParamIds
        for i in 0..<min(p.count, ids.count) {
            if abs(params[i] - p[i]) > 1e-6 {
                try setGain(paramId: ids[i], p[i])
                params[i] = p[i]
            }
        }
    }

    private func drainSamples(since t0: Date) -> [TuneSample] {
        stateLock.lock()
        defer { stateLock.unlock() }
        return pendingSamples
    }

    /// One trial with the parameters already applied. Settle → collect → score.
    private func measure() throws -> (cost: Double, metrics: TuneMetrics) {
        Thread.sleep(forTimeInterval: cfg.settleSeconds)
        stateLock.lock()
        pendingSamples.removeAll()
        stateLock.unlock()

        let t0 = now
        let window = cfg.trialSeconds
        while now.timeIntervalSince(t0) < window {
            try checkStop()
            Thread.sleep(forTimeInterval: 0.1)
        }

        let frames = drainSamples(since: t0)
        guard !frames.isEmpty else {
            status("No telemetry — is the tune log block running?")
            throw StopError.stopped
        }

        let angles = frames.map(\.angle)
        let mean = angles.reduce(0, +) / Double(angles.count)
        let angleStd = sqrt(angles.reduce(0) { $0 + ($1 - mean) * ($1 - mean) } / Double(angles.count))

        // Gyro-rate RMS: finite difference of the angle samples. For the yaw
        // axis the log carries a rate already (stabilizer.yaw is deg/s), but
        // the same formula still gives a usable magnitude.
        var gyroSq = 0.0
        if frames.count > 1 {
            for i in 1..<frames.count {
                let dt = max(1e-3, cfg.trialSeconds / Double(frames.count))
                let rate = (frames[i].angle - frames[i - 1].angle) / dt
                gyroSq += rate * rate
            }
        }
        let gyroRms = frames.count > 1 ? sqrt(gyroSq / Double(frames.count - 1)) : 0

        let motorSq = frames.reduce(0.0) { acc, f in
            let m = f.motorMicros.reduce(0, +) / Double(max(1, f.motorMicros.count))
            return acc + m * m
        }
        let motorRms = sqrt(motorSq / Double(frames.count))

        let ctrlSq = frames.reduce(0.0) { $0 + $1.controller * $1.controller }
        let controlEffort = sqrt(ctrlSq / Double(frames.count))

        // Motor saturation at the µs limits the firmware writes (1000…2000).
        let sat = frames.reduce(0) { acc, f in
            let hot = f.motorMicros.contains { $0 >= 2000 || $0 <= 1000 }
            return acc + (hot ? 1 : 0)
        }
        let satShare = Double(sat) / Double(frames.count)

        let metrics = TuneMetrics(
            angleMean: mean,
            angleStd: angleStd,
            gyroRms: gyroRms,
            motorRms: motorRms,
            controlEffort: controlEffort,
            satShare: satShare,
            sampleCount: frames.count
        )
        return (metrics.cost, metrics)
    }

    /// Evaluate one candidate: apply, measure, bookkeep.
    private func trial(_ p: [Double]) throws -> Double {
        try checkStop()
        try apply(p)
        let (cost, _) = try measure()
        k += 1

        let normalized = toY(cost)
        if let u = space.toUnit(p) {
            xs.append(u)
            ys.append(normalized)
            own.append(true)
        }
        if cost < bestCost {
            bestCost = cost
            bestParams = p
        }

        let report = TrialReport(n: k, total: cfg.trials,
                                 cost: cost, candidate: p,
                                 best: bestParams, bestCost: bestCost)
        DispatchQueue.main.async { [weak self] in
            self?.listener?.tunerTrial(report)
        }
        return normalized
    }

    /// log1p min-cost transform — matches AutoTuner.kt's `toY`.
    private func toY(_ cost: Double) -> Double {
        log1p(min(cost, 300))
    }

    private func latinHypercube(count: Int, d: Int) -> [[Double]] {
        guard count > 0 else { return [] }
        var cols: [[Double]] = []
        for _ in 0..<d {
            var col = (0..<count).map { (Double($0) + Double.random(in: 0..<1, using: &rng)) / Double(count) }
            col.shuffle(using: &rng)
            cols.append(col)
        }
        return (0..<count).map { i in
            (0..<d).map { cols[$0][i] }
        }
    }

    private func clip(_ x: [Double]) -> [Double] {
        x.map { min(1, max(0, $0)) }
    }

    // MARK: - Run

    private func run() {
        stateLock.lock()
        running = true
        stopped = false
        stateLock.unlock()

        var reason = "completed"
        defer {
            // Leave the best gains on the drone.
            do {
                try apply(bestParams)
                reason += " (best gains applied)"
            } catch {
                reason += " (could not apply best gains)"
            }
            stateLock.lock()
            running = false
            stateLock.unlock()
            let start = startParams
            let result = params
            DispatchQueue.main.async { [weak self] in
                self?.listener?.tunerFinished(reason: reason, start: start, result: result)
            }
        }

        do {
            switch cfg.method {
            case .ml:
                try searchML()
            case .twiddle:
                try searchTwiddle()
            case .nelderMead:
                try searchNelderMead()
            case .random:
                try searchRandom()
            }
            if k < cfg.trials {
                reason = "converged"
            } else {
                reason = "trial budget used"
            }
        } catch StopError.stopped {
            reason = "stopped"
        } catch {
            reason = "error: \(error.localizedDescription)"
        }
    }

    // MARK: ML (Bayesian GP + expected improvement)

    private func searchML() throws {
        let d = cfg.axis.gainNames.count
        let initPoints: [[Double]] = [
            space.toUnit(params) ?? [Double](repeating: 0.5, count: d)
        ] + latinHypercube(count: max(0, d + 2), d: d)

        while k < cfg.trials {
            try checkStop()
            let x: [Double]
            if k < initPoints.count {
                x = initPoints[k]
            } else {
                guard let gp = GaussianProcess(xs: xs, y: ys) else {
                    x = [Double](repeating: 0.5, count: d)
                    _ = try trial(candidates(from: x))
                    continue
                }
                let yRef = ys.min() ?? 0
                var cand: [[Double]] = (0..<2000).map { _ in
                    (0..<d).map { _ in Double.random(in: 0..<1, using: &rng) }
                }
                if let bestX = xs.min(by: { a, b in
                    gp.predict(a).mean < gp.predict(b).mean
                }) {
                    cand += (0..<600).map { _ in
                        bestX.map { min(1, max(0, $0 + 0.07 * Double.random(in: -1...1, using: &rng))) }
                    }
                }
                let best = cand.max(by: { gp.expectedImprovement($0, yBest: yRef) < gp.expectedImprovement($1, yBest: yRef) })
                x = best ?? [Double](repeating: 0.5, count: d)
            }
            _ = try trial(candidates(from: x))
        }
    }

    /// Unit-cube point → real gains.
    private func candidates(from x: [Double]) -> [Double] {
        (0..<x.count).map { space.value($0, x[$0]) }
    }

    // MARK: Twiddle

    private func searchTwiddle() throws {
        let ids = Array(0..<cfg.axis.gainNames.count)
        var bestP = bestParams
        let lo = bestP.map { $0 / cfg.range }
        let hi = bestP.map { $0 * cfg.range }

        status("Measuring the starting gains (2 trials)")
        let c0 = try trial(bestP)
        let c1 = try trial(bestP)
        bestCost = (toY(c0) + toY(c1)) / 2
        // Undo the bookkeeping double-count: the two identical trials share one
        // bestCost entry in AutoTuner.kt too; keep the running best as the mean.
        bestCost = (toY(c0) + toY(c1)) / 2

        var step = [Double](repeating: 0.25, count: ids.count)

        while k < cfg.trials && step.max() ?? 0 > 0.04 {
            for i in ids {
                var improved = false
                for sign in [1.0, -1.0] {
                    if k >= cfg.trials { break }
                    var cand = bestP
                    cand[i] = bestP[i] * (1 + sign * step[i])
                    cand[i] = min(hi[i], max(lo[i], cand[i]))
                    if abs(cand[i] - bestP[i]) < 1e-9 { continue }

                    let c = try trial(cand)
                    if c >= bestCost || k >= cfg.trials { continue }
                    // Confirm: noise must not win.
                    let c2 = try trial(cand)
                    if (c + c2) / 2 < bestCost {
                        bestP = cand
                        bestCost = (c + c2) / 2
                        bestParams = bestP
                        improved = true
                        status("++ \(cfg.axis.gainNames[i]) = \(String(format: "%.4g", bestP[i])) (best cost \(String(format: "%.2f", bestCost)))")
                        break
                    }
                }
                step[i] *= improved ? 1.3 : 0.6
                if k >= cfg.trials { break }
            }
        }
        status(k < cfg.trials ? "Steps below 4%: converged" : "Trial budget used")
    }

    // MARK: Nelder-Mead

    private func searchNelderMead() throws {
        let d = cfg.axis.gainNames.count
        let x0 = space.toUnit(params) ?? [Double](repeating: 0.5, count: d)
        var pts: [[Double]] = [x0]
        for j in 0..<d {
            var p = x0
            p[j] += p[j] <= 0.75 ? 0.25 : -0.25
            pts.append(clip(p))
        }
        var simplex = try pts.map { (try f($0), $0) }

        while k < cfg.trials {
            try checkStop()
            simplex.sort { $0.0 < $1.0 }
            let c = (0..<d).map { j in
                simplex.dropLast().reduce(0.0) { $0 + $1.1[j] } / Double(simplex.count - 1)
            }
            func along(_ t: Double, _ x: [Double]) -> [Double] {
                clip((0..<d).map { c[$0] + t * (x[$0] - c[$0]) })
            }
            let (fw, xw) = simplex[simplex.count - 1]
            let xr = along(-1.0, xw)
            let fr = try f(xr)
            if fr < simplex[0].0 {
                let xe = along(-2.0, xw)
                let fe = try f(xe)
                simplex[simplex.count - 1] = fe < fr ? (fe, xe) : (fr, xr)
            } else if fr < simplex[simplex.count - 2].0 {
                simplex[simplex.count - 1] = (fr, xr)
            } else {
                let xc = along(0.5, fr < fw ? xr : xw)
                let fc = try f(xc)
                if fc < min(fr, fw) {
                    simplex[simplex.count - 1] = (fc, xc)
                } else {
                    // Shrink towards the best, re-measured.
                    let b = simplex[0].1
                    let head = ((simplex[0].0 + (try f(b))) / 2, b)
                    var next: [(Double, [Double])] = [head]
                    for (_, x) in simplex.dropFirst() {
                        let y = (0..<d).map { b[$0] + 0.5 * (x[$0] - b[$0]) }
                        next.append((try f(y), y))
                    }
                    simplex = next
                }
            }
        }
    }

    /// One trial at unit-cube point x; returns its score toY(cost).
    private func f(_ x: [Double]) throws -> Double {
        if k >= cfg.trials { throw StopError.stopped }
        let u = clip(x)
        return try trial(candidates(from: u))
    }

    // MARK: Random (Latin-hypercube baseline)

    private func searchRandom() throws {
        let d = cfg.axis.gainNames.count
        let points = latinHypercube(count: max(2, cfg.trials - k), d: d)
        for x in points {
            try checkStop()
            _ = try f(x)
        }
    }
}
