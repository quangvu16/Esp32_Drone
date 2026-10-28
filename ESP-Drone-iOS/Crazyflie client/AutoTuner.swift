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
//  ML / Twiddle / Nelder-Mead / Random were the first four. The rest of
//  ml_tune.py's SearchTuner family is ported too (Neural, RL, CMA-ES, PSO,
//  Genetic, Differential evolution, SPSA) so the UI can compare against them;
//  each searches the same unit cube, scores a trial as toY(cost) and reports
//  the result as ML does — the tried set with the lowest *predicted* cost.
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

    /// The surviving-window score, identical to the Android app's `costOf` and
    /// `drone.cost_of` in tools/pid-tune so a run means the same thing
    /// whichever tool produced it. The wheel-encoder travel term is dropped —
    /// a quad has no wheels. The reference's fall cost is applied in
    /// `measure()` instead, because it needs to know how much of the trial
    /// was lost.
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
    /// Motor output in µs (motor.m1..m4), as the firmware writes them.
    let motorMicros: [Double]
    /// Controller output for the active axis (controller.roll / .pitch / .yaw).
    let controller: Double
    /// Level angles from the status block, for the tip check. Past
    /// `AutoTuner.tipDeg` on either axis the drone has lost control.
    let roll: Double
    let pitch: Double
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

extension GaussianProcess: CostModel {}

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

/// The optimizers. `tag` starts the log's stage column, as on the PC — the
/// strings match ml_tune.py's `method` where it has one.
enum TuneMethod: Int, CaseIterable {
    case ml = 0
    case twiddle
    case nelderMead
    case random
    case neural
    case rl
    case cmaes
    case pso
    case ga
    case de
    case spsa

    var title: String {
        switch self {
        case .ml: return "ML (Bayesian)"
        case .twiddle: return "Twiddle"
        case .nelderMead: return "Nelder-Mead"
        case .random: return "Random"
        case .neural: return "Neural (deep ensemble)"
        case .rl: return "RL (policy gradient)"
        case .cmaes: return "CMA-ES"
        case .pso: return "Particle swarm (PSO)"
        case .ga: return "Genetic algorithm"
        case .de: return "Differential evolution"
        case .spsa: return "SPSA"
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
        case .neural:
            return "5 small neural nets learn the cost; tries the lowest confidence bound."
        case .rl:
            return "PGPE: a Gaussian policy over the gains, 2 trials per update."
        case .cmaes:
            return "Evolution strategy that learns the step size and gain correlations."
        case .pso:
            return "Swarm search. Needs 30+ trials."
        case .ga:
            return "Selection, crossover, mutation, elitism. Needs 30+ trials."
        case .de:
            return "Population search, DE/rand/1/bin. Needs 30+ trials."
        case .spsa:
            return "Noisy gradient from 2 trials per step (extremum-seeking style). Local."
        }
    }

    /// ml_tune.py's `method` strings; the trial log's stage column starts with it.
    var tag: String {
        switch self {
        case .ml: return "ml"
        case .twiddle: return "twiddle"
        case .nelderMead: return "nelder-mead"
        case .random: return "random"
        case .neural: return "neural"
        case .rl: return "rl"
        case .cmaes: return "cma-es"
        case .pso: return "pso"
        case .ga: return "ga"
        case .de: return "de"
        case .spsa: return "spsa"
        }
    }

    /// Search box as a multiplier on the starting gains.
    var defaultRange: Double {
        self == .twiddle ? 3.0 : 2.0
    }

    /// The result is the tried set with the lowest GP-*predicted* cost
    /// (ml_tune.py's `choose_best`), not the luckiest single trial. Everything
    /// but Twiddle: BayesTuner defines the rule and `SearchTuner.choose_best`
    /// aliases it, so the whole SearchTuner family — Random and Nelder-Mead
    /// included — reports the predicted best too. Twiddle alone keeps its own
    /// best, because its confirm-a-pair rule is the point of the method.
    var usesPredictedBest: Bool {
        self != .twiddle
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

    /// What the model looks like after a trial — ml_tune.py's `post("trial", …)`
    /// payload: the fitted model (`gp`), the search space, every trial's
    /// unit-cube point and score (`X`, `Y`), which of them are this run's
    /// (`own`, `fell_mask`) and the predicted-best point (`best_x`).
    struct TrialReport {
        let n: Int
        let total: Int
        let cost: Double
        let candidate: [Double]
        let best: [Double]
        let bestCost: Double
        // The payload's model fields, for ModelView.
        let method: TuneMethod
        let space: TuneSpace
        let xs: [[Double]]          // X
        let ys: [Double]            // Y, in toY space
        let costs: [Double]         // fromY(ys) — what ModelView plots
        let own: [Bool]
        let fellMask: [Bool]
        let bestX: [Double]?        // best_x
        let bestPred: Double?       // best_pred, cost units
        let model: CostModel?       // gp: always the fitted GaussianProcess here
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
    private var fell: [Bool] = []            // did this run's trial tip the drone?
    private var k = 0
    private var stage = ""
    private var model: GaussianProcess?
    private var bestX: [Double]?
    private var bestPred: Double?

    /// SearchTuner family state (ml_tune.py: d and x0).
    private var d = 0
    private var x0: [Double] = []

    private var logHandle: FileHandle?

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

    /// Thrown by `f` when the trial budget is used up — ml_tune.py's `Budget`,
    /// which the endless SearchTuner loops end on (a normal run's end).
    private struct Budget: Error {}

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
    // Cost-model constants, shared with the Android app and tools/pid-tune so a
    // tuning run scores the same wherever it is run.
    static let tipDeg = 25.0            // the firmware's MAX_TILT_DEG
    static let motorCentre = 1500.0     // µs midpoint of the 1000..2000 convention
    static let motorSatHi = 1990.0
    static let motorSatLo = 1010.0

    /// The drone has lost control. On a stand this flags wild oscillation; in
    /// the air it flags an actual fall.
    private func tipped(_ f: TuneSample) -> Bool {
        abs(f.roll) > Self.tipDeg || abs(f.pitch) > Self.tipDeg
    }

    /// One trial with the parameters already applied. Settle → collect → score.
    /// The trial ends early the moment the drone tips.
    private func measure() throws -> (cost: Double, metrics: TuneMetrics, fell: Bool) {
        Thread.sleep(forTimeInterval: cfg.settleSeconds)
        stateLock.lock()
        pendingSamples.removeAll()
        stateLock.unlock()

        let t0 = now
        var window = cfg.trialSeconds
        while now.timeIntervalSince(t0) < window {
            try checkStop()
            Thread.sleep(forTimeInterval: 0.1)
            if drainSamples(since: t0).contains(where: tipped) {
                window = 0                // the drone is over; stop collecting
            }
        }

        let frames = drainSamples(since: t0)
        guard !frames.isEmpty else {
            status("No telemetry — is the tune log block running?")
            throw StopError.stopped
        }

        // Score only the window before the tip, as the other tools do.
        let badAt = frames.firstIndex(where: tipped)
        let ok = badAt.map { Array(frames[..<$0]) } ?? frames
        let good = ok.count >= 5 ? ok : Array(frames.prefix(5))
        let n = Double(good.count)

        let angles = good.map(\.angle)
        let mean = angles.reduce(0, +) / n
        let angleStd = sqrt(angles.reduce(0) { $0 + ($1 - mean) * ($1 - mean) } / n)

        // Gyro-rate RMS: finite difference of the angle samples. For the yaw
        // axis the log carries a rate already (stabilizer.yaw is deg/s), but
        // the same formula still gives a usable magnitude.
        var gyroSq = 0.0
        if good.count > 1 {
            let dt = max(1e-3, cfg.trialSeconds / Double(good.count))
            for i in 1..<good.count {
                let rate = (good[i].angle - good[i - 1].angle) / dt
                gyroSq += rate * rate
            }
        }
        let gyroRms = good.count > 1 ? sqrt(gyroSq / Double(good.count - 1)) : 0

        // Motor RMS about the 1500 µs midpoint. Squaring the raw µs would add
        // a flat 1500 to every cost, clamp them all at COST_CAP and leave the
        // model with nothing to learn.
        let motorSq = good.reduce(0.0) { acc, f in
            let m = f.motorMicros.reduce(0, +) / Double(max(1, f.motorMicros.count))
                - Self.motorCentre
            return acc + m * m
        }
        let motorRms = sqrt(motorSq / n)

        // Mean |output|, as the other tools score it — not an RMS.
        let controlEffort = good.reduce(0.0) { $0 + abs($1.controller) } / n

        // Saturation at the rails the apps watch.
        let sat = good.reduce(0) { acc, f in
            let hot = f.motorMicros.contains { $0 >= Self.motorSatHi || $0 <= Self.motorSatLo }
            return acc + (hot ? 1 : 0)
        }
        let satShare = Double(sat) / n

        let metrics = TuneMetrics(
            angleMean: mean,
            angleStd: angleStd,
            gyroRms: gyroRms,
            motorRms: motorRms,
            controlEffort: controlEffort,
            satShare: satShare,
            sampleCount: good.count
        )

        var cost = metrics.cost
        if let badAt = badAt {
            // The reference's fall cost: without it a short pre-tip window can
            // outscore a clean full trial and the model learns to tip.
            let survived = Double(badAt) / Double(max(frames.count, 1)) * cfg.trialSeconds
            cost = 100 + 10 * (cfg.trialSeconds - survived)
        }
        return (cost, metrics, badAt != nil)
    }

    /// Evaluate one candidate: apply, measure, bookkeep. Returns the **raw**
    /// cost and whether the drone tipped (ml_tune.py's `trial(p, x)`), so the
    /// searches compare costs in one space; only the model is fitted on
    /// `toY(cost)`. `x` is the unit-cube point the search asked for, used only
    /// when the firmware's round-trip lands outside the box.
    private func trial(_ p: [Double], x: [Double]? = nil) throws -> (cost: Double, fell: Bool) {
        try checkStop()
        try apply(p)
        let (cost, metrics, fellNow) = try measure()
        k += 1

        // What the trial really flew, before a fall puts the best values back.
        let tried = params
        // The firmware may have clamped/rounded the values: learn what was
        // really applied (ml_tune.py's `tried`).
        if let u = space.toUnit(tried) ?? x {
            xs.append(u)
            ys.append(toY(cost))
            own.append(true)
            fell.append(fellNow)
        }
        logTrial(stage: stage, params: tried, cost: cost, metrics: metrics, fell: fellNow)
        if fellNow {
            // The drone tipped: put the best gains back before the next trial,
            // as the Android app does.
            try? apply(bestParams)
        }

        // Model bookkeeping shared by all methods (ml_tune.py's trial).
        // Note `trial` never writes `bestParams`/`bestCost` — `chooseBest` does,
        // so the winner is never a lucky single trial.
        model = xs.isEmpty ? nil : GaussianProcess(xs: xs, y: ys)
        if cfg.method == .twiddle {
            // Twiddle owns `best` and moves it only on a confirmed pair; here we
            // just mirror it into the window (ml_tune.py's TwiddleTuner.choose_best).
            bestX = space.toUnit(bestParams)
            bestPred = bestCost.isFinite ? bestCost : nil
        } else {
            chooseBestPredicted()
        }

        let report = TrialReport(n: k, total: cfg.trials,
                                 cost: cost, candidate: p,
                                 best: bestParams, bestCost: bestCost,
                                 method: cfg.method, space: space,
                                 xs: xs, ys: ys, costs: ys.map { fromY($0) },
                                 own: own, fellMask: fell,
                                 bestX: bestX, bestPred: bestPred, model: model)
        DispatchQueue.main.async { [weak self] in
            self?.listener?.tunerTrial(report)
        }
        return (cost, fellNow)
    }

    /// The tried set (this run, did not fall) with the lowest predicted cost —
    /// ml_tune.py's BayesTuner.choose_best, which `SearchTuner` aliases for the
    /// whole family. It becomes the applied best, so a lucky trial never wins
    /// on its own. Only Twiddle skips this and keeps its own pair-mean best.
    private func chooseBestPredicted() {
        guard let model = model else { return }
        var means: [Double] = []
        for x in xs {
            means.append(model.predict(x).mean)
        }
        let good = own.indices.filter { own[$0] && !fell[$0] }
        guard let j = good.min(by: { means[$0] < means[$1] }) else { return }
        bestX = xs[j]
        let pred = fromY(means[j])
        bestPred = pred
        bestCost = pred
        bestParams = candidates(from: xs[j])
    }

    /// log1p min-cost transform — matches AutoTuner.kt's `toY`.
    private func toY(_ cost: Double) -> Double {
        log1p(min(cost, 300))
    }

    /// The inverse — the cost a toY score stands for (ml_tune.py's `from_y`).
    private func fromY(_ y: Double) -> Double {
        expm1(y)
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
            closeTrialLog()
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
            openTrialLog()
            begin()
            switch cfg.method {
            case .ml:
                try searchML()
            case .twiddle:
                try searchTwiddle()
            case .nelderMead:
                try searchNelderMead()
            case .random:
                try searchRandom()
            case .neural:
                try searchNeural()
            case .rl:
                try searchPolicyGradient()
            case .cmaes:
                try searchCmaes()
            case .pso:
                try searchPso()
            case .ga:
                try searchGenetic()
            case .de:
                try searchDifferentialEvolution()
            case .spsa:
                try searchSpsa()
            }
        } catch StopError.stopped {
            reason = "stopped"
        } catch is Budget {
            // Trial budget used up inside a search loop: a normal end.
        } catch {
            reason = "error: \(error.localizedDescription)"
        }
        if reason == "completed" {
            reason = k < cfg.trials ? "converged" : "trial budget used"
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
        // `trial` hands back a raw cost, so every comparison below is in one
        // space — the same space the status line prints. (ml_tune.py's
        // TwiddleTuner.search keeps `best_cost` raw the same way.)
        let c0 = try trial(bestP).cost
        let c1 = try trial(bestP).cost
        bestCost = (c0 + c1) / 2
        bestPred = bestCost

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

                    let r = try trial(cand)
                    if r.fell || r.cost >= bestCost || k >= cfg.trials { continue }
                    // Confirm: noise must not win.
                    let r2 = try trial(cand)
                    if !r2.fell && (r.cost + r2.cost) / 2 < bestCost {
                        bestP = cand
                        bestCost = (r.cost + r2.cost) / 2
                        bestParams = bestP
                        bestPred = bestCost
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

    /// One trial at unit-cube point x; returns its score toY(cost). Raises
    /// `Budget` when the trials are used up.
    private func f(_ x: [Double]) throws -> Double {
        if k >= cfg.trials { throw Budget() }
        let u = clip(x)
        return toY(try trial(candidates(from: u), x: u).cost)
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

    // MARK: - History seeding + trial log (ml_tune.py's seed/read_history)

    /// One persisted trial row: the axis gains, the cost, whether the drone
    /// tipped during the trial.
    private struct HistoryRow {
        let values: [Double]     // kp, ki, kd
        let cost: Double
        let fell: Bool
    }

    private static let logHeader =
        "trial,stage,axis,kp,ki,kd,cost,fell,angle_mean,angle_std,gyro_rms,motor_rms,control_effort,sat"

    /// ml_tune.py's ml_tune_log.csv: trial rows kept across runs.
    private static var trialLogURL: URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("ml_tune_log.csv")
    }

    /// Search box around the current gains, seeded with matching past trials
    /// (ml_tune.py's begin). The stage names the method and the tuned gains,
    /// as on the PC.
    private func begin() {
        stage = cfg.method.tag + ":" + space.names.joined(separator: "+")
        d = cfg.axis.gainNames.count
        x0 = space.toUnit(params) ?? [Double](repeating: 0.5, count: d)
        let (seedX, seedY) = seed()
        xs.append(contentsOf: seedX)
        ys.append(contentsOf: seedY)
        own.append(contentsOf: [Bool](repeating: false, count: seedX.count))
        fell.append(contentsOf: [Bool](repeating: false, count: seedX.count))
        if !seedX.isEmpty {
            status("learned from \(seedX.count) past trials")
        }
    }

    /// Past trials taken with the same other gains, as unit-cube points
    /// (ml_tune.py's seed). All three of the axis's gains are tuned here, so
    /// the "other gains within 10%" check has nothing left to compare — as in
    /// ml_tune.py when every gain is ticked — and a quad has no angle0 offset,
    /// so the 1 deg balance-angle check has no counterpart. The search box is
    /// the filter that remains.
    private func seed() -> (xs: [[Double]], ys: [Double]) {
        var xs: [[Double]] = []
        var ys: [Double] = []
        for row in Self.readHistory(axis: cfg.axis) {
            if let x = space.toUnit(row.values) {
                xs.append(x)
                ys.append(toY(row.cost))
            }
        }
        return (xs, ys)
    }

    /// ml_tune.py's read_history: trial rows from the log, if it exists.
    private static func readHistory(axis: TuneAxis) -> [HistoryRow] {
        guard let text = try? String(contentsOf: trialLogURL, encoding: .utf8) else { return [] }
        var header: [String] = []
        var rows: [HistoryRow] = []
        for line in text.split(separator: "\n", omittingEmptySubsequences: true) {
            let cols = line.split(separator: ",").map(String.init)
            if header.isEmpty {
                header = cols
                continue
            }
            func col(_ name: String) -> String? {
                guard let i = header.firstIndex(of: name), i < cols.count else { return nil }
                return cols[i]
            }
            guard col("axis") == axis.title.lowercased(),
                  let kp = col("kp").flatMap(Double.init),
                  let ki = col("ki").flatMap(Double.init),
                  let kd = col("kd").flatMap(Double.init),
                  let cost = col("cost").flatMap(Double.init) else { continue }
            rows.append(HistoryRow(values: [kp, ki, kd], cost: cost,
                                   fell: col("fell") == "True"))
        }
        return rows
    }

    private func openTrialLog() {
        let url = Self.trialLogURL
        if !FileManager.default.fileExists(atPath: url.path) {
            try? (Self.logHeader + "\n").write(to: url, atomically: true, encoding: .utf8)
        }
        logHandle = try? FileHandle(forWritingTo: url)
        _ = try? logHandle?.seekToEnd()
    }

    /// One log row per trial (ml_tune.py's evaluate), flushed.
    private func logTrial(stage: String, params: [Double], cost: Double, metrics: TuneMetrics,
                          fell: Bool) {
        guard let h = logHandle else { return }
        let row = ([String(k), stage, cfg.axis.title.lowercased()]
            + params.map { String(format: "%.5g", $0) }
            + [String(format: "%.3f", cost), fell ? "True" : "False",
               String(format: "%.4g", metrics.angleMean),
               String(format: "%.4g", metrics.angleStd),
               String(format: "%.4g", metrics.gyroRms),
               String(format: "%.4g", metrics.motorRms),
               String(format: "%.4g", metrics.controlEffort),
               String(format: "%.4g", metrics.satShare)])
            .joined(separator: ",") + "\n"
        if let data = row.data(using: .utf8) {
            try? h.write(contentsOf: data)
        }
    }

    private func closeTrialLog() {
        try? logHandle?.close()
        logHandle = nil
    }

    // MARK: - ml_tune.py's SearchTuner family
    // Each search*() is an endless loop over unit-cube points; f() raises
    // Budget when the trials are used up, and trial() applies choose_best
    // (the tried set with the lowest GP-predicted cost) after every trial.

    /// Population for the evolutionary methods: 4 + 2 per gain, but at least 3 generations.
    private func popSize() -> Int {
        max(4, min(4 + 2 * d, (cfg.trials - k) / 3))
    }

    /// ml_tune.py's first_population: the start point plus a Latin hypercube.
    private func firstPopulation(_ n: Int) -> [[Double]] {
        [x0] + latinHypercube(count: n - 1, d: d)
    }

    private func mean<S: Sequence>(_ v: S) -> Double where S.Element == Double {
        var s = 0.0
        var n = 0
        for x in v {
            s += x
            n += 1
        }
        return s / Double(n)
    }

    private func popStd(_ v: [Double]) -> Double {
        let m = mean(v)
        return (v.reduce(0) { $0 + ($1 - m) * ($1 - m) } / Double(v.count)).squareRoot()
    }

    /// ml_tune.py's NeuralTuner: seed, then fit an MlpEnsemble on every trial
    /// so far and run the candidate minimising its lower confidence bound.
    private func searchNeural() throws {
        for x in firstPopulation(max(1, d + 3 - xs.count)) {
            _ = try f(x)
        }
        while true {
            try checkStop()
            let net = MlpEnsemble(xs: xs, y: ys, rng: &rng)
            var cand: [[Double]] = (0..<3000).map { _ in
                (0..<d).map { _ in Double.random(in: 0..<1, using: &rng) }
            }
            if let b = bestX {
                cand += (0..<600).map { _ in
                    b.map { min(1, max(0, $0 + 0.07 * rng.nextGaussian())) }
                }
            }
            var pick = cand[0]
            var bestLcb = Double.infinity
            for c in cand {
                let (mean, std) = net.predict(c)
                let lcb = mean - 1.5 * std
                if lcb < bestLcb {
                    bestLcb = lcb
                    pick = c
                }
            }
            _ = try f(pick)
        }
    }

    /// ml_tune.py's PolicyGradientTuner: PGPE (Sehnke et al. 2010), a
    /// policy-gradient over the gains themselves. The policy is a Gaussian over
    /// the unit cube; each step plays the symmetric pair mu ± eps (one trial
    /// each, reward = -score) and moves the mean towards the better one and the
    /// widths towards what paid off.
    private func searchPolicyGradient() throws {
        var mu = x0
        var sig = [Double](repeating: 0.15, count: d)
        var rewards: [Double] = []
        while true {
            try checkStop()
            let eps = sig.map { $0 * rng.nextGaussian() }
            let rPlus = -(try f(zip(mu, eps).map { $0 + $1 }))
            let rMinus = -(try f(zip(mu, eps).map { $0 - $1 }))
            rewards.append(rPlus)
            rewards.append(rMinus)
            let scale = max(popStd(rewards), 0.05)
            let base = rewards.count > 2 ? mean(rewards.dropLast(2)) : (rPlus + rMinus) / 2
            for i in 0..<d {
                let dm = min(0.15, max(-0.15, 0.5 * eps[i] * (rPlus - rMinus) / (2 * scale)))
                mu[i] = min(1, max(0, mu[i] + dm))
                sig[i] = min(0.3, max(0.03, sig[i]
                    + 0.1 * ((rPlus + rMinus) / 2 - base) / scale
                    * (eps[i] * eps[i] - sig[i] * sig[i]) / sig[i]))
            }
        }
    }

    /// ml_tune.py's CmaesTuner: Hansen's (mu/mu_w, lambda) CMA-ES, sigma0 0.2
    /// of the box; the clipped steps are the ones learned from, and sigma is
    /// clamped to at most 0.5.
    private func searchCmaes() throws {
        let dd = Double(d)
        let lam = 4 + Int(3 * log(dd))
        let mu = lam / 2
        var w = (1...mu).map { log(Double(mu) + 0.5) - log(Double($0)) }
        let wSum = w.reduce(0, +)
        w = w.map { $0 / wSum }
        let mueff = 1 / w.reduce(0) { $0 + $1 * $1 }
        let cc = (4 + mueff / dd) / (dd + 4 + 2 * mueff / dd)
        let cs = (mueff + 2) / (dd + mueff + 5)
        let c1 = 2 / (pow(dd + 1.3, 2) + mueff)
        let cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / (pow(dd + 2, 2) + mueff))
        let damps = 1 + 2 * max(0, ((mueff - 1) / (dd + 1)).squareRoot() - 1) + cs
        let chi = dd.squareRoot() * (1 - 1 / (4 * dd) + 1 / (21 * dd * dd))
        var m = x0
        var sigma = 0.2
        var cov = (0..<d).map { i in (0..<d).map { j in i == j ? 1.0 : 0.0 } }
        var pc = [Double](repeating: 0, count: d)
        var ps = [Double](repeating: 0, count: d)
        var gen = 0
        while true {
            gen += 1
            let (vals, b) = symmetricEigen(cov)
            let sd = vals.map { max($0, 1e-12).squareRoot() }
            var xsGen: [[Double]] = []
            for _ in 0..<lam {
                let z = (0..<d).map { _ in rng.nextGaussian() }
                var y = [Double](repeating: 0, count: d)
                for i in 0..<d {
                    for j in 0..<d {
                        y[i] += b[i][j] * sd[j] * z[j]
                    }
                }
                xsGen.append(clip((0..<d).map { m[$0] + sigma * y[$0] }))
            }
            // The clipped steps are the ones learned from.
            let steps = xsGen.map { x in (0..<d).map { (x[$0] - m[$0]) / sigma } }
            var scores: [Double] = []
            for x in xsGen {
                scores.append(try f(x))
            }
            let order = scores.indices.sorted { scores[$0] < scores[$1] }.prefix(mu)
            var yw = [Double](repeating: 0, count: d)
            for (r, j) in order.enumerated() {
                for i in 0..<d {
                    yw[i] += w[r] * steps[j][i]
                }
            }
            for i in 0..<d {
                m[i] += sigma * yw[i]
            }
            // ps = (1 - cs) ps + sqrt(cs (2 - cs) mueff) C^(-1/2) yw
            var bt = [Double](repeating: 0, count: d)
            for j in 0..<d {
                for i in 0..<d {
                    bt[j] += b[i][j] * yw[i]
                }
            }
            var whitened = [Double](repeating: 0, count: d)
            for i in 0..<d {
                for j in 0..<d {
                    whitened[i] += b[i][j] * bt[j] / sd[j]
                }
            }
            for i in 0..<d {
                ps[i] = (1 - cs) * ps[i] + (cs * (2 - cs) * mueff).squareRoot() * whitened[i]
            }
            let psNorm = ps.reduce(0) { $0 + $1 * $1 }.squareRoot()
            let hs = psNorm / (1 - pow(1 - cs, 2 * Double(gen))).squareRoot()
                < (1.4 + 2 / (dd + 1)) * chi
            for i in 0..<d {
                pc[i] = (1 - cc) * pc[i] + (hs ? (cc * (2 - cc) * mueff).squareRoot() : 0) * yw[i]
            }
            let keep = hs ? 0.0 : cc * (2 - cc)
            let old = cov
            cov = (0..<d).map { i in
                (0..<d).map { j in
                    var rank = 0.0
                    for (r, q) in order.enumerated() {
                        rank += w[r] * steps[q][i] * steps[q][j]
                    }
                    return (1 - c1 - cmu) * old[i][j] + c1 * (pc[i] * pc[j] + keep * old[i][j])
                        + cmu * rank
                }
            }
            sigma = min(0.5, sigma * exp(cs / damps * (psNorm / chi - 1)))
        }
    }

    /// ml_tune.py's PsoTuner: particle swarm (Kennedy & Eberhart 1995), each
    /// particle pulled towards its own best and the swarm's best — inertia 0.6,
    /// c1 = c2 = 1.5, speed <= 0.2 per step.
    private func searchPso() throws {
        let n = popSize()
        var pos = firstPopulation(n)
        var vel = (0..<n).map { _ in (0..<d).map { _ in 0.05 * rng.nextGaussian() } }
        var pb = pos
        var pf = [Double](repeating: .infinity, count: n)
        while true {
            for j in 0..<n {
                let fx = try f(pos[j])
                if fx < pf[j] {
                    pb[j] = pos[j]
                    pf[j] = fx
                }
            }
            let g = pb[pf.indices.min { pf[$0] < pf[$1] }!]
            let r1 = (0..<n).map { _ in (0..<d).map { _ in Double.random(in: 0..<1, using: &rng) } }
            let r2 = (0..<n).map { _ in (0..<d).map { _ in Double.random(in: 0..<1, using: &rng) } }
            for j in 0..<n {
                for q in 0..<d {
                    vel[j][q] = min(0.2, max(-0.2, 0.6 * vel[j][q]
                        + 1.5 * r1[j][q] * (pb[j][q] - pos[j][q])
                        + 1.5 * r2[j][q] * (g[q] - pos[j][q])))
                    pos[j][q] = min(1, max(0, pos[j][q] + vel[j][q]))
                }
            }
        }
    }

    /// ml_tune.py's GeneticTuner: real-coded genetic algorithm — tournament
    /// selection, BLX-0.3 blend crossover, Gaussian mutation, the best
    /// individual carried over (elitism).
    private func searchGenetic() throws {
        let n = popSize()
        var pop = firstPopulation(n)
        var fit: [Double] = []
        for x in pop {
            fit.append(try f(x))
        }
        let mutation = max(1 / Double(d), 0.3)
        while true {
            func pick() -> [Double] {
                let a = Int.random(in: 0..<n, using: &rng)
                let b = Int.random(in: 0..<n, using: &rng)
                return fit[a] < fit[b] ? pop[a] : pop[b]
            }
            var kids: [[Double]] = [pop[fit.indices.min { fit[$0] < fit[$1] }!]]
            while kids.count < n {
                let p1 = pick()
                let p2 = pick()
                var child: [Double] = []
                for j in 0..<d {
                    let lo = min(p1[j], p2[j])
                    let span = max(p1[j], p2[j]) - lo
                    var v = lo - 0.3 * span + Double.random(in: 0..<1, using: &rng) * 1.6 * span
                    if Double.random(in: 0..<1, using: &rng) < mutation {
                        v += 0.1 * rng.nextGaussian()
                    }
                    child.append(min(1, max(0, v)))
                }
                kids.append(child)
            }
            let eliteF = fit.min()!
            pop = kids
            fit = [eliteF]
            for x in kids.dropFirst() {
                fit.append(try f(x))
            }
        }
    }

    /// ml_tune.py's DifferentialEvolutionTuner: differential evolution
    /// (Storn & Price 1997), DE/rand/1/bin, F 0.7, CR 0.9 — a trial vector
    /// replaces its parent only if it scores better.
    private func searchDifferentialEvolution() throws {
        let n = popSize()
        var pop = firstPopulation(n)
        var fit: [Double] = []
        for x in pop {
            fit.append(try f(x))
        }
        while true {
            for j in 0..<n {
                var others = Array(0..<n)
                others.removeAll { $0 == j }
                others.shuffle(using: &rng)
                let (a, b, c) = (pop[others[0]], pop[others[1]], pop[others[2]])
                var cross = (0..<d).map { _ in Double.random(in: 0..<1, using: &rng) < 0.9 }
                cross[Int.random(in: 0..<d, using: &rng)] = true
                var u: [Double] = []
                for q in 0..<d {
                    let v = cross[q] ? a[q] + 0.7 * (b[q] - c[q]) : pop[j][q]
                    u.append(min(1, max(0, v)))
                }
                let fu = try f(u)
                if fu < fit[j] {
                    pop[j] = u
                    fit[j] = fu
                }
            }
        }
    }

    /// ml_tune.py's SpsaTuner: SPSA (Spall 1992) — every iteration perturbs all
    /// gains at once by ±c, two trials give a gradient estimate, and a
    /// gain-scheduled step goes downhill. Built for noisy costs.
    private func searchSpsa() throws {
        var x = x0
        var a: Double?
        let stab = 2.0
        var iter = 0
        while true {
            try checkStop()
            let ck = 0.12 / pow(Double(iter + 1), 0.101)
            let delta = (0..<d).map { _ in Bool.random(using: &rng) ? 1.0 : -1.0 }
            let diff = try f(zip(x, delta).map { $0 + ck * $1 })
                - (try f(zip(x, delta).map { $0 - ck * $1 }))
            let g = delta.map { diff / (2 * ck) * $0 }
            if a == nil {
                // First step moves ~0.1 of the box.
                a = 0.1 * pow(stab + 1, 0.602) / max(g.map { abs($0) }.max()!, 1e-6)
            }
            let step = g.map { a! / pow(Double(iter + 1) + stab, 0.602) * $0 }
            x = clip(zip(x, step).map { $0 - min(0.15, max(-0.15, $1)) })
            iter += 1
        }
    }

    /// Eigenvalues and eigenvectors (as columns) of a small symmetric matrix,
    /// cyclic Jacobi — CMA-ES's C is symmetric positive definite.
    private func symmetricEigen(_ m: [[Double]]) -> (values: [Double], vectors: [[Double]]) {
        let n = m.count
        var a = m
        var v = (0..<n).map { i in (0..<n).map { j in i == j ? 1.0 : 0.0 } }
        for _ in 0..<50 {
            var off = 0.0
            for p in 0..<n {
                for q in (p + 1)..<n {
                    off += a[p][q] * a[p][q]
                }
            }
            if off < 1e-22 { break }
            for p in 0..<n {
                for q in (p + 1)..<n {
                    if abs(a[p][q]) < 1e-300 { continue }
                    let theta = (a[q][q] - a[p][p]) / (2 * a[p][q])
                    let t = (theta >= 0 ? 1.0 : -1.0) / (abs(theta) + (theta * theta + 1).squareRoot())
                    let c = 1 / (t * t + 1).squareRoot()
                    let s = t * c
                    for k in 0..<n {
                        let kp = a[k][p]
                        let kq = a[k][q]
                        a[k][p] = c * kp - s * kq
                        a[k][q] = s * kp + c * kq
                    }
                    for k in 0..<n {
                        let pk = a[p][k]
                        let qk = a[q][k]
                        a[p][k] = c * pk - s * qk
                        a[q][k] = s * pk + c * qk
                    }
                    for k in 0..<n {
                        let kp = v[k][p]
                        let kq = v[k][q]
                        v[k][p] = c * kp - s * kq
                        v[k][q] = s * kp + c * kq
                    }
                }
            }
        }
        return ((0..<n).map { a[$0][$0] }, v)
    }
}
