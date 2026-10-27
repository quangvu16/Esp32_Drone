//
//  PidTuneViewController.swift
//  Crazyflie client
//
//  PID tuning screen (the main feature). Per-axis strip charts + per-term
//  panels + statistics + gain editors with ±10% steps + Save/Load/Defaults +
//  auto-tune behind a hold-to-confirm safety gate.
//
//  Interaction design and the ±10% / hint-line editors come from
//  PID-Calibration's MainActivity; the chart behaviour from ChartView.kt.
//  Everything on the wire is CRTP (PROTOCOL.md §3/§5), not Protocol.kt's
//  framing.
//

import UIKit

final class PidTuneViewController: UIViewController {

    private let client: CrtpClient
    private let telemetry: TelemetryStore

    // MARK: Controls

    private let axisControl = UISegmentedControl(items: TuneAxis.allCases.map(\.title))
    private let pauseButton = UIButton(type: .system)
    private let statsLabel = UILabel()

    private let angleChart = ChartView()
    private let controllerChart = ChartView()
    private let termsChart = ChartView()

    private let editorsStack = UIStackView()
    private var editorRows: [ParamEditorRow] = []

    private let saveButton = UIButton(type: .system)
    private let loadButton = UIButton(type: .system)
    private let defaultsButton = UIButton(type: .system)

    // Auto-tune
    private let methodControl = UISegmentedControl(items: TuneMethod.allCases.map(\.title))
    private let startTuneButton = UIButton(type: .system)
    private let stopTuneButton = UIButton(type: .system)
    private let tuneStatusLabel = UILabel()
    private let safetyLabel = UILabel()

    private var tuner: AutoTuner?
    private var tunerHoldStart: Date?
    private var listener: ((LogSample) -> Void)?

    /// Set by the Fly screen: opens another section after this screen closes.
    var onSwitchSection: ((AppSection) -> Void)?
    private var sectionHeader: SectionHeader?

    /// Param TOC ids we drive (PROTOCOL.md §3): 9 PID gains + 3 flight + nav.kp.
    private struct TuneParam {
        let id: UInt8
        let group: String
        let name: String
        let isFloat: Bool
        let hint: String
    }

    private let tuneParams: [TuneParam] = [
        .init(id: 3, group: "pid", name: "roll_kp", isFloat: true,
              hint: "Roll P — how hard it fights a roll error. Too high: buzzes. Too low: flops over."),
        .init(id: 4, group: "pid", name: "roll_ki", isFloat: true,
              hint: "Roll I — trims out a steady lean. Too high: slow wobble. Keep small."),
        .init(id: 5, group: "pid", name: "roll_kd", isFloat: true,
              hint: "Roll D — damps the wobble. Raise until it buzzes, back off 20%."),
        .init(id: 6, group: "pid", name: "pitch_kp", isFloat: true,
              hint: "Pitch P — same as roll P. Match the roll value unless the frame is asymmetric."),
        .init(id: 7, group: "pid", name: "pitch_ki", isFloat: true,
              hint: "Pitch I — trims out a steady lean fore/aft."),
        .init(id: 8, group: "pid", name: "pitch_kd", isFloat: true,
              hint: "Pitch D — damps pitch wobble."),
        .init(id: 9, group: "pid", name: "yaw_kp", isFloat: true,
              hint: "Yaw P — holds the heading. Too high: twitchy nose."),
        .init(id: 10, group: "pid", name: "yaw_ki", isFloat: true,
              hint: "Yaw I — trims out a steady yaw drift."),
        .init(id: 11, group: "pid", name: "yaw_kd", isFloat: true,
              hint: "Yaw D — usually 0 here; the board has no yaw damper term worth tuning."),
        .init(id: 12, group: "flight", name: "hover_thrust", isFloat: false,
              hint: "Hover thrust (µs, 8000…55000). The thrust Hold and Takeoff use."),
        .init(id: 13, group: "flight", name: "takeoff_ms", isFloat: false,
              hint: "Takeoff ramp duration in ms. Longer is gentler."),
        .init(id: 14, group: "flight", name: "land_ms", isFloat: false,
              hint: "Land ramp duration in ms."),
        .init(id: 15, group: "nav", name: "kp", isFloat: true,
              hint: "Nav P gain — how aggressively it chases a Follow/RTH target (m of error → m/s)."),
    ]

    // MARK: - Init

    init(client: CrtpClient, telemetry: TelemetryStore) {
        self.client = client
        self.telemetry = telemetry
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not supported")
    }

    deinit {
        tuner?.stop()
        if let listener = listener {
            telemetry.listeners.removeAll { $0 as AnyObject === listener as AnyObject }
        }
    }

    // MARK: - View

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = "Tune"
        navigationItem.leftBarButtonItem = UIBarButtonItem(
            barButtonSystemItem: .done, target: self, action: #selector(close))
        let header = SectionHeader(host: self, selected: .tune, telemetry: telemetry, twoTapCut: false) { [weak self] in
            self?.panicStop()
        }
        header.leaveGuard = { [weak self] in
            (self?.tuner?.isRunning ?? false)
                ? "Auto-tune is running. Stop it or use CUT MOTORS before switching."
                : nil
        }
        header.prepareToLeave = { [weak self] in
            self?.tuner?.stop()
        }
        header.onSwitch = onSwitchSection
        header.install()
        sectionHeader = header

        let scroll = UIScrollView()
        scroll.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(scroll)
        let stack = UIStackView()
        stack.axis = .vertical
        stack.spacing = 12
        stack.translatesAutoresizingMaskIntoConstraints = false
        scroll.addSubview(stack)
        NSLayoutConstraint.activate([
            scroll.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            scroll.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            scroll.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            scroll.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            stack.topAnchor.constraint(equalTo: scroll.contentLayoutGuide.topAnchor, constant: 12),
            stack.leadingAnchor.constraint(equalTo: scroll.frameLayoutGuide.leadingAnchor, constant: 12),
            stack.trailingAnchor.constraint(equalTo: scroll.frameLayoutGuide.trailingAnchor, constant: -12),
            stack.bottomAnchor.constraint(equalTo: scroll.contentLayoutGuide.bottomAnchor, constant: -12),
            stack.widthAnchor.constraint(equalTo: scroll.frameLayoutGuide.widthAnchor, constant: -24),
        ])

        // Axis picker + pause
        axisControl.selectedSegmentIndex = 0
        axisControl.addTarget(self, action: #selector(axisChanged), for: .valueChanged)
        // Chart pause is routine and reversible: icon-only, words in the VoiceOver label.
        pauseButton.applyLabel(nil, symbol: "pause.fill", accessibilityText: "Pause charts")
        pauseButton.addTarget(self, action: #selector(togglePause), for: .touchUpInside)
        let topRow = UIStackView(arrangedSubviews: [axisControl, pauseButton])
        topRow.axis = .horizontal
        topRow.spacing = 8
        stack.addArrangedSubview(topRow)

        statsLabel.numberOfLines = 0
        statsLabel.font = .monospacedDigitSystemFont(ofSize: 11, weight: .regular)
        statsLabel.textColor = .secondaryLabel
        stack.addArrangedSubview(statsLabel)

        // Charts
        configureCharts()
        stack.addArrangedSubview(chartBlock(title: "Angle (deg)", chart: angleChart))
        stack.addArrangedSubview(chartBlock(title: "Controller output", chart: controllerChart))
        stack.addArrangedSubview(chartBlock(title: "PID terms (P / I / D)", chart: termsChart))

        // Editors
        let editHeader = UILabel()
        editHeader.text = "Parameters"
        editHeader.font = .systemFont(ofSize: 17, weight: .semibold)
        stack.addArrangedSubview(editHeader)

        editorsStack.axis = .vertical
        editorsStack.spacing = 8
        stack.addArrangedSubview(editorsStack)
        rebuildEditors()

        // Save / Load / Defaults
        // These write to / read from the drone's flash, not a phone file, so they keep text.
        saveButton.applyLabel("Save gains", symbol: "tray.and.arrow.down", font: .systemFont(ofSize: 15))
        loadButton.applyLabel("Load gains", symbol: "tray.and.arrow.up", font: .systemFont(ofSize: 15))
        defaultsButton.applyLabel("Restore defaults", symbol: nil, font: .systemFont(ofSize: 15))
        saveButton.addTarget(self, action: #selector(saveGains), for: .touchUpInside)
        loadButton.addTarget(self, action: #selector(loadGains), for: .touchUpInside)
        defaultsButton.addTarget(self, action: #selector(restoreDefaults), for: .touchUpInside)
        let persistRow = UIStackView(arrangedSubviews: [saveButton, loadButton, defaultsButton])
        persistRow.axis = .horizontal
        persistRow.distribution = .fillEqually
        stack.addArrangedSubview(persistRow)

        // Auto-tune
        let tuneHeader = UILabel()
        tuneHeader.text = "Auto-tune"
        tuneHeader.font = .systemFont(ofSize: 17, weight: .semibold)
        stack.addArrangedSubview(tuneHeader)

        methodControl.selectedSegmentIndex = 0
        methodControl.addTarget(self, action: #selector(methodChanged), for: .valueChanged)
        stack.addArrangedSubview(methodControl)

        let methodHint = UILabel()
        methodHint.numberOfLines = 0
        methodHint.font = .systemFont(ofSize: 12)
        methodHint.textColor = .secondaryLabel
        methodHint.tag = 9001
        methodHint.text = TuneMethod.allCases[0].hint
        stack.addArrangedSubview(methodHint)

        safetyLabel.numberOfLines = 0
        safetyLabel.font = .systemFont(ofSize: 12, weight: .semibold)
        safetyLabel.textColor = .systemRed
        safetyLabel.text = """
        SAFETY — read before starting.
        The props WILL spin during a trial. Secure the drone on a stand or hold it tethered.
        One axis at a time. CUT MOTORS halts the tuner AND cuts the motors.
        """
        stack.addArrangedSubview(safetyLabel)

        setStartTuneLabel("Hold to confirm — start auto-tune")
        startTuneButton.backgroundColor = .systemOrange
        startTuneButton.layer.cornerRadius = 10
        startTuneButton.heightAnchor.constraint(equalToConstant: 52).isActive = true
        startTuneButton.addTarget(self, action: #selector(startTuneDown), for: .touchDown)
        startTuneButton.addTarget(self, action: #selector(startTuneUp), for: [.touchUpInside, .touchUpOutside, .touchCancel])
        stack.addArrangedSubview(startTuneButton)

        // Same name, colour and icon as CUT MOTORS on the flight and GPS screens.
        stopTuneButton.applyLabel("CUT MOTORS — stop tuner", symbol: "exclamationmark.triangle.fill",
                                  accessibilityText: "Cut motors and stop the tuner",
                                  font: .systemFont(ofSize: 16, weight: .bold), color: .white)
        stopTuneButton.backgroundColor = .systemRed
        stopTuneButton.layer.cornerRadius = 10
        stopTuneButton.heightAnchor.constraint(equalToConstant: 52).isActive = true
        stopTuneButton.addTarget(self, action: #selector(panicStop), for: .touchUpInside)
        stopTuneButton.isEnabled = false
        stack.addArrangedSubview(stopTuneButton)

        tuneStatusLabel.numberOfLines = 0
        tuneStatusLabel.font = .monospacedSystemFont(ofSize: 12, weight: .regular)
        tuneStatusLabel.textColor = .secondaryLabel
        tuneStatusLabel.text = "Tuner idle."
        stack.addArrangedSubview(tuneStatusLabel)

        // Telemetry listener
        let listener: (LogSample) -> Void = { [weak self] sample in
            self?.consume(sample)
        }
        self.listener = listener
        telemetry.listeners.append(listener)

        // Ask the client for the params once the TOC is up.
        client.onParamTocReady = { [weak self] _ in
            self?.client.readAllParams()
        }
        client.onParamValue = { [weak self] id, value in
            self?.applyParamEcho(id: id, value: value)
            self?.tuner?.ingestParam(id: id, value: value.asDouble)
        }
    }

    private func chartBlock(title: String, chart: ChartView) -> UIStackView {
        let label = UILabel()
        label.text = title
        label.font = .systemFont(ofSize: 13, weight: .semibold)
        chart.heightAnchor.constraint(equalToConstant: 160).isActive = true
        let s = UIStackView(arrangedSubviews: [label, chart])
        s.axis = .vertical
        s.spacing = 4
        return s
    }

    private func configureCharts() {
        angleChart.title = nil
        angleChart.sharedScale = false
        angleChart.unit = "°"
        angleChart.series = [
            .init(name: "angle", unit: "°", color: ChartView.color(at: 0), minRange: 5, enabled: true),
            .init(name: "target", unit: "°", color: ChartView.color(at: 1), minRange: 5, enabled: true),
        ]

        controllerChart.sharedScale = false
        controllerChart.unit = ""
        controllerChart.markers = [1000, -1000]
        controllerChart.series = [
            .init(name: "ctrl", unit: "", color: ChartView.color(at: 2), minRange: 20, enabled: true),
        ]

        termsChart.sharedScale = true
        termsChart.unit = ""
        termsChart.series = [
            .init(name: "P", unit: "", color: ChartView.color(at: 0), minRange: 1, enabled: true),
            .init(name: "I", unit: "", color: ChartView.color(at: 1), minRange: 1, enabled: true),
            .init(name: "D", unit: "", color: ChartView.color(at: 2), minRange: 1, enabled: true),
        ]
    }

    // MARK: - Editors

    private func rebuildEditors() {
        editorRows.forEach { $0.removeFromSuperview() }
        editorRows = []
        for p in tuneParams {
            let row = ParamEditorRow(
                title: "\(p.group).\(p.name)",
                hint: p.hint,
                isFloat: p.isFloat,
                onChange: { [weak self] value in
                    self?.write(p, value)
                })
            editorRows.append(row)
            editorsStack.addArrangedSubview(row)
        }
    }

    private func write(_ p: TuneParam, _ value: Double) {
        if p.isFloat {
            client.writeFloat(group: p.group, name: p.name, Float(value))
        } else {
            client.writeUInt16(group: p.group, name: p.name, UInt16(max(0, min(65535, value))))
        }
    }

    private func applyParamEcho(id: UInt8, value: CrtpValue) {
        guard let index = tuneParams.firstIndex(where: { $0.id == id }) else { return }
        let v = value.asDouble
        DispatchQueue.main.async { [weak self] in
            self?.editorRows[index].set(v)
        }
    }

    // MARK: - Telemetry → charts

    private var axis: TuneAxis {
        TuneAxis(rawValue: axisControl.selectedSegmentIndex) ?? .roll
    }

    private func consume(_ sample: LogSample) {
        let a = axis
        if let angle = sample[a.angleLogName] {
            angleChart.add([Float(angle), 0])
        }
        if let ctrl = sample[a.controllerLogName] {
            controllerChart.add([Float(ctrl)])
        }
        let pName = "pid.\(a.title.lowercased())_p"
        let iName = "pid.\(a.title.lowercased())_i"
        let dName = "pid.\(a.title.lowercased())_d"
        if let p = sample[pName], let i = sample[iName], let d = sample[dName] {
            termsChart.add([Float(p), Float(i), Float(d)])
        }
        updateStats()
    }

    private func updateStats() {
        // Stats over the visible window (the chart ring).
        // Angle series is series 0 of angleChart; we recompute from telemetry
        // history so the numbers match the log, not the drawing.
        let window = telemetry.history.suffix(300)
        let a = axis
        let angles = window.compactMap { $0[a.angleLogName] }
        guard angles.count > 2 else {
            statsLabel.text = "stats — waiting for telemetry"
            return
        }
        let mean = angles.reduce(0, +) / Double(angles.count)
        let variance = angles.reduce(0) { $0 + ($1 - mean) * ($1 - mean) } / Double(angles.count)
        let std = sqrt(variance)
        let rms = sqrt(angles.reduce(0) { $0 + $1 * $1 } / Double(angles.count))
        let lo = angles.min() ?? 0
        let hi = angles.max() ?? 0
        statsLabel.text = String(
            format: "%@  n=%d  min %.2f  max %.2f  mean %.2f  σ %.2f  RMS %.2f",
            a.title, angles.count, lo, hi, mean, std, rms)
    }

    // MARK: - Actions

    @objc private func close() {
        tuner?.stop()
        dismiss(animated: true)
    }

    @objc private func axisChanged() {
        // Recreate the tune log block for the new axis so the term panels track it.
        let a = axis
        rebuildTuneBlock(for: a)
        configureCharts()
        angleChart.clear()
        controllerChart.clear()
        termsChart.clear()
    }

    @objc private func togglePause() {
        let paused = !angleChart.paused
        angleChart.paused = paused
        controllerChart.paused = paused
        termsChart.paused = paused
        pauseButton.applyLabel(nil, symbol: paused ? "play.fill" : "pause.fill",
                               accessibilityText: paused ? "Resume charts" : "Pause charts")
    }

    @objc private func methodChanged() {
        let m = TuneMethod(rawValue: methodControl.selectedSegmentIndex) ?? .ml
        if let hint = view.viewWithTag(9001) as? UILabel {
            hint.text = m.hint
        }
    }

    @objc private func saveGains() {
        client.saveGains()
        flash("Saved to flash.")
    }

    @objc private func loadGains() {
        client.loadGains()
        client.readAllParams()
        flash("Loaded from flash.")
    }

    @objc private func restoreDefaults() {
        client.restoreDefaults()
        client.readAllParams()
        flash("Restored firmware defaults.")
    }

    private func flash(_ text: String) {
        tuneStatusLabel.text = text
    }

    // MARK: Log block for the active axis

    /// Per-axis tune block. Budget is 25 value bytes (PROTOCOL.md §4):
    /// pid.X_p/i/d (12) + stabilizer.X (4) + controller.X (4) + motor.m1 (2) = 22.
    private func rebuildTuneBlock(for axis: TuneAxis) {
        let base = axis.title.lowercased()
        let vars: [LogVariable] = [
            LogVariable(tocId: 21 + UInt8(axis.rawValue) * 3, logType: .float, name: "pid.\(base)_p"),
            LogVariable(tocId: 22 + UInt8(axis.rawValue) * 3, logType: .float, name: "pid.\(base)_i"),
            LogVariable(tocId: 23 + UInt8(axis.rawValue) * 3, logType: .float, name: "pid.\(base)_d"),
            LogVariable(tocId: 1 + UInt8(axis.rawValue), logType: .float, name: "stabilizer.\(base)"),
            LogVariable(tocId: 4 + UInt8(axis.rawValue), logType: .float, name: "controller.\(base)"),
            LogVariable(tocId: 7, logType: .uint16, name: "motor.m1"),
        ]
        let block = LogBlock(id: TelemetryStore.BlockId.tune.rawValue, variables: vars)
        guard block.fitsBudget else {
            flash("Tune block exceeds 25-byte budget")
            return
        }
        client.stopBlock(id: block.id)
        client.deleteBlock(id: block.id)
        client.createBlock(id: block.id, variables: vars, periodMs: 50) { [weak self] ok, err in
            guard ok else {
                self?.flash("Could not create tune log block: \(err.message)")
                return
            }
            self?.client.startBlock(id: block.id, periodMs: 50)
        }
    }

    // MARK: - Auto-tune safety gate

    @objc private func startTuneDown() {
        tunerHoldStart = Date()
        setStartTuneLabel("KEEP HOLDING…")
        // A 1.5 s hold is the two-step against accidental taps.
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) { [weak self] in
            guard let self = self, let t0 = self.tunerHoldStart else { return }
            if Date().timeIntervalSince(t0) >= 1.4 {
                self.confirmStart()
            }
        }
    }

    @objc private func startTuneUp() {
        tunerHoldStart = nil
        setStartTuneLabel("Hold to confirm — start auto-tune")
    }

    private func setStartTuneLabel(_ text: String) {
        startTuneButton.applyLabel(text, symbol: "play.fill",
                                   font: .systemFont(ofSize: 16, weight: .bold), color: .white)
    }

    private func confirmStart() {
        tunerHoldStart = nil
        setStartTuneLabel("Hold to confirm — start auto-tune")

        // Never start while disarmed without asking.
        let armed = (telemetry.value("flight.armed") ?? 0) > 0.5
        if !armed {
            let alert = UIAlertController(
                title: "Drone is disarmed",
                message: "Auto-tune spins the props. Arm the drone first, and make sure it is secured on a stand or held tethered.",
                preferredStyle: .alert)
            alert.addAction(UIAlertAction(title: "Cancel", style: .cancel))
            alert.addAction(UIAlertAction(title: "Arm and start", style: .destructive) { [weak self] _ in
                self?.client.arm()
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                    self?.launchTuner()
                }
            })
            present(alert, animated: true)
            return
        }

        let alert = UIAlertController(
            title: "Start auto-tune?",
            message: "Props spinning — secure the drone on a stand or hold it tethered.\n\nTuning \(axis.title) with \(TuneMethod(rawValue: methodControl.selectedSegmentIndex)?.title ?? "?").",
            preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        alert.addAction(UIAlertAction(title: "Start", style: .destructive) { [weak self] _ in
            self?.launchTuner()
        })
        present(alert, animated: true)
    }

    private func launchTuner() {
        let a = axis
        let method = TuneMethod(rawValue: methodControl.selectedSegmentIndex) ?? .ml

        // Starting gains come from the editors (the drone's current values).
        let start = editorRows.prefix(3).map(\.value)
        var cfg = AutoTuner.Config(axis: a, method: method,
                                   trials: 12,
                                   trialSeconds: 3.0,
                                   settleSeconds: 1.5,
                                   range: method.defaultRange)
        cfg.trials = 12

        // Make sure the tune block is streaming so `measure()` has data.
        rebuildTuneBlock(for: a)

        let tuner = AutoTuner(config: cfg, client: client, startGains: start)
        tuner.listener = self
        self.tuner = tuner
        tuner.start()

        startTuneButton.isEnabled = false
        stopTuneButton.isEnabled = true
        tuneStatusLabel.text = "Tuner running — \(a.title) / \(method.title)"
    }

    @objc private func panicStop() {
        // Halt the tuner AND cut the motors. This is the panic path.
        tuner?.stop()
        client.emergency()
        let generator = UINotificationFeedbackGenerator()
        generator.notificationOccurred(.error)
        startTuneButton.isEnabled = true
        stopTuneButton.isEnabled = false
        tuneStatusLabel.text = "CUT MOTORS sent — motors off, tuner halted."
    }
}

// MARK: - AutoTuner.Listener

extension PidTuneViewController: AutoTuner.Listener {
    func tunerStatus(_ text: String) {
        tuneStatusLabel.text = text
    }

    func tunerTrial(_ report: AutoTuner.TrialReport) {
        let cand = report.candidate.map { String(format: "%.4g", $0) }.joined(separator: ", ")
        let best = report.best.map { String(format: "%.4g", $0) }.joined(separator: ", ")
        tuneStatusLabel.text = String(
            format: "trial %d/%d  cost %.3f\nbest %.3f [%@]\ntried [%@]",
            report.n, report.total, report.cost,
            report.bestCost, best, cand)
    }

    func tunerFinished(reason: String, start: [Double]?, result: [Double]?) {
        startTuneButton.isEnabled = true
        stopTuneButton.isEnabled = false
        var text = "Tuner finished: \(reason)"
        if let s = start, let r = result {
            text += "\n" + zip(s, r).map { String(format: "%.4g → %.4g", $0, $1) }.joined(separator: "\n")
        }
        tuneStatusLabel.text = text
    }
}

// MARK: - Param editor row

/// Numeric field + ±10% buttons + hint line. Ported from MainActivity's
/// param editor rows: the buttons are the fast path while flying, the field
/// is for typing an exact value.
final class ParamEditorRow: UIView {

    private(set) var value: Double = 0

    private let titleLabel = UILabel()
    private let field = UITextField()
    private let minusButton = UIButton(type: .system)
    private let plusButton = UIButton(type: .system)
    private let hintLabel = UILabel()
    private let isFloat: Bool
    private let onChange: (Double) -> Void

    init(title: String, hint: String, isFloat: Bool, onChange: @escaping (Double) -> Void) {
        self.isFloat = isFloat
        self.onChange = onChange
        super.init(frame: .zero)

        titleLabel.text = title
        titleLabel.font = .monospacedSystemFont(ofSize: 14, weight: .semibold)

        field.borderStyle = .roundedRect
        field.keyboardType = .decimalPad
        field.font = .monospacedDigitSystemFont(ofSize: 15, weight: .regular)
        field.addTarget(self, action: #selector(fieldChanged), for: .editingChanged)

        minusButton.setTitle("−10%", for: .normal)
        plusButton.setTitle("+10%", for: .normal)
        minusButton.addTarget(self, action: #selector(stepMinus), for: .touchUpInside)
        plusButton.addTarget(self, action: #selector(stepPlus), for: .touchUpInside)

        hintLabel.text = hint
        hintLabel.numberOfLines = 0
        hintLabel.font = .systemFont(ofSize: 11)
        hintLabel.textColor = .secondaryLabel

        let row = UIStackView(arrangedSubviews: [titleLabel, field, minusButton, plusButton])
        row.axis = .horizontal
        row.spacing = 8
        row.distribution = .fill
        titleLabel.setContentHuggingPriority(.required, for: .horizontal)
        field.widthAnchor.constraint(equalToConstant: 90).isActive = true

        let stack = UIStackView(arrangedSubviews: [row, hintLabel])
        stack.axis = .vertical
        stack.spacing = 2
        stack.translatesAutoresizingMaskIntoConstraints = false
        addSubview(stack)
        NSLayoutConstraint.activate([
            stack.topAnchor.constraint(equalTo: topAnchor),
            stack.leadingAnchor.constraint(equalTo: leadingAnchor),
            stack.trailingAnchor.constraint(equalTo: trailingAnchor),
            stack.bottomAnchor.constraint(equalTo: bottomAnchor),
        ])
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not supported")
    }

    func set(_ v: Double) {
        value = v
        field.text = isFloat ? String(format: "%.4g", v) : String(Int(v.rounded()))
    }

    @objc private func fieldChanged() {
        guard let text = field.text, let v = Double(text) else { return }
        value = v
        onChange(v)
    }

    @objc private func stepMinus() {
        applyStep(-0.10)
    }

    @objc private func stepPlus() {
        applyStep(0.10)
    }

    private func applyStep(_ fraction: Double) {
        // ±10% of the current value — the brief's fast in-flight adjust.
        let next = value * (1 + fraction)
        value = next
        set(next)
        onChange(next)
    }
}
