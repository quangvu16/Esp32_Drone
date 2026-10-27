//
//  FlightViewController.swift
//  Crazyflie client
//
//  Flight-mode screen (PROTOCOL.md §5). Manual keeps the existing stick/tilt
//  control on the main screen; this screen owns Hold / Auto / Follow / RTH,
//  the arm-disarm-takeoff-land commands and the hover-thrust trim.
//
//  The board has only an MPU6500 — no barometer, ToF, GPS or magnetometer —
//  so the UI says what each mode really does instead of implying altitude
//  hold or GPS tracking.
//

import UIKit

final class FlightViewController: UIViewController {

    private let client: CrtpClient
    private let telemetry: TelemetryStore

    private let modeControl = UISegmentedControl(items: ["Manual", "Hold", "Auto", "Follow", "RTH"])
    private let armButton = UIButton(type: .system)
    private let disarmButton = UIButton(type: .system)
    private let takeoffButton = UIButton(type: .system)
    private let landButton = UIButton(type: .system)
    private let emergencyButton = UIButton(type: .system)
    private let hoverSlider = UISlider()
    private let hoverLabel = UILabel()
    private let statusLabel = UILabel()
    private let autoStateLabel = UILabel()
    private let batteryLabel = UILabel()
    private let noticeLabel = UILabel()

    /// Two-step emergency: the first tap arms the button for 3 seconds.
    private var emergencyPrimed = false

    private var listener: ((LogSample) -> Void)?

    init(client: CrtpClient, telemetry: TelemetryStore) {
        self.client = client
        self.telemetry = telemetry
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not supported")
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = "Flight"
        navigationItem.leftBarButtonItem = UIBarButtonItem(
            barButtonSystemItem: .done, target: self, action: #selector(close))

        let stack = UIStackView()
        stack.axis = .vertical
        stack.spacing = 14
        stack.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(stack)
        NSLayoutConstraint.activate([
            stack.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 16),
            stack.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            stack.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16),
        ])

        modeControl.addTarget(self, action: #selector(modeChanged), for: .valueChanged)
        stack.addArrangedSubview(modeControl)

        let actions = UIStackView(arrangedSubviews: [armButton, disarmButton, takeoffButton, landButton])
        actions.axis = .horizontal
        actions.distribution = .fillEqually
        actions.spacing = 8
        for (button, title) in [(armButton, "Arm"), (disarmButton, "Disarm"),
                                (takeoffButton, "Takeoff"), (landButton, "Land")] {
            button.setTitle(title, for: .normal)
            button.titleLabel?.font = .systemFont(ofSize: 17, weight: .semibold)
            button.heightAnchor.constraint(equalToConstant: 48).isActive = true
        }
        armButton.addTarget(self, action: #selector(arm), for: .touchUpInside)
        disarmButton.addTarget(self, action: #selector(disarm), for: .touchUpInside)
        takeoffButton.addTarget(self, action: #selector(takeoff), for: .touchUpInside)
        landButton.addTarget(self, action: #selector(land), for: .touchUpInside)
        stack.addArrangedSubview(actions)

        emergencyButton.setTitle("EMERGENCY — hold to confirm", for: .normal)
        emergencyButton.setTitleColor(.white, for: .normal)
        emergencyButton.backgroundColor = .systemRed
        emergencyButton.titleLabel?.font = .systemFont(ofSize: 18, weight: .bold)
        emergencyButton.layer.cornerRadius = 10
        emergencyButton.heightAnchor.constraint(equalToConstant: 64).isActive = true
        emergencyButton.addTarget(self, action: #selector(emergencyTapped), for: .touchUpInside)
        stack.addArrangedSubview(emergencyButton)

        hoverLabel.font = .monospacedDigitSystemFont(ofSize: 14, weight: .regular)
        hoverLabel.textColor = .secondaryLabel
        hoverSlider.minimumValue = 8000
        hoverSlider.maximumValue = 55000
        hoverSlider.value = Float(telemetry.value("flight.hover_thrust") ?? 30000)
        hoverSlider.addTarget(self, action: #selector(hoverChanged), for: .valueChanged)
        stack.addArrangedSubview(hoverLabel)
        stack.addArrangedSubview(hoverSlider)
        updateHoverLabel()

        statusLabel.font = .monospacedSystemFont(ofSize: 15, weight: .semibold)
        autoStateLabel.font = .monospacedSystemFont(ofSize: 13, weight: .regular)
        autoStateLabel.textColor = .secondaryLabel
        batteryLabel.font = .monospacedDigitSystemFont(ofSize: 13, weight: .regular)
        batteryLabel.textColor = .secondaryLabel
        stack.addArrangedSubview(statusLabel)
        stack.addArrangedSubview(autoStateLabel)
        stack.addArrangedSubview(batteryLabel)

        noticeLabel.numberOfLines = 0
        noticeLabel.font = .systemFont(ofSize: 12)
        noticeLabel.textColor = .secondaryLabel
        noticeLabel.text = """
        Hardware limits (PROTOCOL.md §5):
        • Hold holds a thrust level, not an altitude — no barometer/ToF on this board. Hover height drifts as the battery sags.
        • Auto is takeoff → hover → land, not waypoint flight.
        • Follow / RTH use accelerometer dead reckoning. Approximate — drifts metres per minute.
        • Heading is gyro-integrated; heading hold drifts slowly over a flight.
        """
        stack.addArrangedSubview(noticeLabel)

        let listener: (LogSample) -> Void = { [weak self] _ in
            self?.refreshStatus()
        }
        self.listener = listener
        telemetry.listeners.append(listener)
        refreshStatus()
    }

    deinit {
        if let listener = listener {
            telemetry.listeners.removeAll { $0 as AnyObject === listener as AnyObject }
        }
    }

    // MARK: - Telemetry

    private func refreshStatus() {
        let mode = Int(telemetry.value("flight.mode") ?? 0)
        let modeName = ["MANUAL", "HOLD", "AUTO", "FOLLOW", "RTH"][min(mode, 4)]
        let armed = (telemetry.value("flight.armed") ?? 0) > 0.5
        statusLabel.text = "mode \(modeName)   \(armed ? "ARMED" : "disarmed")"
        statusLabel.textColor = armed ? .systemRed : .label

        let autoState = Int(telemetry.value("flight.auto_state") ?? 0)
        autoStateLabel.text = "auto sequence: " +
            ["idle", "takeoff", "hold", "land"][min(autoState, 3)]

        if let vbat = telemetry.value("pm.vbat") {
            batteryLabel.text = String(format: "battery %.2f V", vbat)
            batteryLabel.textColor = vbat < 3.3 ? .systemRed : .secondaryLabel
        }
    }

    // MARK: - Actions

    @objc private func close() {
        dismiss(animated: true)
    }

    @objc private func modeChanged() {
        // PROTOCOL.md §5 pins 0..4 on SET_MODE (MANUAL…RTH). The firmware's
        // mode ceiling is MODE_RTH, so all five are accepted.
        guard let mode = CrtpClient.FlightMode(rawValue: UInt8(modeControl.selectedSegmentIndex)) else { return }
        client.setFlightMode(mode)
    }

    @objc private func arm() {
        client.arm()
    }

    @objc private func disarm() {
        client.disarm()
    }

    @objc private func takeoff() {
        client.takeoff(hoverThrust: UInt16(hoverSlider.value))
    }

    @objc private func land() {
        client.land()
    }

    @objc private func emergencyTapped() {
        if emergencyPrimed {
            emergencyPrimed = false
            emergencyButton.setTitle("EMERGENCY — hold to confirm", for: .normal)
            emergencyButton.backgroundColor = .systemRed
            // Props off now — this is the panic path, so no confirm dialog.
            client.emergency()
            let generator = UINotificationFeedbackGenerator()
            generator.notificationOccurred(.error)
        } else {
            emergencyPrimed = true
            emergencyButton.setTitle("TAP AGAIN — cut motors NOW", for: .normal)
            emergencyButton.backgroundColor = .systemOrange
            DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in
                guard let self = self, self.emergencyPrimed else { return }
                self.emergencyPrimed = false
                self.emergencyButton.setTitle("EMERGENCY — hold to confirm", for: .normal)
                self.emergencyButton.backgroundColor = .systemRed
            }
        }
    }

    @objc private func hoverChanged() {
        updateHoverLabel()
        client.setHoverThrust(UInt16(hoverSlider.value))
    }

    private func updateHoverLabel() {
        hoverLabel.text = String(format: "hover thrust %d  (8000…55000, feeds Takeoff)", Int(hoverSlider.value))
    }
}
