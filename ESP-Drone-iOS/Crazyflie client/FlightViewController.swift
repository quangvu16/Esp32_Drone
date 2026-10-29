//
//  FlightViewController.swift
//  Crazyflie client
//
//  Flight-mode screen (PROTOCOL.md §5). Manual keeps the existing stick/tilt
//  control on the main screen; this screen owns Hold / Auto / Follow / RTH,
//  the arm-disarm-takeoff-land commands and the hover-thrust trim.
//  Màn hình chế độ bay (PROTOCOL.md §5). Manual giữ nguyên điều khiển cần/nghiêng
//  hiện có ở màn hình chính; màn hình này sở hữu Hold / Auto / Follow / RTH,
//  các lệnh arm-disarm-takeoff-land và phần tinh chỉnh hover-thrust.
//
//  The board has only an MPU6500 — no barometer, ToF, GPS or magnetometer —
//  so the UI says what each mode really does instead of implying altitude
//  hold or GPS tracking.
//  Bo mạch chỉ có MPU6500 — không có barometer, ToF, GPS hay từ kế — nên giao
//  diện nói đúng việc mỗi chế độ thực sự làm thay vì ngụ ý giữ độ cao hay
//  theo dõi GPS.
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
    /// Khẩn cấp hai bước: lần chạm đầu kích hoạt nút trong 3 giây.
    private var emergencyPrimed = false

    private var listener: ((LogSample) -> Void)?

    /// Set by the Fly screen: opens another section after this screen closes.
    /// Do màn hình Fly đặt: mở một khu vực khác sau khi màn hình này đóng.
    var onSwitchSection: ((AppSection) -> Void)?
    private var sectionHeader: SectionHeader?

    init(client: CrtpClient, telemetry: TelemetryStore) {
        self.client = client
        self.telemetry = telemetry
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not supported")
    }

    /// The Fly screen stays landscape-only (sticks on both sides, like Android's).
    /// Màn hình Bay vẫn chỉ nằm ngang (cần hai bên, như bên Android).
    override var supportedInterfaceOrientations: UIInterfaceOrientationMask { .landscapeRight }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = "Flight"
        navigationItem.leftBarButtonItem = UIBarButtonItem(
            barButtonSystemItem: .done, target: self, action: #selector(close))
        let header = SectionHeader(host: self, selected: .fly, telemetry: telemetry, twoTapCut: true) { [weak self] in
            self?.client.emergency()
        }
        header.onSwitch = onSwitchSection
        header.install()
        sectionHeader = header

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
        // Flight commands: icon AND text. A wrong guess here arms or drops the drone.
        // Icon above the text so four of them fit across a phone.
        // Lệnh bay: có cả biểu tượng VÀ chữ. Bấm nhầm ở đây là kích hoạt hoặc thả drone.
        // Biểu tượng nằm trên chữ để bốn nút vừa ngang một chiếc điện thoại.
        for (button, title, symbol) in [(armButton, "Arm", "lock.open.fill"),
                                        (disarmButton, "Disarm", "lock.fill"),
                                        (takeoffButton, "Takeoff", "airplane.departure"),
                                        (landButton, "Land", "airplane.arrival")] {
            button.applyLabel(title, symbol: symbol,
                              font: .systemFont(ofSize: 15, weight: .semibold), imageOnTop: true)
            button.heightAnchor.constraint(equalToConstant: 60).isActive = true
        }
        armButton.addTarget(self, action: #selector(arm), for: .touchUpInside)
        disarmButton.addTarget(self, action: #selector(disarm), for: .touchUpInside)
        takeoffButton.addTarget(self, action: #selector(takeoff), for: .touchUpInside)
        landButton.addTarget(self, action: #selector(land), for: .touchUpInside)
        stack.addArrangedSubview(actions)

        // Same name, colour and icon as CUT MOTORS on the GPS and tuning screens.
        // The old label said "hold to confirm", but the button has always been tap-twice.
        // Cùng tên, màu và biểu tượng với CUT MOTORS trên màn hình GPS và tinh chỉnh.
        // Nhãn cũ ghi "hold to confirm", nhưng nút này vốn luôn là chạm hai lần.
        setEmergencyLabel(primed: false)
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
        // PROTOCOL.md §5 cố định 0..4 cho SET_MODE (MANUAL…RTH). Trần chế độ của
        // firmware là MODE_RTH, nên cả năm giá trị đều được chấp nhận.
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
            setEmergencyLabel(primed: false)
            // Props off now — this is the panic path, so no confirm dialog.
            // Cánh quạt tắt ngay — đây là đường xử lý hoảng loạn, nên không có hộp thoại xác nhận.
            client.emergency()
            let generator = UINotificationFeedbackGenerator()
            generator.notificationOccurred(.error)
        } else {
            emergencyPrimed = true
            setEmergencyLabel(primed: true)
            DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in
                guard let self = self, self.emergencyPrimed else { return }
                self.emergencyPrimed = false
                self.setEmergencyLabel(primed: false)
            }
        }
    }

    @objc private func hoverChanged() {
        updateHoverLabel()
        client.setHoverThrust(UInt16(hoverSlider.value))
    }

    private func setEmergencyLabel(primed: Bool) {
        emergencyButton.applyLabel(primed ? "TAP AGAIN — CUT MOTORS NOW" : "CUT MOTORS — tap twice",
                                   symbol: "exclamationmark.triangle.fill",
                                   accessibilityText: primed ? "Tap again to cut motors now" : "Cut motors, tap twice",
                                   font: .systemFont(ofSize: 18, weight: .bold),
                                   color: .white)
        emergencyButton.backgroundColor = primed ? .systemOrange : .systemRed
    }

    private func updateHoverLabel() {
        hoverLabel.text = String(format: "hover thrust %d  (8000…55000, feeds Takeoff)", Int(hoverSlider.value))
    }
}

// MARK: - Button labels with SF Symbols

extension UIButton {
    /// Sets a button's text and SF Symbol in one place, for every screen.
    ///
    /// Flight commands pass a title AND a symbol. Routine, reversible tools pass
    /// `title: nil` to go icon-only; their words then live in `accessibilityText`,
    /// which VoiceOver reads and which must always be given for an icon-only button.
    ///
    /// Every later title change on a button set up this way must also go through here:
    /// this uses UIButton.Configuration, and mixing it with the older
    /// setTitle/titleLabel API gives unpredictable results.
    /// Đặt chữ và SF Symbol của một nút tại một nơi, cho mọi màn hình.
    ///
    /// Lệnh bay truyền cả title VÀ symbol. Các công cụ thường ngày, có thể hoàn
    /// tác truyền `title: nil` để chỉ còn biểu tượng; chữ của chúng nằm trong
    /// `accessibilityText`, thứ VoiceOver đọc và luôn phải có cho nút chỉ-có-biểu-tượng.
    ///
    /// Mọi lần đổi title về sau trên nút đã thiết lập theo cách này cũng phải đi qua
    /// đây: hàm dùng UIButton.Configuration, và trộn nó với API setTitle/titleLabel
    /// cũ sẽ cho kết quả khó lường.
    func applyLabel(_ title: String?,
                    symbol: String?,
                    accessibilityText: String? = nil,
                    font: UIFont = .systemFont(ofSize: 17, weight: .semibold),
                    color: UIColor? = nil,
                    imageOnTop: Bool = false) {
        var config = configuration ?? UIButton.Configuration.plain()
        if let title = title {
            var attributes = AttributeContainer()
            attributes.font = font
            config.attributedTitle = AttributedString(title, attributes: attributes)
        } else {
            config.attributedTitle = nil
        }
        if let symbol = symbol {
            // Returns nil for a symbol this iOS lacks; the button then keeps its text.
            // Trả về nil nếu iOS này không có symbol đó; nút khi đó giữ nguyên chữ.
            config.image = UIImage(systemName: symbol,
                                   withConfiguration: UIImage.SymbolConfiguration(font: font))
        } else {
            config.image = nil
        }
        config.imagePadding = 6
        config.imagePlacement = imageOnTop ? .top : .leading
        config.titleLineBreakMode = .byTruncatingTail
        if let color = color {
            config.baseForegroundColor = color
        }
        configuration = config
        accessibilityLabel = accessibilityText ?? title
    }
}

// MARK: - Shared screen header (redesign stage 3)

/// The six sections every screen's header offers. The order is the flight
/// story: Fly (Flight Data), Plan (Flight Plan, the Mission Planner screen),
/// Mission (fly the plan), then the tools.
/// Sáu khu mà header của mọi màn hình đều có. Thứ tự là câu chuyện bay: Fly
/// (Flight Data), Plan (Flight Plan, màn hình Mission Planner), Mission (bay
/// kế hoạch), rồi các công cụ.
enum AppSection: Int {
    case fly = 0, plan, mission, indoor, tune, setup
}

/// Puts the shared header into a pop-up screen's navigation bar: status on the left
/// (after Done), the sections in the middle, CUT MOTORS on the right — the same
/// places as on the Fly screen.
///
/// Picking another section closes this screen and asks the Fly screen to open that
/// one (`onSwitch`), so there is only ever one pop-up and Done always returns to Fly.
/// Đặt header dùng chung vào thanh điều hướng của màn hình pop-up: trạng thái bên trái
/// (sau Done), các khu ở giữa, CUT MOTORS bên phải — đúng các vị trí như trên màn hình Fly.
///
/// Chọn khu khác sẽ đóng màn hình này và nhờ màn hình Fly mở khu đó (`onSwitch`),
/// nên chỉ bao giờ có một pop-up và Done luôn quay về Fly.
final class SectionHeader: NSObject {
    private weak var host: UIViewController?
    private let selected: AppSection
    private let twoTapCut: Bool
    private let onCut: () -> Void
    private weak var telemetry: TelemetryStore?

    /// A reason this screen must stay open (e.g. a mission is flying), or nil.
    /// Lý do màn hình này phải mở (ví dụ sứ mệnh đang bay), hoặc nil.
    var leaveGuard: (() -> String?)?
    /// Stops this screen's timers before it closes for another section.
    /// Dừng các timer của màn hình này trước khi nó đóng để sang khu khác.
    var prepareToLeave: (() -> Void)?
    /// Called after this screen has closed, with the section to open.
    /// Được gọi sau khi màn hình này đã đóng, kèm khu cần mở.
    var onSwitch: ((AppSection) -> Void)?

    private let sections = UISegmentedControl(items: ["Fly", "Plan", "Mission", "Indoor", "Tune", "Setup"])
    private let statusLabel = UILabel()
    private let cutButton = UIButton(type: .system)
    private var cutPrimed = false
    private var listener: ((LogSample) -> Void)?

    private static let danger = UIColor(red: 0xB3 / 255.0, green: 0x26 / 255.0, blue: 0x1E / 255.0, alpha: 1)
    private static let armedText = UIColor(red: 0xB2 / 255.0, green: 0x6A / 255.0, blue: 0x00 / 255.0, alpha: 1)

    /// - Parameter twoTapCut: true on the Fly family (thumbs are on the sticks, so a
    ///   stray tap must not cut); false on Mission and Tune, as on Android.
    /// - Parameter twoTapCut: true cho nhóm Fly (ngón cái đang ở trên cần, nên chạm
    ///   nhầm không được cắt); false trên Mission và Tune, như trên Android.
    init(host: UIViewController, selected: AppSection, telemetry: TelemetryStore?,
         twoTapCut: Bool, onCut: @escaping () -> Void) {
        self.host = host
        self.selected = selected
        self.telemetry = telemetry
        self.twoTapCut = twoTapCut
        self.onCut = onCut
        super.init()
    }

    deinit {
        if let listener = listener {
            telemetry?.listeners.removeAll { $0 as AnyObject === listener as AnyObject }
        }
    }

    func install() {
        guard let host = host else { return }
        sections.selectedSegmentIndex = selected.rawValue
        sections.addTarget(self, action: #selector(sectionChanged), for: .valueChanged)
        host.navigationItem.titleView = sections

        statusLabel.font = .monospacedDigitSystemFont(ofSize: 13, weight: .semibold)
        statusLabel.textColor = .secondaryLabel
        var left = host.navigationItem.leftBarButtonItems ?? []
        left.append(UIBarButtonItem(customView: statusLabel))
        host.navigationItem.leftBarButtonItems = left

        var config = UIButton.Configuration.plain()
        config.contentInsets = NSDirectionalEdgeInsets(top: 4, leading: 12, bottom: 4, trailing: 12)
        cutButton.configuration = config
        cutButton.layer.cornerRadius = 8
        cutButton.addTarget(self, action: #selector(cutTapped), for: .touchUpInside)
        setCutLabel(primed: false)
        host.navigationItem.rightBarButtonItem = UIBarButtonItem(customView: cutButton)

        if let telemetry = telemetry {
            let listener: (LogSample) -> Void = { [weak self] _ in self?.refreshStatus() }
            self.listener = listener
            telemetry.listeners.append(listener)
        }
        refreshStatus()
    }

    private func refreshStatus() {
        var parts: [String] = []
        if let v = telemetry?.value("pm.vbat") {
            parts.append(String(format: "%.2f V", v))
        }
        let armed = (telemetry?.value("flight.armed") ?? 0) > 0.5
        if telemetry?.value("flight.armed") != nil {
            parts.append(armed ? "● ARMED" : "disarmed")
        }
        statusLabel.text = parts.joined(separator: "  ")
        statusLabel.textColor = armed ? SectionHeader.armedText : .secondaryLabel
        statusLabel.accessibilityLabel = parts.isEmpty ? nil : parts.joined(separator: ", ")
        statusLabel.sizeToFit()
    }

    private func setCutLabel(primed: Bool) {
        let spoken = twoTapCut
            ? (primed ? "Tap again to cut motors now" : "Cut motors, tap twice")
            : "Cut motors now"
        cutButton.applyLabel(primed ? "TAP AGAIN" : "CUT MOTORS", symbol: "exclamationmark.triangle.fill",
                             accessibilityText: spoken,
                             font: .systemFont(ofSize: 13, weight: .bold), color: .white)
        cutButton.backgroundColor = primed ? .systemOrange : SectionHeader.danger
        cutButton.sizeToFit()
    }

    @objc private func cutTapped() {
        if !twoTapCut || cutPrimed {
            cutPrimed = false
            setCutLabel(primed: false)
            UINotificationFeedbackGenerator().notificationOccurred(.error)
            onCut()
            return
        }
        cutPrimed = true
        setCutLabel(primed: true)
        DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in
            guard let self = self, self.cutPrimed else { return }
            self.cutPrimed = false
            self.setCutLabel(primed: false)
        }
    }

    @objc private func sectionChanged() {
        guard let target = AppSection(rawValue: sections.selectedSegmentIndex), target != selected else { return }
        if let reason = leaveGuard?() {
            sections.selectedSegmentIndex = selected.rawValue
            let alert = UIAlertController(title: "Can't switch yet", message: reason, preferredStyle: .alert)
            alert.addAction(UIAlertAction(title: "OK", style: .default))
            host?.present(alert, animated: true)
            return
        }
        prepareToLeave?()
        let onSwitch = self.onSwitch
        host?.dismiss(animated: true) {
            onSwitch?(target)
        }
    }
}

