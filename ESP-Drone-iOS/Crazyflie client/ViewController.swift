//
//  ViewController.swift
//  Crazyflie client
//
//  Created by Martin Eberl on 23.01.17.
//  Copyright © 2017 Bitcraze. All rights reserved.
//

import UIKit

final class ViewController: UIViewController {
    private var leftJoystick: BCJoystick?
    private var rightJoystick: BCJoystick?
    
    private var viewModel: ViewModel?
    private var settingsViewController: SettingsViewController?
    
    @IBOutlet weak var unlockLabel: UILabel!
    @IBOutlet weak var connectButton: UIButton!
    @IBOutlet weak var settingsButton: UIButton!
    @IBOutlet weak var connectProgress: UIProgressView!
    @IBOutlet weak var leftView: UIView!
    @IBOutlet weak var rightView: UIView!

    // MARK: Fly screen (redesign stage 1) — see the "Fly screen" section below.
    private let header = UIView()
    private var sectionButtons: [UIButton] = []
    private let linkIcon = UIImageView()
    private let linkStatus = UILabel()
    private let batteryStatus = UILabel()
    private let modeChip = ChipLabel()
    private let armedChip = ChipLabel()
    private let cutMotorsButton = UIButton(type: .system)
    private var cutPrimed = false

    private let flyControls = UIStackView()
    private let modeControl = UISegmentedControl(items: ["Manual", "Hold", "Auto"])
    private let primaryButton = UIButton(type: .system)
    private let primaryCaption = UILabel()
    private let thumbsHint = UILabel()
    private var telemetryValueLabels: [UILabel] = []
    private let moreButton = UIButton(type: .system)

    private let disconnectedCard = UIStackView()
    private let connectNowButton = UIButton(type: .system)
    private let connectNowProgress = UIProgressView(progressViewStyle: .default)
    private let linkCaption = UILabel()

    private let morePanel = UIView()
    private let hoverSlider = UISlider()
    private let hoverValueLabel = UILabel()
    private let yawSwitch = UISwitch()
    private let logLabel = UILabel()
    private var logLines: [String] = []

    private var telemetryListener: ((LogSample) -> Void)?
    private var wasConnected = false
    private var armedSince: Date?
    private var flightClock: Timer?
    // Joystick moves refresh this screen many times a second; only rebuild a
    // button's label when what it says actually changes.
    // Di chuyển joystick làm màn hình này refresh nhiều lần mỗi giây; chỉ dựng
    // lại nhãn nút khi nội dung nó hiển thị thực sự thay đổi.
    private var shownPrimaryArmed: Bool?
    private var shownConnectTitle: String?
    private var shownCutConnected: Bool?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        if viewModel == nil {
            viewModel = ViewModel()
            viewModel?.delegate = self
        }
        
        setupUI()
        viewModel?.updateSettings()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        viewModel?.loadSettings()
        updateUI()
    }
    
    override var prefersStatusBarHidden: Bool {
        return true
    }
    
    @IBAction func yawOnOffClick(_ sender: Any) {
//        print("switch", (sender as! UISwitch).isOn)
        let defaults = UserDefaults.standard
        defaults.set((sender as! UISwitch).isOn, forKey: "isYawOnOff")
        defaults.synchronize()
    }
    
    //MARK: - IBActions
    
    @IBAction func connectClicked(_ sender: Any) {
        viewModel?.connect()
    }
    
    @IBAction func settingsClicked(_ sender: Any) {
        performSegue(withIdentifier: "settings", sender: nil)
    }

    // MARK: - Feature screens

    @objc private func openFlight() {
        guard let session = viewModel?.session else { return }
        let vc = FlightViewController(client: session.crazyFlie.crtp,
                                      telemetry: session.telemetry)
        vc.onSwitchSection = { [weak self] section in self?.openSection(section) }
        let nav = UINavigationController(rootViewController: vc)
        nav.modalPresentationStyle = .fullScreen
        present(nav, animated: true)
    }

    @objc private func openTune() {
        guard let session = viewModel?.session else { return }
        let vc = PidTuneViewController(client: session.crazyFlie.crtp,
                                       telemetry: session.telemetry)
        vc.onSwitchSection = { [weak self] section in self?.openSection(section) }
        let nav = UINavigationController(rootViewController: vc)
        nav.modalPresentationStyle = .fullScreen
        present(nav, animated: true)
    }

    @objc private func openNav() {
        guard let session = viewModel?.session else { return }
        let vc = GpsNavViewController(client: session.crazyFlie.crtp,
                                      telemetry: session.telemetry)
        vc.onSwitchSection = { [weak self] section in self?.openSection(section) }
        let nav = UINavigationController(rootViewController: vc)
        nav.modalPresentationStyle = .fullScreen
        present(nav, animated: true)
    }

    @objc private func openConnection() {
        let vc = ConnectionViewController()
        vc.onApply = { [weak self] in
            self?.viewModel?.applyConnectionSettings()
        }
        let nav = UINavigationController(rootViewController: vc)
        nav.modalPresentationStyle = .formSheet
        present(nav, animated: true)
    }
    
    //MARK: - Private
    
    private func setupUI() {
        guard let viewModel = viewModel else { return }
        connectProgress.progress = 0
        
        connectButton.layer.borderColor = connectButton.tintColor.cgColor
        settingsButton.layer.borderColor = settingsButton.tintColor.cgColor

        //Init joysticks
        let frame = UIScreen.main.bounds
        
        let leftViewModel = viewModel.leftJoystickProvider
        let leftJoystick = BCJoystick(frame: frame, viewModel: leftViewModel)
        leftViewModel.delegate = leftJoystick
        leftViewModel.add(observer: viewModel)
        leftView.addSubview(leftJoystick)
        self.leftJoystick = leftJoystick
        
        let rightViewModel = viewModel.rightJoystickProvider
        let rightJoystick = BCJoystick(frame: frame, viewModel: rightViewModel)
        rightViewModel.delegate = rightJoystick
        rightViewModel.add(observer: viewModel)
        rightView.addSubview(rightJoystick)
        self.rightJoystick = rightJoystick

        setupFlyScreen()
    }
    
    fileprivate func updateUI() {
        guard let viewModel = viewModel else {
            return
        }
        unlockLabel.isHidden = viewModel.bothThumbsOnJoystick
        
        leftJoystick?.hLabel.text = viewModel.leftXTitle
        leftJoystick?.vLabel.text = viewModel.leftYTitle
        rightJoystick?.hLabel.text = viewModel.rightXTitle
        rightJoystick?.vLabel.text = viewModel.rightYTitle
        
        connectProgress.setProgress(viewModel.progress, animated: true)
        connectButton.setTitle(viewModel.topButtonTitle, for: .normal)
        refreshFly()
    }
    
    // MARK: - Fly screen
    //
    // One screen to hold while flying (redesign stage 1):
    //   header  = sections · link · battery · mode · ARMED · CUT MOTORS, always visible
    //   centre  = mode, one main action (Takeoff / Land), four live numbers, More
    //   sides   = the two floating joysticks, unchanged
    // The storyboard's own centre strip (Connect, Settings, progress, Yaw switch) is
    // hidden rather than deleted, so the storyboard itself does not have to change.
    // Một màn hình duy nhất để cầm khi bay (thiết kế lại giai đoạn 1):
    //   header  = các khu · liên kết · pin · chế độ · ARMED · CUT MOTORS, luôn hiện
    //   centre  = chế độ, một hành động chính (Takeoff / Land), bốn số trực tiếp, More
    //   sides   = hai joystick nổi, không đổi
    // Dải giữa của storyboard (Connect, Settings, thanh tiến độ, công tắc Yaw) bị
    // ẩn chứ không xóa, nên bản thân storyboard không phải thay đổi.

    private enum Fly {
        static func rgb(_ hex: UInt32) -> UIColor {
            UIColor(red: CGFloat((hex >> 16) & 0xFF) / 255, green: CGFloat((hex >> 8) & 0xFF) / 255,
                    blue: CGFloat(hex & 0xFF) / 255, alpha: 1)
        }
        static let ground = rgb(0xF3F4F1)
        static let surface = UIColor.white
        static let ink = rgb(0x15181C)
        static let muted = rgb(0x4A525A)
        static let line = rgb(0xD5D9D4)
        static let control = rgb(0xC9CEC8)
        static let track = rgb(0xE7EAE5)
        static let accent = rgb(0x1F5FBF)
        static let danger = rgb(0xB3261E)
        static let armed = rgb(0xF2A900)
        static let warn = rgb(0xB26A00)
        static let ok = rgb(0x1E6B37)
        static let hoverKey = "flyHoverThrust"
    }

    private var isConnected: Bool { (viewModel?.progress ?? 0) >= 1 }
    private var client: CrtpClient? { viewModel?.session?.crazyFlie.crtp }
    private var telemetry: TelemetryStore? { viewModel?.session?.telemetry }

    private var hoverThrust: UInt16 {
        let stored = UserDefaults.standard.integer(forKey: Fly.hoverKey)
        return UInt16(stored == 0 ? 30000 : min(max(stored, 8000), 55000))
    }

    private func setupFlyScreen() {
        // The design is a high-contrast light theme for use outdoors in daylight.
        // Thiết kế là theme sáng độ tương phản cao để dùng ngoài trời ban ngày.
        overrideUserInterfaceStyle = .light
        view.backgroundColor = Fly.ground

        // Retire the storyboard's centre strip; its jobs move into the header,
        // the "not connected" card and the More panel.
        // Khai tử dải giữa của storyboard; việc của nó chuyển vào header,
        // thẻ "not connected" và bảng More.
        connectButton.isHidden = true
        settingsButton.isHidden = true
        connectProgress.isHidden = true
        unlockLabel.alpha = 0
        for sub in view.subviews where sub is UISwitch || (sub as? UILabel)?.text == "Yaw" {
            sub.isHidden = true
        }

        buildHeader()
        buildCentre()
        buildMorePanel()

        let listener: (LogSample) -> Void = { [weak self] _ in self?.refreshFly() }
        telemetryListener = listener
        telemetry?.listeners.append(listener)
        viewModel?.session?.onConsole = { [weak self] text in
            DispatchQueue.main.async { self?.appendLog(text) }
        }
        refreshFly()
    }

    // MARK: Header

    private func buildHeader() {
        header.backgroundColor = Fly.surface
        header.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(header)
        let hairline = UIView()
        hairline.backgroundColor = Fly.line
        hairline.translatesAutoresizingMaskIntoConstraints = false
        header.addSubview(hairline)

        // Sections. Fly is this screen; the others open their screens as before.
        // Các khu. Fly là màn hình này; các khu khác mở màn hình của chúng như trước.
        let tabs = UIStackView()
        tabs.axis = .horizontal
        tabs.spacing = 2
        tabs.isLayoutMarginsRelativeArrangement = true
        tabs.layoutMargins = UIEdgeInsets(top: 2, left: 2, bottom: 2, right: 2)
        tabs.backgroundColor = Fly.track
        tabs.layer.cornerRadius = 10
        for (index, title) in ["Fly", "Mission", "Tune", "Setup"].enumerated() {
            let b = UIButton(type: .system)
            var config = UIButton.Configuration.plain()
            config.contentInsets = NSDirectionalEdgeInsets(top: 0, leading: 14, bottom: 0, trailing: 14)
            b.configuration = config
            let selected = index == 0
            b.applyLabel(title, symbol: nil,
                         font: .systemFont(ofSize: 14, weight: selected ? .semibold : .medium),
                         color: selected ? Fly.ink : Fly.muted)
            b.backgroundColor = selected ? Fly.surface : .clear
            b.layer.cornerRadius = 8
            b.heightAnchor.constraint(equalToConstant: 44).isActive = true
            b.tag = index
            b.accessibilityTraits = selected ? [.button, .selected] : .button
            b.addTarget(self, action: #selector(sectionTapped(_:)), for: .touchUpInside)
            tabs.addArrangedSubview(b)
            sectionButtons.append(b)
        }

        linkIcon.image = UIImage(systemName: "wifi")
        linkIcon.contentMode = .scaleAspectFit
        linkIcon.setContentHuggingPriority(.required, for: .horizontal)
        linkStatus.font = .systemFont(ofSize: 13, weight: .medium)
        let batteryIcon = UIImageView(image: UIImage(systemName: "battery.75"))
        batteryIcon.tintColor = Fly.ink
        batteryIcon.contentMode = .scaleAspectFit
        batteryStatus.font = .monospacedDigitSystemFont(ofSize: 13, weight: .semibold)
        for chip in [modeChip, armedChip] {
            chip.font = .systemFont(ofSize: 12, weight: .bold)
            chip.layer.cornerRadius = 12
            chip.layer.masksToBounds = true
        }

        let status = UIStackView(arrangedSubviews: [linkIcon, linkStatus, spacer(8), batteryIcon, batteryStatus,
                                                    spacer(8), modeChip, armedChip])
        status.axis = .horizontal
        status.alignment = .center
        status.spacing = 6
        let statusHolder = UIView()
        status.translatesAutoresizingMaskIntoConstraints = false
        statusHolder.addSubview(status)
        NSLayoutConstraint.activate([
            status.centerXAnchor.constraint(equalTo: statusHolder.centerXAnchor),
            status.centerYAnchor.constraint(equalTo: statusHolder.centerYAnchor),
            status.leadingAnchor.constraint(greaterThanOrEqualTo: statusHolder.leadingAnchor),
            status.trailingAnchor.constraint(lessThanOrEqualTo: statusHolder.trailingAnchor),
        ])

        var cutConfig = UIButton.Configuration.plain()
        cutConfig.contentInsets = NSDirectionalEdgeInsets(top: 0, leading: 16, bottom: 0, trailing: 16)
        cutMotorsButton.configuration = cutConfig
        cutMotorsButton.layer.cornerRadius = 10
        cutMotorsButton.heightAnchor.constraint(equalToConstant: 44).isActive = true
        cutMotorsButton.setContentCompressionResistancePriority(.required, for: .horizontal)
        cutMotorsButton.addTarget(self, action: #selector(cutMotorsTapped), for: .touchUpInside)
        setCutLabel(primed: false)

        let row = UIStackView(arrangedSubviews: [tabs, statusHolder, cutMotorsButton])
        row.axis = .horizontal
        row.alignment = .center
        row.spacing = 12
        row.translatesAutoresizingMaskIntoConstraints = false
        header.addSubview(row)

        let safe = view.safeAreaLayoutGuide
        NSLayoutConstraint.activate([
            header.topAnchor.constraint(equalTo: view.topAnchor),
            header.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            header.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            header.bottomAnchor.constraint(equalTo: safe.topAnchor, constant: 60),
            hairline.leadingAnchor.constraint(equalTo: header.leadingAnchor),
            hairline.trailingAnchor.constraint(equalTo: header.trailingAnchor),
            hairline.bottomAnchor.constraint(equalTo: header.bottomAnchor),
            hairline.heightAnchor.constraint(equalToConstant: 1),
            row.leadingAnchor.constraint(equalTo: safe.leadingAnchor, constant: 12),
            row.trailingAnchor.constraint(equalTo: safe.trailingAnchor, constant: -12),
            row.bottomAnchor.constraint(equalTo: header.bottomAnchor, constant: -8),
            row.heightAnchor.constraint(equalToConstant: 44),
            statusHolder.heightAnchor.constraint(equalToConstant: 44),
        ])
    }

    private func spacer(_ width: CGFloat) -> UIView {
        let v = UIView()
        v.widthAnchor.constraint(equalToConstant: width).isActive = true
        return v
    }

    private func setCutLabel(primed: Bool) {
        cutMotorsButton.applyLabel(primed ? "TAP AGAIN" : "CUT MOTORS",
                                   symbol: "exclamationmark.triangle.fill",
                                   accessibilityText: primed ? "Tap again to cut motors now" : "Cut motors, tap twice",
                                   font: .systemFont(ofSize: 14, weight: .bold),
                                   color: isConnected ? .white : Fly.muted)
        cutMotorsButton.backgroundColor = !isConnected ? Fly.line : (primed ? .systemOrange : Fly.danger)
        cutMotorsButton.isEnabled = isConnected
    }

    @objc private func cutMotorsTapped() {
        guard isConnected else { return }
        if cutPrimed {
            cutPrimed = false
            setCutLabel(primed: false)
            client?.emergency()
            UINotificationFeedbackGenerator().notificationOccurred(.error)
        } else {
            cutPrimed = true
            setCutLabel(primed: true)
            DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in
                guard let self = self, self.cutPrimed else { return }
                self.cutPrimed = false
                self.setCutLabel(primed: false)
            }
        }
    }

    @objc private func sectionTapped(_ sender: UIButton) {
        guard let section = AppSection(rawValue: sender.tag) else { return }
        openSection(section)
    }

    /// Opens a section from the Fly header, or after another screen's header closed it.
    /// Mở một khu từ header của Fly, hoặc sau khi header màn hình khác đóng nó.
    private func openSection(_ section: AppSection) {
        switch section {
        case .fly: break
        case .mission: openNav()
        case .tune: openTune()
        case .setup:
            let anchor: UIView = sectionButtons.count > 3 ? sectionButtons[3] : view
            openSetup(from: anchor)
        }
    }

    /// Setup is still two existing screens (link, sticks), picked from this short menu.
    /// Setup vẫn là hai màn hình sẵn có (liên kết, cần điều khiển), chọn từ menu ngắn này.
    private func openSetup(from source: UIView) {
        let sheet = UIAlertController(title: "Setup", message: nil, preferredStyle: .actionSheet)
        sheet.addAction(UIAlertAction(title: "Link — Wi-Fi or Bluetooth", style: .default) { [weak self] _ in
            self?.openConnection()
        })
        sheet.addAction(UIAlertAction(title: "Sticks and sensitivity", style: .default) { [weak self] _ in
            self?.performSegue(withIdentifier: "settings", sender: nil)
        })
        sheet.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        sheet.popoverPresentationController?.sourceView = source
        sheet.popoverPresentationController?.sourceRect = source.bounds
        present(sheet, animated: true)
    }

    // MARK: Centre column

    private func buildCentre() {
        // --- connected: fly controls ---
        flyControls.axis = .vertical
        flyControls.spacing = 8

        let modeTitle = smallLabel("Flight mode")
        modeControl.selectedSegmentTintColor = Fly.surface
        modeControl.backgroundColor = Fly.track
        modeControl.setTitleTextAttributes([.font: UIFont.systemFont(ofSize: 14, weight: .medium),
                                            .foregroundColor: Fly.muted], for: .normal)
        modeControl.setTitleTextAttributes([.font: UIFont.systemFont(ofSize: 14, weight: .semibold),
                                            .foregroundColor: Fly.ink], for: .selected)
        modeControl.heightAnchor.constraint(equalToConstant: 44).isActive = true
        modeControl.addTarget(self, action: #selector(flyModeChanged), for: .valueChanged)
        let modeBlock = UIStackView(arrangedSubviews: [modeTitle, modeControl])
        modeBlock.axis = .vertical
        modeBlock.spacing = 4

        primaryButton.layer.cornerRadius = 12
        primaryButton.backgroundColor = Fly.accent
        primaryButton.heightAnchor.constraint(equalToConstant: 52).isActive = true
        primaryButton.addTarget(self, action: #selector(primaryTapped), for: .touchUpInside)
        primaryCaption.font = .systemFont(ofSize: 12)
        primaryCaption.textColor = Fly.muted
        primaryCaption.textAlignment = .center
        primaryCaption.numberOfLines = 2

        let cells = UIStackView()
        cells.axis = .horizontal
        cells.distribution = .fillEqually
        cells.spacing = 8
        telemetryValueLabels = []
        for title in ["ROLL", "PITCH", "SEQUENCE", "FLIGHT"] {
            let name = UILabel()
            name.text = title
            name.font = .systemFont(ofSize: 11, weight: .semibold)
            name.textColor = Fly.muted
            let value = UILabel()
            value.text = "—"
            value.font = .monospacedDigitSystemFont(ofSize: 15, weight: .semibold)
            value.textColor = Fly.ink
            value.adjustsFontSizeToFitWidth = true
            value.minimumScaleFactor = 0.7
            let cell = UIStackView(arrangedSubviews: [name, value])
            cell.axis = .vertical
            cell.isLayoutMarginsRelativeArrangement = true
            cell.layoutMargins = UIEdgeInsets(top: 6, left: 10, bottom: 6, right: 10)
            cell.backgroundColor = Fly.surface
            cell.layer.cornerRadius = 10
            cell.layer.borderWidth = 1
            cell.layer.borderColor = Fly.line.cgColor
            cell.isAccessibilityElement = true
            cells.addArrangedSubview(cell)
            telemetryValueLabels.append(value)
        }

        thumbsHint.font = .systemFont(ofSize: 12, weight: .medium)
        thumbsHint.textColor = Fly.muted
        thumbsHint.textAlignment = .center
        thumbsHint.numberOfLines = 2
        thumbsHint.text = "Fly by hand: both thumbs on the screen sides, then throttle up from zero."

        var moreConfig = UIButton.Configuration.plain()
        moreConfig.contentInsets = NSDirectionalEdgeInsets(top: 0, leading: 12, bottom: 0, trailing: 12)
        moreButton.configuration = moreConfig
        moreButton.applyLabel("More — hover thrust, yaw, all modes", symbol: "slider.horizontal.3",
                              font: .systemFont(ofSize: 14, weight: .medium), color: Fly.ink)
        moreButton.backgroundColor = Fly.surface
        moreButton.layer.cornerRadius = 10
        moreButton.layer.borderWidth = 1.5
        moreButton.layer.borderColor = Fly.control.cgColor
        moreButton.heightAnchor.constraint(equalToConstant: 44).isActive = true
        moreButton.addTarget(self, action: #selector(toggleMore), for: .touchUpInside)

        for v in [modeBlock, primaryButton, primaryCaption, cells, thumbsHint, moreButton] as [UIView] {
            flyControls.addArrangedSubview(v)
        }
        flyControls.setCustomSpacing(4, after: primaryButton)

        // --- not connected: what to do next ---
        disconnectedCard.axis = .vertical
        disconnectedCard.spacing = 12
        disconnectedCard.isLayoutMarginsRelativeArrangement = true
        disconnectedCard.layoutMargins = UIEdgeInsets(top: 18, left: 20, bottom: 18, right: 20)
        disconnectedCard.backgroundColor = Fly.surface
        disconnectedCard.layer.cornerRadius = 16
        disconnectedCard.layer.borderWidth = 1
        disconnectedCard.layer.borderColor = Fly.line.cgColor

        let cardTitle = UILabel()
        cardTitle.text = "Not connected to the drone"
        cardTitle.font = .systemFont(ofSize: 20, weight: .semibold)
        cardTitle.textColor = Fly.ink
        cardTitle.accessibilityTraits = .header
        let steps = UILabel()
        steps.numberOfLines = 0
        steps.font = .systemFont(ofSize: 14)
        steps.textColor = Fly.ink
        switch LinkSettings.linkType {
        case .bleGatt:
            steps.text = "1. Power the drone on.\n2. Keep this phone's Bluetooth on.\n3. Tap Connect."
        case .wifiStation:
            steps.text = "1. Power the drone on; it joins your Wi-Fi.\n2. Keep this phone on the same network.\n3. Tap Connect."
        default:
            steps.text = "1. Power the drone on.\n2. Join the Wi-Fi network “ESP-Drone” in Settings.\n3. Come back here and tap Connect."
        }
        connectNowButton.layer.cornerRadius = 12
        connectNowButton.backgroundColor = Fly.accent
        connectNowButton.heightAnchor.constraint(equalToConstant: 52).isActive = true
        connectNowButton.addTarget(self, action: #selector(connectClicked(_:)), for: .touchUpInside)
        connectNowProgress.progressTintColor = Fly.accent
        linkCaption.font = .systemFont(ofSize: 13)
        linkCaption.textColor = Fly.muted
        linkCaption.textAlignment = .center
        linkCaption.text = "Link: \(LinkSettings.linkType.title) · change it in Setup"
        for v in [cardTitle, steps, connectNowButton, connectNowProgress, linkCaption] as [UIView] {
            disconnectedCard.addArrangedSubview(v)
        }

        for column in [flyControls, disconnectedCard] {
            column.translatesAutoresizingMaskIntoConstraints = false
            view.addSubview(column)
            let width = column.widthAnchor.constraint(equalToConstant: 400)
            width.priority = .defaultHigh
            NSLayoutConstraint.activate([
                column.centerXAnchor.constraint(equalTo: view.safeAreaLayoutGuide.centerXAnchor),
                column.topAnchor.constraint(equalTo: header.bottomAnchor, constant: 14),
                column.bottomAnchor.constraint(lessThanOrEqualTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -8),
                column.widthAnchor.constraint(lessThanOrEqualTo: view.safeAreaLayoutGuide.widthAnchor, multiplier: 0.5),
                width,
            ])
        }
    }

    private func smallLabel(_ text: String) -> UILabel {
        let l = UILabel()
        l.text = text
        l.font = .systemFont(ofSize: 12, weight: .medium)
        l.textColor = Fly.muted
        return l
    }

    @objc private func flyModeChanged() {
        guard let mode = CrtpClient.FlightMode(rawValue: UInt8(modeControl.selectedSegmentIndex)) else { return }
        client?.setFlightMode(mode)
    }

    @objc private func primaryTapped() {
        let armed = (telemetry?.value("flight.armed") ?? 0) > 0.5
        if armed {
            client?.land()
        } else {
            client?.takeoff(hoverThrust: hoverThrust)
        }
    }

    private func updatePrimary(armed: Bool) {
        if armed {
            primaryButton.applyLabel("Land", symbol: "airplane.arrival",
                                     font: .systemFont(ofSize: 17, weight: .semibold), color: .white)
            primaryCaption.text = "Ramps thrust down to the ground, then disarms."
        } else {
            primaryButton.applyLabel("Takeoff", symbol: "airplane.departure",
                                     font: .systemFont(ofSize: 17, weight: .semibold), color: .white)
        }
    }

    // MARK: More panel

    private func buildMorePanel() {
        morePanel.backgroundColor = Fly.surface
        morePanel.layer.shadowColor = UIColor.black.cgColor
        morePanel.layer.shadowOpacity = 0.12
        morePanel.layer.shadowRadius = 12
        morePanel.layer.shadowOffset = CGSize(width: -6, height: 0)
        morePanel.isHidden = true
        morePanel.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(morePanel)

        let title = UILabel()
        title.text = "More"
        title.font = .systemFont(ofSize: 17, weight: .semibold)
        title.accessibilityTraits = .header
        let close = UIButton(type: .system)
        close.applyLabel(nil, symbol: "xmark", accessibilityText: "Close More panel",
                         font: .systemFont(ofSize: 15, weight: .semibold), color: Fly.ink)
        close.backgroundColor = Fly.track
        close.layer.cornerRadius = 10
        close.widthAnchor.constraint(equalToConstant: 44).isActive = true
        close.heightAnchor.constraint(equalToConstant: 44).isActive = true
        close.addTarget(self, action: #selector(toggleMore), for: .touchUpInside)
        let titleRow = UIStackView(arrangedSubviews: [title, UIView(), close])
        titleRow.alignment = .center

        let hoverTitle = smallLabel("Hover thrust — used by Takeoff and Hold")
        hoverValueLabel.font = .monospacedDigitSystemFont(ofSize: 13, weight: .semibold)
        hoverValueLabel.textColor = Fly.ink
        let hoverRow = UIStackView(arrangedSubviews: [hoverTitle, UIView(), hoverValueLabel])
        hoverSlider.minimumValue = 8000
        hoverSlider.maximumValue = 55000
        hoverSlider.value = Float(hoverThrust)
        hoverSlider.minimumTrackTintColor = Fly.accent
        hoverSlider.accessibilityLabel = "Hover thrust"
        hoverSlider.addTarget(self, action: #selector(hoverChanged), for: .valueChanged)
        hoverValueLabel.text = "\(hoverThrust)"

        let yawTitle = UILabel()
        yawTitle.text = "Yaw on the left stick"
        yawTitle.font = .systemFont(ofSize: 14, weight: .semibold)
        let yawCaption = smallLabel("Off: the yaw stick is ignored, so the nose cannot turn by accident.")
        yawCaption.numberOfLines = 2
        let yawText = UIStackView(arrangedSubviews: [yawTitle, yawCaption])
        yawText.axis = .vertical
        yawSwitch.isOn = UserDefaults.standard.bool(forKey: "isYawOnOff")
        yawSwitch.onTintColor = Fly.accent
        yawSwitch.addTarget(self, action: #selector(yawToggled), for: .valueChanged)
        let yawRow = UIStackView(arrangedSubviews: [yawText, yawSwitch])
        yawRow.alignment = .center
        yawRow.spacing = 12

        let allModes = UIButton(type: .system)
        var allConfig = UIButton.Configuration.plain()
        allConfig.contentInsets = NSDirectionalEdgeInsets(top: 0, leading: 12, bottom: 0, trailing: 12)
        allModes.configuration = allConfig
        allModes.applyLabel("All flight modes and sequence…", symbol: "list.bullet",
                            font: .systemFont(ofSize: 14, weight: .medium), color: Fly.ink)
        allModes.layer.cornerRadius = 10
        allModes.layer.borderWidth = 1.5
        allModes.layer.borderColor = Fly.control.cgColor
        allModes.heightAnchor.constraint(equalToConstant: 44).isActive = true
        allModes.addTarget(self, action: #selector(openFlight), for: .touchUpInside)

        let logTitle = smallLabel("DRONE LOG")
        logLabel.numberOfLines = 3
        logLabel.font = .monospacedSystemFont(ofSize: 11, weight: .regular)
        logLabel.textColor = Fly.track
        logLabel.backgroundColor = Fly.ink
        logLabel.layer.cornerRadius = 8
        logLabel.layer.masksToBounds = true
        logLabel.text = " (no messages yet)"

        let limits = smallLabel("This board has only a motion sensor: Hold keeps a thrust level, not a height, and map position is estimated.")
        limits.numberOfLines = 3
        limits.font = .systemFont(ofSize: 12)

        let stack = UIStackView(arrangedSubviews: [titleRow, hoverRow, hoverSlider, yawRow, allModes, logTitle, logLabel, limits])
        stack.axis = .vertical
        stack.spacing = 8
        stack.setCustomSpacing(2, after: hoverRow)
        stack.setCustomSpacing(2, after: logTitle)
        stack.translatesAutoresizingMaskIntoConstraints = false
        morePanel.addSubview(stack)

        let width = morePanel.widthAnchor.constraint(equalToConstant: 440)
        width.priority = .defaultHigh
        NSLayoutConstraint.activate([
            morePanel.topAnchor.constraint(equalTo: header.bottomAnchor),
            morePanel.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            morePanel.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            morePanel.widthAnchor.constraint(lessThanOrEqualTo: view.widthAnchor, multiplier: 0.6),
            width,
            stack.topAnchor.constraint(equalTo: morePanel.topAnchor, constant: 10),
            stack.leadingAnchor.constraint(equalTo: morePanel.leadingAnchor, constant: 16),
            stack.trailingAnchor.constraint(equalTo: morePanel.safeAreaLayoutGuide.trailingAnchor, constant: -16),
            stack.bottomAnchor.constraint(lessThanOrEqualTo: morePanel.safeAreaLayoutGuide.bottomAnchor, constant: -10),
        ])
    }

    @objc private func toggleMore() {
        morePanel.isHidden.toggle()
    }

    @objc private func hoverChanged() {
        let value = UInt16(hoverSlider.value)
        hoverValueLabel.text = "\(value)"
        UserDefaults.standard.set(Int(value), forKey: Fly.hoverKey)
        client?.setHoverThrust(value)
        refreshFly()
    }

    @objc private func yawToggled() {
        UserDefaults.standard.set(yawSwitch.isOn, forKey: "isYawOnOff")
    }

    private func appendLog(_ text: String) {
        let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return }
        logLines.append(trimmed)
        if logLines.count > 3 { logLines.removeFirst(logLines.count - 3) }
        logLabel.text = logLines.map { " " + $0 }.joined(separator: "\n")
    }

    // MARK: Refresh

    /// Pulls every Fly-screen value from the connection state and the status
    /// log block. Called on view-model updates and on every telemetry sample.
    /// Lấy mọi giá trị của màn hình Fly từ trạng thái kết nối và khối log trạng
    /// thái. Được gọi khi view-model cập nhật và trên mỗi mẫu dữ liệu từ xa.
    private func refreshFly() {
        guard viewModel != nil, !sectionButtons.isEmpty else { return }
        let connected = isConnected

        // The status block (battery, mode, armed, sequence, roll, pitch) is what
        // the header and the numbers read. Request it each time a link comes up.
        // Khối trạng thái (pin, chế độ, armed, chuỗi, roll, pitch) là thứ header
        // và các con số đọc. Yêu cầu nó mỗi lần liên kết được thiết lập.
        if connected && !wasConnected {
            client?.createBlock(id: TelemetryStore.BlockId.status.rawValue,
                                variables: TelemetryStore.statusVariables, periodMs: 200)
        }
        wasConnected = connected

        flyControls.isHidden = !connected
        disconnectedCard.isHidden = connected
        if !connected { morePanel.isHidden = true }
        let connectTitle = viewModel?.topButtonTitle ?? "Connect"
        if connectTitle != shownConnectTitle {
            shownConnectTitle = connectTitle
            connectNowButton.applyLabel(connectTitle, symbol: "wifi",
                                        font: .systemFont(ofSize: 17, weight: .semibold), color: .white)
        }
        connectNowProgress.setProgress(viewModel?.progress ?? 0, animated: true)
        connectNowProgress.isHidden = (viewModel?.progress ?? 0) <= 0

        // Link
        linkIcon.image = UIImage(systemName: connected ? "wifi" : "wifi.slash")
        linkIcon.tintColor = connected ? Fly.ok : Fly.muted
        linkStatus.text = connected ? "Connected" : "Not connected"
        linkStatus.textColor = connected ? Fly.ok : Fly.muted

        // Battery
        let vbat = connected ? telemetry?.value("pm.vbat") : nil
        if let v = vbat {
            batteryStatus.text = String(format: "%.2f V", v)
            batteryStatus.textColor = v < 3.3 ? Fly.danger : (v < 3.5 ? Fly.warn : Fly.ink)
        } else {
            batteryStatus.text = "— V"
            batteryStatus.textColor = Fly.muted
        }

        // Mode + armed
        let mode = Int(telemetry?.value("flight.mode") ?? 0)
        let armed = connected && (telemetry?.value("flight.armed") ?? 0) > 0.5
        let autoState = Int(telemetry?.value("flight.auto_state") ?? 0)
        modeChip.isHidden = !connected
        armedChip.isHidden = !connected
        modeChip.text = ["MANUAL", "HOLD", "AUTO", "FOLLOW", "RTH"][min(max(mode, 0), 4)]
        modeChip.backgroundColor = Fly.track
        modeChip.textColor = Fly.ink
        if armed {
            armedChip.text = "● ARMED"
            armedChip.backgroundColor = Fly.armed
            armedChip.textColor = Fly.ink
            armedChip.layer.borderWidth = 0
        } else {
            armedChip.text = "DISARMED"
            armedChip.backgroundColor = .clear
            armedChip.textColor = Fly.muted
            armedChip.layer.borderWidth = 1.5
            armedChip.layer.borderColor = Fly.control.cgColor
        }
        armedChip.accessibilityLabel = armed ? "Armed" : "Disarmed"
        if connected != shownCutConnected {
            shownCutConnected = connected
            if !connected { cutPrimed = false }
            setCutLabel(primed: cutPrimed)
        }

        // Mode control: Follow and RTH live on the Mission screen, so none is selected.
        // Điều khiển chế độ: Follow và RTH nằm trên màn hình Mission, nên không chọn gì.
        modeControl.selectedSegmentIndex = mode <= 2 ? mode : UISegmentedControl.noSegment

        // One main action that follows the situation.
        // Một hành động chính luôn theo hoàn cảnh hiện tại.
        if armed != shownPrimaryArmed {
            shownPrimaryArmed = armed
            updatePrimary(armed: armed)
        }
        if !armed {
            primaryCaption.text = "Arms, climbs to hover thrust \(hoverThrust), then holds level."
        }


        // Sticks only steer in Manual; dim them otherwise so that is visible.
        // Cần chỉ lái ở Manual; các chế độ khác làm chúng mờ đi để thấy rõ điều đó.
        let sticksLive = mode == 0
        leftView.alpha = sticksLive ? 1 : 0.45
        rightView.alpha = sticksLive ? 1 : 0.45
        thumbsHint.isHidden = !(sticksLive && !(viewModel?.bothThumbsOnJoystick ?? false))

        // Numbers
        if armed && armedSince == nil {
            armedSince = Date()
            flightClock = Timer.scheduledTimer(withTimeInterval: 1, repeats: true) { [weak self] _ in
                self?.refreshFly()
            }
        } else if !armed && armedSince != nil {
            armedSince = nil
            flightClock?.invalidate()
            flightClock = nil
        }
        let roll = telemetry?.value("stabilizer.roll")
        let pitch = telemetry?.value("stabilizer.pitch")
        let seconds = Int(armedSince.map { Date().timeIntervalSince($0) } ?? 0)
        let values = [
            roll.map { String(format: "%.1f°", $0) } ?? "—",
            pitch.map { String(format: "%.1f°", $0) } ?? "—",
            ["idle", "takeoff", "hold", "land"][min(max(autoState, 0), 3)],
            String(format: "%02d:%02d", seconds / 60, seconds % 60),
        ]
        for (label, text) in zip(telemetryValueLabels, values) {
            label.text = text
        }
        for (index, cell) in telemetryValueLabels.compactMap({ $0.superview }).enumerated() {
            cell.accessibilityLabel = ["Roll", "Pitch", "Sequence", "Flight time"][index] + " " + values[index]
        }
    }

    // MARK: - Navigation
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "settings" {
            guard let viewController = segue.destination as? SettingsViewController else {
                return
            }
            
            viewController.viewModel = viewModel?.settingsViewModel
        }
    }

}

/// A label with padding, for the status chips in the Fly header.
/// Một nhãn có padding, dùng cho các chip trạng thái trong header của Fly.
final class ChipLabel: UILabel {
    var insets = UIEdgeInsets(top: 4, left: 10, bottom: 4, right: 10)

    override func drawText(in rect: CGRect) {
        super.drawText(in: rect.inset(by: insets))
    }

    override var intrinsicContentSize: CGSize {
        let size = super.intrinsicContentSize
        return CGSize(width: size.width + insets.left + insets.right,
                      height: size.height + insets.top + insets.bottom)
    }
}

extension ViewController: ViewModelDelegate {
    func signalUpdate() {
        updateUI()
    }
    
    func signalFailed(with title: String, message: String?) {
        let alert = UIAlertController(title: title,
                                      message: message,
                                      preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: "Ok",
                                      style: .default,
                                      handler: {[weak alert] (action) in
            alert?.dismiss(animated: true, completion: nil)
        }))
        present(alert, animated: true, completion: nil)
    }
}
/*
                    
                    - (void) updateSettings: (NSUserDefaults*) defaults
{
    controlMode = [defaults doubleForKey:@];
    NSLog(@"controlMode %d", controlMode);
    sensitivities = (NSMutableDictionary*)[defaults dictionaryForKey:@"sensitivities"];
    sensitivitySetting = [defaults stringForKey:@"sensitivitySettings"];
    
    NSDictionary *sensitivity = (NSDictionary*)[sensitivities valueForKey:sensitivitySetting];
    pitchRate = [(NSNumber*)[sensitivity valueForKey:@"pitchRate"] floatValue];
    yawRate = [(NSNumber*)[sensitivity valueForKey:@"yawRate"] floatValue];
    maxThrust = [(NSNumber*)[sensitivity valueForKey:@"maxThrust"] floatValue];
    
    if ([MotionLink new].canAccessMotion) {
        if (controlMode == 5) {
            [self startMotionUpdate];
        }
        else {
            [self stopMotionUpdate];
        }
        
            }
    else {
        
    }
    
    leftJoystick.deadbandX = 0;
    rightJoystick.deadbandX = 0;
    if ([leftJoystick.hLabel.text isEqualToString:@"Yaw"]) {
        leftJoystick.deadbandX = 0.1;
    } else {
        rightJoystick.deadbandX = 0.1;
    }
    
    leftJoystick.thrustControl = NO;
    rightJoystick.thrustControl = NO;
    if ([leftJoystick.vLabel.text isEqualToString:@"Thrust"]) {
        leftJoystick.thrustControl = YES;
    } else {
        rightJoystick.thrustControl = YES;
    }
    }



@end*/
