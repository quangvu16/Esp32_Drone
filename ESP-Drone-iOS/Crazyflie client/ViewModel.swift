//
//  ViewModel.swift
//  Crazyflie client
//
//  Created by Martin Eberl on 23.01.17.
//  Copyright © 2017 Bitcraze. All rights reserved.
//

import Foundation

protocol ViewModelDelegate: class {
    func signalUpdate()
    func signalFailed(with title: String, message: String?)
}

final class ViewModel {
    weak var delegate: ViewModelDelegate?
    let leftJoystickProvider: BCJoystickViewModel
    let rightJoystickProvider: BCJoystickViewModel
    
    private var motionLink: MotionLink?
    private var crazyFlie: CrazyFlie?
    /// Owns the CrazyFlie + CrtpClient pair and fans log samples out to the
    /// screens. A transport change rebuilds it via `reconnect`.
    /// Sở hữu cặp CrazyFlie + CrtpClient và phân phối các mẫu log tới các màn
    /// hình. Đổi tầng truyền sẽ dựng lại nó qua `reconnect`.
    private(set) var session: DroneSession!
    /// Tilt-to-fly, when the user selects that control mode.
    /// Nghiêng để bay, khi người dùng chọn chế độ điều khiển đó.
    let tiltControl = TiltControl()
    private var sensitivity: Sensitivity = .slow
    private var controlMode: ControlMode = ControlMode.current ?? .mode1

    fileprivate(set) var progress: Float = 0
    fileprivate(set) var topButtonTitle: String

    init() {
        topButtonTitle = "Connect"

        leftJoystickProvider = BCJoystickViewModel()
        rightJoystickProvider = BCJoystickViewModel(deadbandX: 0.1, vLabelLeft: true)

        leftJoystickProvider.add(observer: self)
        rightJoystickProvider.add(observer: self)

        let link = LinkSettings.makeDriver()
        crazyFlie = CrazyFlie(crtpDriver: link, delegate: self)
        session = DroneSession(crazyFlie: crazyFlie!)
        loadDefaults()
    }
    
    deinit {
        leftJoystickProvider.remove(observer: self)
        rightJoystickProvider.remove(observer: self)
    }
    
    var leftXTitle: String? {
        return title(at: 0)
    }
    var rightXTitle: String? {
        return title(at: 2)
    }
    var leftYTitle: String? {
        return title(at: 1)
    }
    var rightYTitle: String? {
        return title(at: 3)
    }
    
    var bothThumbsOnJoystick: Bool {
        return leftJoystickProvider.activated && rightJoystickProvider.activated
    }
    
    lazy var settingsViewModel: SettingsViewModel? = {
        guard let crtpDriver = self.crazyFlie?.crtpDriver else {
            return nil
        }
        let settings = SettingsViewModel(sensitivity: self.sensitivity, controlMode: self.controlMode, crtpDriver: crtpDriver)
        settings.add(observer: self)
        return settings
    }()
    
    // MARK: - Public Methods
    
    func loadSettings() {
        
    }
    
    func connect() {
        crazyFlie?.connect(nil)
    }

    /// Tears down and rebuilds the link from the current LinkSettings. Called
    /// when the connection picker saves.
    /// Gỡ và dựng lại liên kết từ LinkSettings hiện tại. Được gọi khi trình
    /// chọn kết nối lưu thiết lập.
    func applyConnectionSettings() {
        session.reconnect(delegate: self)
        crazyFlie = session.crazyFlie
        applyCommander()
    }
    
    // MARK: - Private Methods
    
    private func title(at index: Int) -> String? {
        guard controlMode.titles.indices.contains(index) else { return nil }
        
        return controlMode.titles[index]
    }
    
    private func startMotionUpdate() {
        // CMDeviceMotion attitude (TiltControl) is the tilt-to-fly source
        // (PROTOCOL.md §9). The legacy MotionLink accelerometer path stays
        // wired for the other control modes that reference it.
        // Tư thế CMDeviceMotion (TiltControl) là nguồn cho nghiêng-để-bay
        // (PROTOCOL.md §9). Đường dẫn gia tốc kế MotionLink cũ vẫn được nối
        // cho các chế độ điều khiển khác có tham chiếu đến nó.
        if motionLink == nil {
            motionLink = MotionLink()
        }
        tiltControl.start()
        motionLink?.startDeviceMotionUpdates()
        motionLink?.startAccelerometerUpdates()
    }

    private func stopMotionUpdate() {
        tiltControl.stop()
        motionLink?.stopDeviceMotionUpdates()
        motionLink?.stopAccelerometerUpdates()
    }
    
    private func loadDefaults() {
        guard let url = Bundle.main.url(forResource: "DefaultPreferences", withExtension: "plist"),
            let defaultPrefs = NSDictionary(contentsOf: url) else {
                return
        }
        let defaults = UserDefaults.standard
        defaults.register(defaults: defaultPrefs as! [String : Any])
        defaults.synchronize()
        
        updateSettings()
    }
    
    func updateSettings() {
        if controlMode == .tilt,
            MotionLink().canAccessMotion {
            startMotionUpdate()
        }
        else {
            stopMotionUpdate()
        }
        
        applyCommander()
    }
    
    fileprivate func calibrateMotionIfNeeded() {
        if leftJoystickProvider.touchesChanged || rightJoystickProvider.touchesChanged, controlMode == .tilt {
            motionLink?.calibrate()
        }
    }
    
    fileprivate func changed(controlMode: ControlMode) {
        self.controlMode = controlMode
        updateSettings()
    }
    
    private func applyCommander() {
        if controlMode == .tilt {
            // TiltControl is itself a CrazyFlieCommander: roll/pitch/yaw come
            // from the phone's CMDeviceMotion attitude, thrust stays on the
            // existing stick (PROTOCOL.md §9).
            // TiltControl vốn là một CrazyFlieCommander: roll/pitch/yaw lấy từ
            // tư thế CMDeviceMotion của điện thoại, lực đẩy vẫn ở trên cần điều
            // khiển hiện có (PROTOCOL.md §9).
            rightJoystickProvider.thrustControl = .y
            leftJoystickProvider.thrustControl = .none
            tiltControl.thrustProvider = { [weak self] in
                guard let self = self else { return 0 }
                // Reuse the stick's thrust scaling path.
                // Dùng lại đường tỷ lệ lực đẩy của cần điều khiển.
                return self.tiltThrust()
            }
            crazyFlie?.commander = tiltControl
            return
        }
        crazyFlie?.commander = controlMode.commander(
            leftJoystick: leftJoystickProvider,
            rightJoystick: rightJoystickProvider,
            motionLink: motionLink,
            settings: sensitivity.settings)
    }

    /// Thrust from the right stick, scaled exactly as SimpleCrazyFlieCommander
    /// does, so tilt mode and stick modes command the same wire values.
    /// Lực đẩy lấy từ cần bên phải, tỷ lệ đúng như SimpleCrazyFlieCommander làm,
    /// để chế độ nghiêng và chế độ cần điều khiển ra cùng giá trị trên đường truyền.
    private func tiltThrust() -> Float {
        guard let settings = sensitivity.settings else { return 0 }
        // With thrustControl == .y the view model already maps the stick into
        // 0…1; that is the same input SimpleCrazyFlieCommander.thrust reads.
        // Với thrustControl == .y, view model đã ánh xạ cần vào 0…1; đó là cùng
        // đầu vào mà SimpleCrazyFlieCommander.thrust đọc.
        let control = rightJoystickProvider.y
        var thrust = control * 65535 * (settings.maxThrust / 100)
        if thrust > 65535 { thrust = 65535 }
        if thrust < 0 { thrust = 0 }
        return thrust
    }
    
    fileprivate func updateWith(state: CrazyFlieState) {
        topButtonTitle = "Cancel"
        switch state {
        case .idle:
            progress = 0
            topButtonTitle = "Connect"
            break
        case .scanning:
            progress = 0
            break
        case .connecting:
            progress = 0.25
            break
        case .services:
            progress = 0.5
            break
        case .characteristics:
            progress = 0.75
            break
        case .connected:
            progress = 1
            break
        }
    }
}

extension ViewModel: BCJoystickViewModelObserver {
    func didUpdateState() {
        calibrateMotionIfNeeded()
        
        delegate?.signalUpdate()
    }
}

extension ViewModel: SettingsViewModelObserver {
    func didUpdate(controlMode: ControlMode) {
        changed(controlMode: controlMode)
    }
}

//MARK: - Crazyflie
extension ViewModel: CrazyFlieDelegate {
    func didSend() {
        
    }
    
    func didUpdate(state: CrazyFlieState) {
        updateWith(state: state)
        delegate?.signalUpdate()
    }
    
    func didFail(with title: String, message: String?) {
        delegate?.signalFailed(with: title, message: message)
    }
}
