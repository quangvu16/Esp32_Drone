//
//  CrazyFlie.swift
//  Crazyflie client
//
//  Created by Martin Eberl on 15.07.16.
//  Copyright © 2016 Bitcraze. All rights reserved.
//

import UIKit

protocol CrazyFlieCommander {
    var pitch: Float { get }
    var roll: Float { get }
    var thrust: Float { get }
    var yaw: Float { get }
    
    func prepareData()
}

enum CrazyFlieHeader: UInt8 {
    case commander = 0x30
}

enum CrazyFlieState {
    case idle, connected , scanning, connecting, services, characteristics
}

protocol CrazyFlieDelegate {
    func didSend()
    func didUpdate(state: CrazyFlieState)
    func didFail(with title: String, message: String?)
}

open class CrazyFlie: NSObject {
    
    private(set) var state:CrazyFlieState {
        didSet {
            delegate?.didUpdate(state: state)
        }
    }
    private var timer:Timer?
    private var delegate: CrazyFlieDelegate?
    private(set) var crtpDriver:CrtpDriver!
    /// The receive path + client state machine (TOC/params/logs/flight/nav).
    /// Created alongside the driver so both share the same transport.
    /// Đường nhận + máy trạng thái của client (TOC/tham số/log/điều khiển bay/điều hướng).
    /// Được tạo cùng trình điều khiển để cả hai dùng chung một tầng truyền.
    private(set) var crtp: CrtpClient!

    var commander: CrazyFlieCommander?

    /// No default driver: the caller must name the transport (LinkSettings
    /// builds it). A silent ESPUDPLink default made "just connect" mean UDP
    /// even where the pilot had chosen Bluetooth.
    /// Không có trình điều khiển mặc định: người gọi phải chỉ rõ tầng truyền
    /// (LinkSettings dựng nó). Mặc định ESPUDPLink im lặng làm "chỉ cần kết
    /// nối" nghĩa là UDP ngay cả khi người lái đã chọn Bluetooth.
    init(crtpDriver:CrtpDriver?, delegate: CrazyFlieDelegate?) {

        state = .idle
        self.delegate = delegate

        self.crtpDriver = crtpDriver
        if let driver = crtpDriver {
            self.crtp = CrtpClient(driver: driver)
        }
        super.init()

        crtpDriver?.onStateUpdated{[weak self] (state) in
            if state.isEqual(to: "idle") {
                self?.state = .idle
            } else if state.isEqual(to: "connected") {
                self?.state = .connected
            } else if state.isEqual(to: "scanning")
                        || state.isEqual(to: "connecting")
                        || state.isEqual(to: "services")
                        || state.isEqual(to: "characteristics") {
                self?.state = .connecting
            }
        }
    }
    
    func connect(_ callback:((Bool) -> Void)?) {
        guard state == .idle else {
            self.disconnect()
            return
        }
        print("CrazyFile connect")
        self.crtpDriver.connect(nil, callback: {[weak self] (connected) in
        callback?(connected)
            guard connected else {
                // Transport-accurate text: a BLE failure is not a network
                // failure, and SPP has its own explanation.
                // Chữ đúng theo tầng truyền: lỗi BLE không phải lỗi mạng,
                // và SPP có lời giải thích riêng.
                let title: String
                let body: String
                if self?.crtpDriver is BleLink {
                    title = "Bluetooth Error"
                    body = "Could not connect to the drone over Bluetooth LE. "
                        + "Check it is powered and on a link profile that serves BLE (LINK 3/4/5/6)."
                } else if self?.crtpDriver is SppLink {
                    title = "Not available on iOS"
                    body = SppLink.unavailableReason
                } else {
                    title = "Network Error"
                    body = "The network is not connected. Join the drone's WiFi "
                        + "(ESP-Drone) or check the host address."
                }
                self?.delegate?.didFail(with: title, message: body)
                return
            }
            self?.startTimer()
            self?.crtp.fetchTocs()
        })
    }
    
    func disconnect() {
        self.crtpDriver.disconnect()
        stopTimer()
    }
    
    // MARK: - Private Methods 
    
    private func startTimer() {
        stopTimer()

        self.timer = Timer.scheduledTimer(timeInterval: 0.05, target: self, selector: #selector(self.updateData), userInfo:nil, repeats:true)
    }

    private func stopTimer() {
        if timer != nil {
            timer?.invalidate()
            timer = nil
        }
    }
    @objc
    private func updateData(timer: Timer) {
        guard let commander = commander else {
            return
        }

        commander.prepareData()
        sendFlightData(commander.roll, pitch: commander.pitch, thrust: commander.thrust, yaw: commander.yaw)
    }
    
    private func sendFlightData(_ roll:Float, pitch:Float, thrust:Float, yaw:Float) {
        let defaults = UserDefaults.standard
        let isYawOnOff = defaults.bool(forKey: "isYawOnOff")
        print("isYawOnOff", isYawOnOff as Any)
        var yawdef = yaw
        if !isYawOnOff {
            yawdef = 0.0
        }
        let commandPacket = CommanderPacket(header: CrazyFlieHeader.commander.rawValue, roll: roll, pitch: pitch, yaw: yawdef, thrust: UInt16(thrust))
        let data = CommandPacketCreator.data(from: commandPacket)
        // Raw CRTP — the driver adds the checksum iff its wire needs one (§1 vs
        // §8). The old sendDataCRC() here appended one unconditionally, which
        // BLE must never carry.
        // CRTP thô — trình điều khiển nối thêm checksum nếu đường truyền cần
        // (mục 1 so với mục 8). sendDataCRC() cũ nối thêm vô điều kiện, còn BLE
        // thì không bao giờ được mang checksum.
        self.crtpDriver.sendPacket(data!, callback: nil)
        print("pitch: \(pitch) roll: \(roll) thrust: \(thrust) yaw: \(yawdef)")
    }
}
