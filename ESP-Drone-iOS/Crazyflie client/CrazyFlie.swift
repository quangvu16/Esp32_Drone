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
    private(set) var crtp: CrtpClient!

    var commander: CrazyFlieCommander?

    init(crtpDriver:CrtpDriver? = ESPUDPLink(), delegate: CrazyFlieDelegate?) {

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
                var title:String
                var body:String?
                
                // Find the reason and prepare a message
                title = "Network Error"
                body = "The network is not connected"
                
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
        let sendData = self.sendDataCRC(data!)
//        print("发送数据: \(sendData)")
        self.crtpDriver.sendPacket(sendData, callback: nil)
        print("pitch: \(pitch) roll: \(roll) thrust: \(thrust) yaw: \(yawdef)")
    }
    func sendDataCRC(_ data:Data) -> Data {
        var sendData = data
        let sendByte = [UInt8](data)
        var count = 0
        for item in sendByte {
            count = count + Int(item)
//            print("字节：\(item) === 总和：\(count)")
        }
        let countByte = UInt8(count & 0xff)
        sendData.append(countByte)
        return sendData
    }
}
