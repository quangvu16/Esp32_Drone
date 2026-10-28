//
//  ESPUDPLink.swift
//  Crazyflie client
//
//  Created by fanbaoying on 2020/4/29.
//  Copyright © 2020 Bitcraze. All rights reserved.
//

import UIKit
import CocoaAsyncSocket
import AFNetworking

/// UDP transport: `[CRTP ...][uint8 checksum]` both ways (PROTOCOL.md §1).
/// The endpoints are user-configurable — WiFi SoftAP defaults to the firmware's
/// 192.168.43.42:2390 out / 2399 in, WiFi Station uses a hostname such as
/// esp-drone.local. `LinkSettings` carries the choice.
/// Tầng truyền UDP: `[CRTP ...][uint8 checksum]` theo cả hai chiều
/// (PROTOCOL.md §1). Các điểm cuối do người dùng cấu hình — WiFi SoftAP mặc
/// định theo firmware là 192.168.43.42:2390 ra / 2399 vào, WiFi Station dùng
/// tên máy chủ như esp-drone.local. `LinkSettings` lưu giữ lựa chọn này.
class ESPUDPLink: NSObject, CrtpDriver, GCDAsyncUdpSocketDelegate {

    var clientSocket: GCDAsyncUdpSocket = GCDAsyncUdpSocket()
    var mainQueue = DispatchQueue.main
    var udpError: String?
    let appPort: UInt16
    let devicePort: UInt16
    let udpHost: String
    var stateCallback: ((NSString) -> ())?
    fileprivate var connectCallback: ((Bool) -> ())?
    fileprivate var packetHandler: ((Data) -> ())?
    fileprivate var state = "idle" {
        didSet {
            stateCallback?(state as NSString)
        }
    }

    init(host: String = LinkSettings.defaultHost,
         devicePort: UInt16 = LinkSettings.defaultDevicePort,
         appPort: UInt16 = LinkSettings.defaultAppPort) {
        self.udpHost = host
        self.devicePort = devicePort
        self.appPort = appPort
        super.init()
        clientSocket = GCDAsyncUdpSocket(delegate: self, delegateQueue: mainQueue)
        do {
            try clientSocket.enableBroadcast(true)
            try clientSocket.bind(toPort: appPort, interface: udpError)
            if (udpError != nil) {
                print("error: \(String(describing: udpError))")
            } else {
                try clientSocket.beginReceiving()
            }
        } catch {
            print("catch:\(String(describing: udpError))")
        }
        state = "idle"
    }

    func udpSocket(_ sock: GCDAsyncUdpSocket, didReceive data: Data, fromAddress address: Data, withFilterContext filterContext: Any?) {
        // Datagram framing: [CRTP ...][checksum]. Verify and strip the checksum,
        // then hand the raw CRTP packet to the client state machine. Console
        // text still decodes as UTF-8 further up, so nothing is printed here
        // that would crash on binary.
        // Đóng khung datagram: [CRTP ...][checksum]. Kiểm tra và tách bỏ byte tổng
        // kiểm tra, rồi chuyển gói tin CRTP thô cho máy trạng thái của client. Chữ
        // ở console vẫn được giải mã UTF-8 ở tầng trên, nên không có gì ở đây được
        // in ra mà gây crash với dữ liệu nhị phân.
        guard let crtp = CrtpChecksum.strip(from: data) else {
            print("didReceive: \(data.count) bytes, bad checksum — dropped")
            return
        }
        packetHandler?(crtp)
    }

    func setPacketHandler(_ handler: @escaping (Data) -> ()) {
        packetHandler = handler
    }

    func udpSocket(_ sock: GCDAsyncUdpSocket, didNotConnect error: Error?) {
        print("Connection failed: \(String(describing: error))")
    }
    func udpSocket(_ sock: GCDAsyncUdpSocket, didSendDataWithTag tag: Int) {
        print("Data has been sent: \(tag)")
    }
    func udpSocketDidClose(_ sock: GCDAsyncUdpSocket, withError error: Error?) {
        print("Disconnect error: \(String(describing: error))")
    }
    func udpSocket(_ sock: GCDAsyncUdpSocket, didConnectToAddress address: Data) {
        print("Start connecting")
    }
    func connect(_ address: String?, callback: @escaping (Bool) -> ()) {
        print("ESPUDPLink connect")
        //网络状态消息监听
        // Observe network-reachability change notifications.
        // Lắng nghe thông báo trạng thái mạng
        NotificationCenter.default.addObserver(self, selector: #selector(sendWifiStatus), name: NSNotification.Name.AFNetworkingReachabilityDidChange, object: nil)
        //开启网络状态消息监听
        // Start monitoring network reachability.
        // Bắt đầu lắng nghe trạng thái mạng
        AFNetworkReachabilityManager.shared().startMonitoring()
        connectCallback = callback
    }
    @objc func sendWifiStatus() {
        if AFNetworkReachabilityManager.shared().isReachable {
            if AFNetworkReachabilityManager.shared().isReachableViaWiFi {
                print("Connection Type：WiFi")
                state = "connected"
                connectCallback?(true)
            } else if AFNetworkReachabilityManager.shared().isReachableViaWWAN {
                print("Connection Type：mobile network")
                connectCallback?(false)
            }
        } else {
            print("Connection Type：No internet connection")
            connectCallback?(false)
        }
    }
    func disconnect() {
        print("Remove message notification")
        state = "idle"
        //关闭网络状态消息监听
        // Stop monitoring network reachability.
        // Dừng lắng nghe trạng thái mạng
        AFNetworkReachabilityManager.shared().stopMonitoring()
        //移除网络状态消息通知
        // Unregister the network-reachability observer.
        // Gỡ đăng ký thông báo trạng thái mạng
        NotificationCenter.default.removeObserver(self)
    }

    func sendPacket(_ packet: Data, callback: ((Bool) -> ())?) {
        clientSocket.send(packet, toHost: udpHost, port: devicePort, withTimeout: -1, tag: 0)
    }

    func onStateUpdated(_ callback: @escaping (NSString) -> ()) {
        stateCallback = callback
    }
}
