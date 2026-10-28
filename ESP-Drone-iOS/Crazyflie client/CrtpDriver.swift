//
//  CrtpDriver.swift
//  Crazyflie client
//
//  Created by fanbaoying on 2020/4/29.
//  Copyright © 2020 Bitcraze. All rights reserved.
//

import UIKit

protocol CrtpDriver: class {
    func connect(_ address: String?, callback: @escaping (Bool) -> ())
    func disconnect()
    func sendPacket(_ packet: Data, callback: ((Bool) -> ())?)
    func onStateUpdated(_ callback: @escaping (NSString) -> ())

    /// Inbound path: the driver strips whatever framing its transport adds
    /// (the UDP/SPP checksum, BLE control-byte segments) and hands over one
    /// complete CRTP packet per call — header included, no checksum.
    /// Đường vào: trình điều khiển lược bỏ phần đóng khung mà tầng truyền thêm
    /// vào (byte checksum UDP/SPP, các đoạn byte điều khiển của BLE) và
    /// mỗi lần gọi chuyển giao một gói tin CRTP hoàn chỉnh — có header, không
    /// có checksum.
    func setPacketHandler(_ handler: @escaping (Data) -> ())
}
