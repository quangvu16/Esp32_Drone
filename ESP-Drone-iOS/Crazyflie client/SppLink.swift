//
//  SppLink.swift
//  Crazyflie client
//
//  Bluetooth Classic SPP transport — **stub, deliberately not implemented**.
//  Tầng truyền Bluetooth Classic SPP — **stub, cố tình chưa triển khai**.
//
//  iOS does not expose Bluetooth Classic serial ports to third-party apps the
//  way Android does. The only sanctioned path is ExternalAccessory (EAAccessory)
//  with an MFi-certified accessory that declares a matching protocol string in
//  its EAProtocolstrings; a bare ESP32 running BluetoothSerial is not MFi, so
//  there is no legal way to open its SPP channel from this app. CoreBluetooth
//  cannot substitute: it speaks BLE GATT only (see BleLink).
//  iOS không mở cổng serial Bluetooth Classic cho ứng dụng bên thứ ba theo cách
//  Android làm. Con đường được phép duy nhất là ExternalAccessory (EAAccessory)
//  với phụ kiện được chứng nhận MFi khai báo chuỗi protocol khớp trong
//  EAProtocolstrings của nó; một ESP32 trần chạy BluetoothSerial không phải MFi,
//  nên không có cách hợp pháp nào để mở channel SPP của nó từ app này.
//  CoreBluetooth không thể thay thế: nó chỉ nói BLE GATT (xem BleLink).
//
//  Wire format would be `[CRTP ...][uint8 checksum]`, identical to UDP
//  (PROTOCOL.md §1), so if Apple/MFi ever makes this reachable the transport
//  only needs this file filled in — the CRTP stack above does not change.
//  Định dạng trên đường truyền sẽ là `[CRTP ...][uint8 checksum]`, giống hệt
//  UDP (PROTOCOL.md §1), nên nếu Apple/MFi cho phép điều này trong tương lai,
//  tầng truyền chỉ cần điền vào file này — ngăn CRTP phía trên không thay đổi.
//
//  The connection picker shows this option greyed out with this explanation
//  rather than pretending it can connect.
//  Trình chọn kết nối hiển thị tùy chọn này ở trạng thái mờ kèm lời giải thích
//  này, thay vì giả vờ rằng nó có thể kết nối.
//

import Foundation

final class SppLink: NSObject, CrtpDriver {

    /// Set by the connection picker to surface the reason in the UI.
    /// Được trình chọn kết nối đặt để hiển thị lý do trên UI.
    static let unavailableReason =
        "Bluetooth Classic SPP needs an MFi accessory (ExternalAccessory). " +
        "A bare ESP32 SPP link cannot be opened from iOS — use WiFi or BLE instead."

    var stateCallback: ((NSString) -> ())?
    private var packetHandler: ((Data) -> ())?

    func connect(_ address: String?, callback: @escaping (Bool) -> ()) {
        print("SppLink: \(SppLink.unavailableReason)")
        callback(false)
    }

    func disconnect() {}

    func setPacketHandler(_ handler: @escaping (Data) -> ()) {
        packetHandler = handler
    }

    func sendPacket(_ packet: Data, callback: ((Bool) -> ())?) {
        // Would append the checksum and write to the EASession's output stream.
        // Sẽ nối thêm byte checksum và ghi vào output stream của EASession.
        callback?(false)
    }

    func onStateUpdated(_ callback: @escaping (NSString) -> ()) {
        stateCallback = callback
    }
}
