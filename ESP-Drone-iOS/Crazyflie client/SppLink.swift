//
//  SppLink.swift
//  Crazyflie client
//
//  Bluetooth Classic SPP transport — **stub, deliberately not implemented**.
//
//  iOS does not expose Bluetooth Classic serial ports to third-party apps the
//  way Android does. The only sanctioned path is ExternalAccessory (EAAccessory)
//  with an MFi-certified accessory that declares a matching protocol string in
//  its EAProtocolstrings; a bare ESP32 running BluetoothSerial is not MFi, so
//  there is no legal way to open its SPP channel from this app. CoreBluetooth
//  cannot substitute: it speaks BLE GATT only (see BleLink).
//
//  Wire format would be `[CRTP ...][uint8 checksum]`, identical to UDP
//  (PROTOCOL.md §1), so if Apple/MFi ever makes this reachable the transport
//  only needs this file filled in — the CRTP stack above does not change.
//
//  The connection picker shows this option greyed out with this explanation
//  rather than pretending it can connect.
//

import Foundation

final class SppLink: NSObject, CrtpDriver {

    /// Set by the connection picker to surface the reason in the UI.
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
        callback?(false)
    }

    func onStateUpdated(_ callback: @escaping (NSString) -> ()) {
        stateCallback = callback
    }
}
