//
//  LinkSettings.swift
//  Crazyflie client
//
//  Which transport the app talks to, and where. Persisted in UserDefaults so
//  the picker survives relaunches. The firmware selects its side of the link
//  from NVS (PROTOCOL.md §8) — the picker's profiles mirror those names so the
//  pilot can match the board's `link` setting.
//  App giao tiếp với tầng truyền nào, và ở đâu. Được lưu trong UserDefaults
//  để trình chọn còn giữ được lựa chọn qua các lần khởi động lại. Firmware
//  chọn phía liên kết của nó từ NVS (PROTOCOL.md §8) — các hồ sơ của trình
//  chọn phản ánh đúng các tên đó để người lái khớp với thiết lập `link` của
//  bo mạch.
//

import Foundation

enum LinkType: Int, CaseIterable {
    case wifiSoftAP = 0        // drone hosts ESP-Drone at 192.168.43.42 (default)
                               // drone phát WiFi ESP-Drone tại 192.168.43.42 (mặc định)
    case wifiStation = 1       // drone joins a network, esp-drone.local
                               // drone tham gia một mạng WiFi, esp-drone.local
    case bluetoothSPP = 2      // Bluetooth Classic serial — not usable on iOS (MFi)
                               // serial Bluetooth Classic — không dùng được trên iOS (MFi)
    case bleGatt = 3           // Bitcraze GATT service
                               // service GATT của Bitcraze

    var title: String {
        switch self {
        case .wifiSoftAP: return "WiFi SoftAP"
        case .wifiStation: return "WiFi Station"
        case .bluetoothSPP: return "Bluetooth Classic (SPP)"
        case .bleGatt: return "Bluetooth LE (GATT)"
        }
    }

    /// Matches PROTOCOL.md §8's transport profile numbering where one exists.
    /// Khớp với cách đánh số hồ sơ tầng truyền trong PROTOCOL.md §8 ở những nơi có đánh số.
    var firmwareLinkMode: UInt8 {
        switch self {
        case .wifiSoftAP: return 0
        case .wifiStation: return 1
        case .bluetoothSPP: return 2
        case .bleGatt: return 3
        }
    }

    var detail: String {
        switch self {
        case .wifiSoftAP:
            return "Join the drone's ESP-Drone network. Drone at 192.168.43.42, UDP 2390 out / 2399 in."
        case .wifiStation:
            return "Drone on your WiFi. Host is user-editable (default esp-drone.local)."
        case .bluetoothSPP:
            return "iOS only exposes Bluetooth Classic serial to MFi accessories (ExternalAccessory). A bare ESP32 SPP link cannot work from this app."
        case .bleGatt:
            return "Bitcraze GATT service 00000201…: short writes raw on 00000202, long writes segmented on 00000203, all notifications framed on 00000204."
        }
    }
}

struct LinkSettings {
    static let defaultHost = "192.168.43.42"
    static let defaultStationHost = "esp-drone.local"
    static let defaultDevicePort: UInt16 = 2390
    static let defaultAppPort: UInt16 = 2399

    private enum Keys {
        static let type = "linkType"
        static let host = "linkHost"
        static let devicePort = "linkDevicePort"
        static let appPort = "linkAppPort"
    }

    static var linkType: LinkType {
        get { LinkType(rawValue: UserDefaults.standard.integer(forKey: Keys.type)) ?? .wifiSoftAP }
        set { UserDefaults.standard.set(newValue.rawValue, forKey: Keys.type) }
    }

    /// Host for the UDP transports. Kept per type so switching back and forth
    /// does not lose the station hostname.
    /// Host cho các tầng truyền UDP. Được lưu theo từng loại để việc chuyển qua
    /// lại không làm mất tên máy chủ của station.
    static func host(for type: LinkType) -> String {
        let saved = UserDefaults.standard.string(forKey: Keys.host)
        switch type {
        case .wifiSoftAP: return saved ?? defaultHost
        case .wifiStation: return saved ?? defaultStationHost
        default: return saved ?? defaultHost
        }
    }

    static func setHost(_ host: String, for type: LinkType) {
        UserDefaults.standard.set(host, forKey: Keys.host)
    }

    static var devicePort: UInt16 {
        get {
            let stored = UserDefaults.standard.integer(forKey: Keys.devicePort)
            return stored == 0 ? defaultDevicePort : UInt16(stored)
        }
        set { UserDefaults.standard.set(Int(newValue), forKey: Keys.devicePort) }
    }

    static var appPort: UInt16 {
        get {
            let stored = UserDefaults.standard.integer(forKey: Keys.appPort)
            return stored == 0 ? defaultAppPort : UInt16(stored)
        }
        set { UserDefaults.standard.set(Int(newValue), forKey: Keys.appPort) }
    }

    /// Builds the driver for the current selection.
    /// Dựng trình điều khiển cho lựa chọn hiện tại.
    static func makeDriver() -> CrtpDriver {
        switch linkType {
        case .wifiSoftAP, .wifiStation:
            return ESPUDPLink(host: host(for: linkType),
                              devicePort: devicePort,
                              appPort: appPort)
        case .bleGatt:
            return BleLink()
        case .bluetoothSPP:
            return SppLink()
        }
    }
}
