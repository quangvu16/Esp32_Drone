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
    case wifiApBle = 4         // WiFi SoftAP + BLE coexistence profile
                               // hồ sơ chung sống WiFi SoftAP + BLE
    case wifiStaBle = 5        // WiFi Station + BLE coexistence profile
                               // hồ sơ chung sống WiFi Station + BLE
    case btDual = 6            // Bluetooth Classic SPP + BLE together
                               // Bluetooth Classic SPP + BLE cùng lúc

    var title: String {
        switch self {
        case .wifiSoftAP: return "WiFi SoftAP"
        case .wifiStation: return "WiFi Station"
        case .bluetoothSPP: return "Bluetooth Classic (SPP)"
        case .bleGatt: return "Bluetooth LE (GATT)"
        case .wifiApBle: return "WiFi SoftAP + BLE"
        case .wifiStaBle: return "WiFi Station + BLE"
        case .btDual: return "Bluetooth SPP + BLE"
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
        case .wifiApBle: return 4
        case .wifiStaBle: return 5
        case .btDual: return 6
        }
    }

    /// True when the profile serves the Bitcraze BLE GATT service (§8).
    /// Đúng khi hồ sơ phục vụ service BLE GATT của Bitcraze (mục 8).
    var servesBle: Bool {
        switch self {
        case .bleGatt, .wifiApBle, .wifiStaBle, .btDual: return true
        default: return false
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
        case .wifiApBle:
            return "Drone serves SoftAP UDP and BLE at once (firmware LINK 4). This app uses the WiFi side; BLE stays free for another client."
        case .wifiStaBle:
            return "Drone on your WiFi and BLE at once (firmware LINK 5). This app uses the WiFi side; BLE stays free for another client."
        case .btDual:
            return "Drone serves Bluetooth Classic SPP and BLE together (firmware LINK 6). This app uses the BLE side — SPP needs an Android or desktop client."
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
        static let bleName = "linkBleName"
        static let bleId = "linkBleId"
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
        case .wifiSoftAP, .wifiApBle: return saved ?? defaultHost
        case .wifiStation, .wifiStaBle: return saved ?? defaultStationHost
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

    /// Preferred BLE peripheral name (blank = first advertisement found).
    /// Persisted so the picker's name field survives relaunches.
    /// Tên thiết bị BLE ưa thích (trống = quảng bá đầu tiên tìm thấy). Được lưu
    /// để ô tên của trình chọn còn giữ qua các lần khởi động lại.
    static var bleName: String {
        get { UserDefaults.standard.string(forKey: Keys.bleName) ?? "" }
        set { UserDefaults.standard.set(newValue, forKey: Keys.bleName) }
    }

    /// Identifier (UUID string) of the exact peripheral the pilot tapped in the
    /// device list — connecting is direct, never a name scan, so two drones
    /// advertising the same name stay apart. Blank = match by name.
    /// Định danh (chuỗi UUID) của đúng thiết bị người lái chạm trong danh sách
    /// — kết nối thẳng, không quét tên, nên hai drone trùng tên quảng bá vẫn
    /// phân biệt được. Trống = khớp theo tên.
    static var bleId: String {
        get { UserDefaults.standard.string(forKey: Keys.bleId) ?? "" }
        set { UserDefaults.standard.set(newValue, forKey: Keys.bleId) }
    }

    /// Builds the driver for the current selection. Coexistence profiles pick
    /// the side this app can use: WiFi for 4/5 (BLE left free for another
    /// client), BLE for 6 (SPP is not reachable from iOS).
    /// Dựng trình điều khiển cho lựa chọn hiện tại. Các hồ sơ chung sống chọn
    /// phía mà app này dùng được: WiFi cho 4/5 (BLE nhàn cho client khác),
    /// BLE cho 6 (SPP không với tới được từ iOS).
    static func makeDriver() -> CrtpDriver {
        switch linkType {
        case .wifiSoftAP, .wifiStation, .wifiApBle, .wifiStaBle:
            return ESPUDPLink(host: host(for: linkType),
                              devicePort: devicePort,
                              appPort: appPort)
        case .bleGatt, .btDual:
            return BleLink(preferredName: bleName.isEmpty ? nil : bleName,
                           preferredId: bleId.isEmpty ? nil : bleId)
        case .bluetoothSPP:
            return SppLink()
        }
    }
}
