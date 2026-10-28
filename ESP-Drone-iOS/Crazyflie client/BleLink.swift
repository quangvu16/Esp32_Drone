//
//  BleLink.swift
//  Crazyflie client
//
//  BLE GATT transport (PROTOCOL.md §8). Note this project previously only
//  linked CoreBluetooth without using it — the GATT client is implemented
//  here against the Bitcraze service the firmware advertises:
//  Tầng truyền BLE GATT (PROTOCOL.md §8). Lưu ý trước đây project này chỉ
//  liên kết CoreBluetooth mà không dùng nó — GATT client được triển khai ở
//  đây với service Bitcraze mà firmware quảng bá:
//
//    00000201-1C7F-4F9E-947B-43B7C00A9A08  service
//      00000202  CRTP      write   raw CRTP when <= 20 bytes
//      00000203  CRTPUP    write   control-byte segmented
//      00000204  CRTPDOWN  notify  ALWAYS control-byte framed
//
//  The asymmetry is the wire's, not ours: short phone->drone writes go raw on
//  00000202, but every drone->phone notification carries a control byte
//  (bit7 start | bits6..5 pid | bits4..0 totalLen-1, 19 CRTP bytes per
//  segment). A datagram is single-segment when its announced length fits in
//  that segment — not when the length field alone looks small.
//  Sự bất đối xứng là của đường truyền, không phải của ta: các lần ghi ngắn từ
//  phone sang drone được gửi thô trên 00000202, nhưng mọi notification từ drone
//  về phone đều mang một byte điều khiển (bit7 start | bits6..5 pid |
//  bits4..0 totalLen-1, 19 byte CRTP mỗi đoạn). Một datagram là đơn đoạn khi
//  chiều dài được báo trước nằm gọn trong đoạn đó — không phải khi riêng trường
//  chiều dài trông có vẻ nhỏ.
//

import Foundation
import CoreBluetooth

final class BleLink: NSObject, CrtpDriver {

    private static let serviceUUID = CBUUID(string: "00000201-1C7F-4F9E-947B-43B7C00A9A08")
    private static let crtpUUID = CBUUID(string: "00000202-1C7F-4F9E-947B-43B7C00A9A08")
    private static let crtpUpUUID = CBUUID(string: "00000203-1C7F-4F9E-947B-43B7C00A9A08")
    private static let crtpDownUUID = CBUUID(string: "00000204-1C7F-4F9E-947B-43B7C00A9A08")

    /// CRTP bytes per 20-byte segment: 1 control byte + 19 payload.
    /// Số byte CRTP mỗi đoạn 20 byte: 1 byte điều khiển + 19 byte phần dữ liệu.
    private static let segmentBytes = 19

    var stateCallback: ((NSString) -> ())?
    /// Discovered drone names, for the connection picker.
    /// Tên các drone đã phát hiện, dùng cho trình chọn kết nối.
    var onPeripheralsFound: (([String]) -> Void)?

    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var crtpChar: CBCharacteristic?
    private var crtpUpChar: CBCharacteristic?
    private var connectCallback: ((Bool) -> ())?
    private var packetHandler: ((Data) -> ())?

    // Control-byte reassembly state (§8).
    // Trạng thái lắp ráp theo byte điều khiển (§8).
    private var rxBuffer = Data()
    private var rxExpected = 0
    private var rxPid: UInt8 = 0xFF

    private var sendPid: UInt8 = 0
    private var knownNames: [String: CBPeripheral] = [:]
    private var preferredName: String?

    private var state = "idle" {
        didSet { stateCallback?(state as NSString) }
    }

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: .main)
    }

    // MARK: - CrtpDriver

    func connect(_ address: String?, callback: @escaping (Bool) -> ()) {
        connectCallback = callback
        preferredName = address
        guard central.state == .poweredOn else {
            // CBCentralManagerDelegate will retry once powered on.
            // CBCentralManagerDelegate sẽ thử lại khi Bluetooth đã được bật.
            return
        }
        scanAndConnect()
    }

    func disconnect() {
        if let peripheral = peripheral {
            central.cancelPeripheralConnection(peripheral)
        }
        peripheral = nil
        crtpChar = nil
        crtpUpChar = nil
        rxBuffer.removeAll()
        state = "idle"
    }

    func setPacketHandler(_ handler: @escaping (Data) -> ()) {
        packetHandler = handler
    }

    /// Sends one CRTP packet. Short packets go raw on 0x202; longer ones are
    /// segmented on 0x203 with control bytes.
    /// Gửi một gói tin CRTP. Gói tin ngắn được gửi thô trên 0x202; gói tin dài
    /// hơn được chia đoạn trên 0x203 kèm byte điều khiển.
    func sendPacket(_ packet: Data, callback: ((Bool) -> ())?) {
        guard let peripheral = peripheral else {
            callback?(false)
            return
        }
        if packet.count <= 20, let crtpChar = crtpChar {
            peripheral.writeValue(packet, for: crtpChar, type: .withResponse)
            callback?(true)
            return
        }
        guard let crtpUpChar = crtpUpChar else {
            callback?(false)
            return
        }
        let total = packet.count
        var offset = 0
        while offset < total {
            let n = min(BleLink.segmentBytes, total - offset)
            // Control byte: start bit, wrapping pid, announced length - 1.
            // Byte điều khiển: bit start, pid quay vòng, chiều dài báo trước - 1.
            var ctrl = (sendPid & 0x03) << 5 | UInt8((total - 1) & 0x1F)
            if offset == 0 { ctrl |= 0x80 }
            var segment = Data([ctrl])
            segment.append(packet[offset..<offset + n])
            peripheral.writeValue(segment, for: crtpUpChar, type: .withResponse)
            offset += n
        }
        sendPid = (sendPid + 1) & 0x03
        callback?(true)
    }

    func onStateUpdated(_ callback: @escaping (NSString) -> ()) {
        stateCallback = callback
    }

    // MARK: - Scanning

    private func scanAndConnect() {
        knownNames.removeAll()
        central.scanForPeripherals(withServices: [BleLink.serviceUUID], options: nil)
        // Connect to the preferred or first match; the picker can name one.
        // Kết nối tới thiết bị ưa thích hoặc kết quả khớp đầu tiên; trình chọn
        // có thể chỉ định một thiết bị.
        DispatchQueue.main.asyncAfter(deadline: .now() + 4) { [weak self] in
            guard let self = self, self.peripheral == nil else { return }
            self.connectToPreferred()
        }
    }

    private func connectToPreferred() {
        central.stopScan()
        let target: CBPeripheral?
        if let name = preferredName, let match = knownNames[name] {
            target = match
        } else {
            target = knownNames.values.first
        }
        guard let target = target else {
            connectCallback?(false)
            return
        }
        peripheral = target
        target.delegate = self
        central.connect(target, options: nil)
        state = "connecting"
    }

    /// Lets the connection picker choose a specific drone by name.
    /// Cho phép trình chọn kết nối chọn một drone cụ thể theo tên.
    func connect(toName name: String) {
        preferredName = name
        if let target = knownNames[name] {
            central.stopScan()
            peripheral = target
            target.delegate = self
            central.connect(target, options: nil)
            state = "connecting"
        } else {
            scanAndConnect()
        }
    }

    // MARK: - Control-byte framing (§8)

    private func handleNotification(_ data: Data) {
        guard data.count >= 2 else { return }
        let ctrl = data[data.startIndex]
        let body = data.dropFirst()
        let start = (ctrl & 0x80) != 0
        let pid = (ctrl >> 5) & 0x03
        let total = Int(ctrl & 0x1F) + 1

        if start {
            rxBuffer = Data(body)
            rxExpected = total
            rxPid = pid
            // Single-segment when the announced length fits in this segment.
            // Là đơn đoạn khi chiều dài báo trước nằm gọn trong đoạn này.
            if total <= body.count {
                packetHandler?(Data(rxBuffer))
                rxBuffer.removeAll()
            }
        } else {
            guard pid == rxPid, !rxBuffer.isEmpty else {
                rxBuffer.removeAll()
                return
            }
            rxBuffer.append(body)
            if rxBuffer.count >= total {
                packetHandler?(Data(rxBuffer))
                rxBuffer.removeAll()
            }
        }
    }
}

extension BleLink: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn, connectCallback != nil {
            scanAndConnect()
        } else if central.state != .poweredOn {
            connectCallback?(false)
        }
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        knownNames[peripheral.name ?? peripheral.identifier.uuidString] = peripheral
        onPeripheralsFound?(knownNames.keys.sorted())
        // Auto-connect to the first advertisement unless the picker is driving.
        // Tự động kết nối với quảng bá đầu tiên, trừ khi trình chọn đang điều khiển.
        if self.peripheral == nil, preferredName == nil {
            connectToPreferred()
        }
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.discoverServices([BleLink.serviceUUID])
    }

    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        state = "idle"
        connectCallback?(false)
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        state = "idle"
        self.peripheral = nil
    }
}

extension BleLink: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        for service in services where service.uuid == BleLink.serviceUUID {
            peripheral.discoverCharacteristics(
                [BleLink.crtpUUID, BleLink.crtpUpUUID, BleLink.crtpDownUUID], for: service)
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let characteristics = service.characteristics else { return }
        for characteristic in characteristics {
            switch characteristic.uuid {
            case BleLink.crtpUUID:
                crtpChar = characteristic
            case BleLink.crtpUpUUID:
                crtpUpChar = characteristic
            case BleLink.crtpDownUUID:
                peripheral.setNotifyValue(true, for: characteristic)
            default:
                break
            }
        }
        if crtpChar != nil {
            state = "connected"
            connectCallback?(true)
            connectCallback = nil
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard characteristic.uuid == BleLink.crtpDownUUID, let data = characteristic.value else { return }
        handleNotification(data)
    }
}
