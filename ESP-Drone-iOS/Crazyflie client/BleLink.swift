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
    private var preferredId: String?

    /// The names the drone firmware advertises under (BT_DEVICE_NAME and the
    /// legacy ones) — the device list sorts these to the top.
    /// Các tên mà firmware drone dùng để quảng bá (BT_DEVICE_NAME và tên cũ) —
    /// danh sách thiết bị xếp chúng lên đầu.
    static func isDroneName(_ name: String) -> Bool {
        return name == "ESP32_Done" || name == "ESP-Drone" || name == "Crazyflie"
    }

    private var state = "idle" {
        didSet { stateCallback?(state as NSString) }
    }

    /// `preferredName` narrows the scan to one drone (the connection picker's
    /// name field feeds it via LinkSettings). Blank/nil = first match.
    /// `preferredId` is the exact peripheral the pilot tapped in the device
    /// list — it wins over the name and skips the scan entirely.
    /// `preferredName` thu hẹp phép quét về một drone (ô tên của trình chọn
    /// kết nối cấp nó qua LinkSettings). Trống/nil = kết quả khớp đầu tiên.
    /// `preferredId` là đúng thiết bị người lái chạm trong danh sách — nó ưu
    /// tiên hơn tên và bỏ qua luôn bước quét.
    init(preferredName: String? = nil, preferredId: String? = nil) {
        self.preferredName = preferredName
        self.preferredId = preferredId
        super.init()
        central = CBCentralManager(delegate: self, queue: .main)
    }

    // MARK: - CrtpDriver

    func connect(_ address: String?, callback: @escaping (Bool) -> ()) {
        connectCallback = callback
        if let address = address {
            preferredName = address          // keep the init's name when nil
                                             // giữ tên từ init khi nil
        }
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
        resetLink()
        state = "idle"
    }

    /// Everything a live connection owns. Called on disconnect and on
    /// disconnect callbacks so a stale characteristic can never be written to.
    /// Mọi thứ mà một kết nối sống sở hữu. Gọi khi ngắt và trong callback ngắt
    /// để không bao giờ ghi vào một characteristic cũ.
    private func resetLink() {
        peripheral = nil
        crtpChar = nil
        crtpUpChar = nil
        rxBuffer.removeAll()
        rxExpected = 0
        rxPid = 0xFF
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
        // Normalize first: a slice keeps its parent's indices and the offset
        // maths below assumes they start at 0.
        // Chuẩn hóa trước: một lát giữ chỉ số của dữ liệu mẹ, còn phép tính
        // offset bên dưới giả sử chỉ số bắt đầu từ 0.
        let packet = Data(packet)
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
        // A picked identifier connects directly: retrievePeripherals knows the
        // peripheral from before, so no scan round is needed. Fall back to
        // scanning when the identifier is unknown (fresh install, forgotten
        // device).
        // Định danh đã chọn thì kết nối thẳng: retrievePeripherals biết thiết
        // bị từ trước nên không cần quét. Quét lại khi định danh lạ (cài mới,
        // thiết bị đã quên).
        if let idString = preferredId, let uuid = UUID(uuidString: idString) {
            let known = central.retrievePeripherals(withIdentifiers: [uuid])
            if let target = known.first {
                peripheral = target
                target.delegate = self
                central.connect(target, options: nil)
                state = "connecting"
                return
            }
        }
        central.scanForPeripherals(withServices: [BleLink.serviceUUID], options: nil)
        // Connect to the preferred or first match; the picker's name field is
        // what sets the preference (LinkSettings -> init).
        // Kết nối tới thiết bị ưa thích hoặc kết quả khớp đầu tiên; ô tên của
        // trình chọn là thứ đặt lựa chọn đó (LinkSettings -> init).
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
            connectCallback = nil
            return
        }
        peripheral = target
        target.delegate = self
        central.connect(target, options: nil)
        state = "connecting"
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
                // The length field is authoritative: dispatch what was
                // announced, so padding cannot reach the parser (bleRxSeg).
                // Trường độ dài là nguồn quyết định: xử lý đúng số đã báo, để phần
                // đệm không lọt vào bộ phân tích (bleRxSeg).
                packetHandler?(Data(rxBuffer.prefix(total)))
                rxBuffer.removeAll()
            }
        } else {
            guard pid == rxPid, !rxBuffer.isEmpty else {
                rxBuffer.removeAll()
                return
            }
            rxBuffer.append(body)
            if rxBuffer.count >= total {
                packetHandler?(Data(rxBuffer.prefix(total)))
                rxBuffer.removeAll()
            }
        }
    }
}

extension BleLink: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn, connectCallback != nil {
            scanAndConnect()
        } else if central.state != .poweredOn, connectCallback != nil {
            // A pending connect can never succeed with the radio off — fail it
            // once and clear it so the callback cannot fire twice.
            // Kết nối đang chờ không thể thành công khi radio tắt — báo hỏng một
            // lần rồi xóa nó để callback không bắn hai lần.
            connectCallback?(false)
            connectCallback = nil
        }
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        knownNames[peripheral.name ?? peripheral.identifier.uuidString] = peripheral
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
        resetLink()
        state = "idle"
        connectCallback?(false)
        connectCallback = nil
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        // Drop every connection-owned handle (chars, rx state) so nothing can
        // write to a stale characteristic, then tell the caller the link is gone.
        // Gỡ mọi thứ thuộc về kết nối (characteristic, trạng thái nhận) để không
        // ai ghi được vào characteristic cũ, rồi báo cho người gọi là link đã mất.
        resetLink()
        state = "idle"
        connectCallback?(false)
        connectCallback = nil
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
