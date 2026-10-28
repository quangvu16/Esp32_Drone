//
//  LogClient.swift
//  Crazyflie client
//
//  Log block model and telemetry decode for CRTP port 5 (PROTOCOL.md §4).
//  Blocks are created from TOC variables, started at a period in 10 ms units,
//  and data packets `<id><ts24><values...>` are decoded against the block's
//  variable list.
//  Mô hình khối log và giải mã dữ liệu từ xa cho CRTP port 5 (PROTOCOL.md §4).
//  Các khối được tạo từ biến TOC, được khởi động với chu kỳ theo đơn vị 10 ms,
//  và các gói tin dữ liệu `<id><ts24><values...>` được giải mã theo danh sách
//  biến của khối.
//
//  Wire-format trap: the Android `Logg.create()` sends a fixed 31-byte buffer
//  with zero padding after the real `<type><tocId>` pairs, and log type codes
//  start at 1 — so a zero type byte terminates the list on the drone. We build
//  trimmed requests instead (the firmware accepts both), but the decode path
//  here follows the same rule so a padded request would parse identically.
//  Cạm bẫy của định dạng trên đường truyền: `Logg.create()` của Android gửi một
//  bộ đệm cố định 31 byte với phần đệm bằng 0 nằm sau các cặp `<type><tocId>`
//  thật, và mã kiểu của log bắt đầu từ 1 — nên một byte kiểu bằng 0 sẽ kết thúc
//  danh sách trên drone. Thay vào đó ta dựng các yêu cầu đã cắt gọn (firmware
//  chấp nhận cả hai), nhưng đường giải mã ở đây vẫn theo cùng quy tắc nên một yêu
//  cầu có đệm cũng được phân tích giống hệt.
//

import Foundation

/// One `<type><tocId>` entry in a log block.
/// Một mục `<type><tocId>` trong một khối log.
struct LogVariable {
    let tocId: UInt8
    let logType: LogType
    let name: String

    var size: Int { logType.size }
}

/// A decoded log data packet.
/// Một gói tin dữ liệu log đã được giải mã.
struct LogSample {
    let blockId: UInt8
    /// Milliseconds since the block started (24-bit, wraps).
    /// Số mili giây kể từ khi khối bắt đầu (24 bit, có tràn vòng).
    let timestampMs: UInt32
    /// One value per variable, in block order.
    /// Mỗi biến một giá trị, theo thứ tự trong khối.
    let values: [CrtpValue]
    /// Variable names in block order, so consumers can key by name without
    /// keeping the block definition around.
    /// Tên biến theo thứ tự trong khối, để nơi sử dụng tra cứu theo tên mà
    /// không cần giữ định nghĩa khối bên mình.
    let names: [String]

    /// `nil` when the block has no such variable (or an undecodable value).
    /// Trả về `nil` khi khối không có biến đó (hoặc giá trị không giải mã được).
    subscript(name: String) -> Double? {
        guard let index = names.firstIndex(of: name),
              values.indices.contains(index) else { return nil }
        return values[index].asDouble
    }

}

/// Client-side mirror of a drone-side log block.
/// Bản sao phía client của một khối log phía drone.
final class LogBlock {
    let id: UInt8
    private(set) var variables: [LogVariable]

    /// Total value bytes per data packet. PROTOCOL.md §4 caps this at 25:
    /// `<header><id><ts24>` already take 5 of the 30 CRTP bytes.
    /// Tổng byte giá trị mỗi gói tin dữ liệu. PROTOCOL.md §4 giới hạn ở 25:
    /// `<header><id><ts24>` đã chiếm 5 trong số 30 byte CRTP.
    static let maxValueBytes = 25

    var valueBytes: Int { variables.reduce(0) { $0 + $1.size } }
    var fitsBudget: Bool { valueBytes <= LogBlock.maxValueBytes }

    init(id: UInt8, variables: [LogVariable]) {
        self.id = id
        self.variables = variables
    }

    /// CREATE payload after `<cmd><id>`: flat `<type><tocId>` pairs, trimmed.
    /// A zero type byte would terminate the list on the drone, which is why
    /// padding is safe — but untrimmed padding wastes nothing either way, so
    /// we send exactly the real pairs.
    /// Phần dữ liệu của CREATE sau `<cmd><id>`: các cặp `<type><tocId>` phẳng,
    /// đã cắt gọn. Một byte kiểu bằng 0 sẽ kết thúc danh sách trên drone, đó là
    /// lý do đệm là an toàn — nhưng đệm không cắt cũng không phí gì, nên ta gửi
    /// đúng các cặp thật.
    var createPayload: Data {
        var out = Data(capacity: variables.count * 2)
        for v in variables {
            out.append(v.logType.rawValue)
            out.append(v.tocId)
        }
        return out
    }

    /// Decodes the value section of a data packet into typed values.
    /// Giải mã phần giá trị của gói tin dữ liệu thành các giá trị có kiểu.
    func decodeValues(_ data: Data) -> [CrtpValue]? {
        var out: [CrtpValue] = []
        out.reserveCapacity(variables.count)
        var offset = data.startIndex
        for v in variables {
            let end = data.index(offset, offsetBy: v.size)
            guard end <= data.endIndex,
                  let value = CrtpValue.decode(logType: v.logType, from: data[offset..<end]) else {
                return nil
            }
            out.append(value)
            offset = end
        }
        return out
    }
}

/// Parses log-channel traffic: settings replies `<cmd><id><err>` and data
/// packets `<id><ts24><values...>`. Block bookkeeping lives here so a repeated
/// control reply cannot corrupt the table (every mutation is keyed and
/// idempotent — CREATE on a known identical block just re-acks).
/// Phân tích lưu lượng trên channel log: câu trả lời của settings `<cmd><id><err>`
/// và gói tin dữ liệu `<id><ts24><values...>`. Việc ghi sổ khối nằm ở đây để một
/// câu trả lời điều khiển lặp lại không thể làm hỏng bảng (mọi thay đổi đều theo
/// khóa và idempotent — CREATE cho một khối đã biết giống hệt chỉ re-ack).
final class LogClient {
    /// Control commands (§4).
    /// Các lệnh điều khiển (§4).
    enum Command: UInt8 {
        case create = 0
        case append = 1
        case delete = 2
        case start = 3
        case stop = 4
        case reset = 5
    }

    /// Error codes the drone returns (§4).
    /// Các mã lỗi mà drone trả về (§4).
    enum ReplyError: UInt8 {
        case ok = 0
        case noId = 2
        case tooLarge = 7
        case noCommand = 8
        case noMemory = 12
        case alreadyExists = 17

        var message: String {
            switch self {
            case .ok: return "OK"
            case .noId: return "no such id"
            case .tooLarge: return "too large"
            case .noCommand: return "command not found"
            case .noMemory: return "no memory"
            case .alreadyExists: return "already exists"
            }
        }
    }

    private(set) var blocks: [UInt8: LogBlock] = [:]

    /// Decoded samples, delivered on the client's queue.
    /// Các mẫu đã giải mã, được giao trên hàng đợi của client.
    var onSample: ((LogSample) -> Void)?
    /// Control replies, for UI feedback. Repeats of the same reply are
    /// filtered out so retransmissions do not double-report.
    /// Câu trả lời điều khiển, dùng để phản hồi lên UI. Các lần lặp lại của
    /// cùng một câu trả lời bị lọc bỏ để việc gửi lại không báo trùng.
    var onReply: ((Command, UInt8, ReplyError) -> Void)?

    private var lastReportedReply: (Command, UInt8, ReplyError)?

    // MARK: - Wire builders

    func createRequest(_ block: LogBlock) -> Data {
        Data([Command.create.rawValue, block.id]) + block.createPayload
    }

    func deleteRequest(_ id: UInt8) -> Data {
        Data([Command.delete.rawValue, id])
    }

    /// `periodMs` is rounded to the drone's 10 ms units (§4).
    /// `periodMs` được làm tròn theo đơn vị 10 ms của drone (§4).
    func startRequest(_ id: UInt8, periodMs: Int) -> Data {
        let units = UInt8(max(1, min(255, periodMs / 10)))
        return Data([Command.start.rawValue, id, units])
    }

    func stopRequest(_ id: UInt8) -> Data {
        Data([Command.stop.rawValue, id])
    }

    var resetRequest: Data {
        Data([Command.reset.rawValue])
    }

    // MARK: - Inbound

    /// Handles a settings-channel payload (`<cmd><id><err>`).
    /// Xử lý phần dữ liệu trên channel settings (`<cmd><id><err>`).
    func handleSettings(payload: Data) {
        guard payload.count >= 3,
              let cmd = Command(rawValue: payload[payload.startIndex]) else { return }
        let id = payload[payload.index(payload.startIndex, offsetBy: 1)]
        let err = ReplyError(rawValue: payload[payload.index(payload.startIndex, offsetBy: 2)]) ?? .noCommand

        if cmd == .reset, err == .ok {
            blocks.removeAll()
        }
        if cmd == .delete, err == .ok {
            blocks[id] = nil
        }

        // The drone re-acks retransmissions; report each distinct reply once.
        // Drone re-ack các lần gửi lại; chỉ báo mỗi câu trả lời khác nhau một lần.
        let seen = (cmd, id, err)
        if let previous = lastReportedReply, previous == seen {
            return
        }
        lastReportedReply = seen
        onReply?(cmd, id, err)
    }

    /// Handles a log-data payload (`<id><ts24><values...>`).
    /// Xử lý phần dữ liệu của log (`<id><ts24><values...>`).
    func handleData(payload: Data) {
        guard payload.count >= 4 else { return }
        let id = payload[payload.startIndex]
        guard let block = blocks[id] else { return }
        let tsBytes = payload.dropFirst().prefix(3)
        var ts: UInt32 = 0
        for (shift, byte) in tsBytes.enumerated() {
            ts |= UInt32(byte) << (8 * shift)
        }
        guard let values = block.decodeValues(payload.dropFirst(4)) else { return }
        onSample?(LogSample(blockId: id, timestampMs: ts,
                            values: values, names: block.variables.map { $0.name }))
    }

    // MARK: - Local block bookkeeping

    /// Registers a block locally after the drone has acked its CREATE.
    /// Idempotent: replacing an identical block keeps the newer definition.
    /// Đăng ký một khối ở phía cục bộ sau khi drone đã ack CREATE của nó.
    /// Idempotent: thay thế một khối giống hệt sẽ giữ định nghĩa mới hơn.
    func register(_ block: LogBlock) {
        blocks[block.id] = block
    }

    func block(_ id: UInt8) -> LogBlock? {
        blocks[id]
    }
}
