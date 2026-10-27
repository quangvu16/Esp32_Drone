//
//  LogClient.swift
//  Crazyflie client
//
//  Log block model and telemetry decode for CRTP port 5 (PROTOCOL.md §4).
//  Blocks are created from TOC variables, started at a period in 10 ms units,
//  and data packets `<id><ts24><values...>` are decoded against the block's
//  variable list.
//
//  Wire-format trap: the Android `Logg.create()` sends a fixed 31-byte buffer
//  with zero padding after the real `<type><tocId>` pairs, and log type codes
//  start at 1 — so a zero type byte terminates the list on the drone. We build
//  trimmed requests instead (the firmware accepts both), but the decode path
//  here follows the same rule so a padded request would parse identically.
//

import Foundation

/// One `<type><tocId>` entry in a log block.
struct LogVariable {
    let tocId: UInt8
    let logType: LogType
    let name: String

    var size: Int { logType.size }
}

/// A decoded log data packet.
struct LogSample {
    let blockId: UInt8
    /// Milliseconds since the block started (24-bit, wraps).
    let timestampMs: UInt32
    /// One value per variable, in block order.
    let values: [CrtpValue]
    /// Variable names in block order, so consumers can key by name without
    /// keeping the block definition around.
    let names: [String]

    /// `nil` when the block has no such variable (or an undecodable value).
    subscript(name: String) -> Double? {
        guard let index = names.firstIndex(of: name),
              values.indices.contains(index) else { return nil }
        return values[index].asDouble
    }

}

/// Client-side mirror of a drone-side log block.
final class LogBlock {
    let id: UInt8
    private(set) var variables: [LogVariable]

    /// Total value bytes per data packet. PROTOCOL.md §4 caps this at 25:
    /// `<header><id><ts24>` already take 5 of the 30 CRTP bytes.
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
    var createPayload: Data {
        var out = Data(capacity: variables.count * 2)
        for v in variables {
            out.append(v.logType.rawValue)
            out.append(v.tocId)
        }
        return out
    }

    /// Decodes the value section of a data packet into typed values.
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
final class LogClient {
    /// Control commands (§4).
    enum Command: UInt8 {
        case create = 0
        case append = 1
        case delete = 2
        case start = 3
        case stop = 4
        case reset = 5
    }

    /// Error codes the drone returns (§4).
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
    var onSample: ((LogSample) -> Void)?
    /// Control replies, for UI feedback. Repeats of the same reply are
    /// filtered out so retransmissions do not double-report.
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
        let seen = (cmd, id, err)
        if let previous = lastReportedReply, previous == seen {
            return
        }
        lastReportedReply = seen
        onReply?(cmd, id, err)
    }

    /// Handles a log-data payload (`<id><ts24><values...>`).
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
    func register(_ block: LogBlock) {
        blocks[block.id] = block
    }

    func block(_ id: UInt8) -> LogBlock? {
        blocks[id]
    }
}
