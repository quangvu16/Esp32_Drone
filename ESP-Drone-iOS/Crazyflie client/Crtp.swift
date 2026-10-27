//
//  Crtp.swift
//  Crazyflie client
//
//  CRTP framing and typed value codec for the ESP32 drone protocol.
//  Byte layouts follow PROTOCOL.md: the header packs port/channel, values are
//  little-endian, and the parameter and log TOCs use *different* type-code
//  tables (§2) — that is why there are two enums here.
//

import Foundation

enum CrtpPort: UInt8 {
    case console = 0
    case params = 2
    case commander = 3
    case logging = 5
    case generic = 7
}

/// Parameter type codes — low nibble of the TOC type byte (PROTOCOL.md §2).
enum ParamType: UInt8 {
    case int8 = 0x00
    case int16 = 0x01
    case int32 = 0x02
    case int64 = 0x03
    case float = 0x06
    case double = 0x07
    case uint8 = 0x08
    case uint16 = 0x09
    case uint32 = 0x0A
    case uint64 = 0x0B

    var size: Int {
        switch self {
        case .int8, .uint8: return 1
        case .int16, .uint16: return 2
        case .int32, .uint32, .float: return 4
        case .int64, .uint64, .double: return 8
        }
    }
}

/// Log type codes — a different numbering from the parameter codes (§2).
enum LogType: UInt8 {
    case uint8 = 0x01
    case uint16 = 0x02
    case uint32 = 0x03
    case int8 = 0x04
    case int16 = 0x05
    case int32 = 0x06
    case float = 0x07

    var size: Int {
        switch self {
        case .uint8, .int8: return 1
        case .uint16, .int16: return 2
        case .uint32, .int32, .float: return 4
        }
    }
}

/// A decoded little-endian scalar. Read back as `double` for maths and charts.
enum CrtpValue {
    case int8(Int8)
    case uint8(UInt8)
    case int16(Int16)
    case uint16(UInt16)
    case int32(Int32)
    case uint32(UInt32)
    case int64(Int64)
    case uint64(UInt64)
    case float(Float)
    case double(Double)

    var asDouble: Double {
        switch self {
        case .int8(let v): return Double(v)
        case .uint8(let v): return Double(v)
        case .int16(let v): return Double(v)
        case .uint16(let v): return Double(v)
        case .int32(let v): return Double(v)
        case .uint32(let v): return Double(v)
        case .int64(let v): return Double(v)
        case .uint64(let v): return Double(v)
        case .float(let v): return Double(v)
        case .double(let v): return v
        }
    }

    var asFloat: Float { Float(asDouble) }

    var size: Int {
        switch self {
        case .int8, .uint8: return 1
        case .int16, .uint16: return 2
        case .int32, .uint32, .float: return 4
        case .int64, .uint64, .double: return 8
        }
    }

    /// Little-endian wire bytes (PROTOCOL.md §3).
    var encoded: Data {
        var out = Data(capacity: size)
        switch self {
        case .int8(let v): out.append(UInt8(bitPattern: v))
        case .uint8(let v): out.append(v)
        case .int16(let v): withUnsafeBytes(of: v.littleEndian) { out.append(contentsOf: $0) }
        case .uint16(let v): withUnsafeBytes(of: v.littleEndian) { out.append(contentsOf: $0) }
        case .int32(let v): withUnsafeBytes(of: v.littleEndian) { out.append(contentsOf: $0) }
        case .uint32(let v): withUnsafeBytes(of: v.littleEndian) { out.append(contentsOf: $0) }
        case .int64(let v): withUnsafeBytes(of: v.littleEndian) { out.append(contentsOf: $0) }
        case .uint64(let v): withUnsafeBytes(of: v.littleEndian) { out.append(contentsOf: $0) }
        case .float(let v):
            withUnsafeBytes(of: v.bitPattern.littleEndian) { out.append(contentsOf: $0) }
        case .double(let v):
            withUnsafeBytes(of: v.bitPattern.littleEndian) { out.append(contentsOf: $0) }
        }
        return out
    }

    /// Builds the value of the given parameter type from little-endian bytes.
    static func decode(paramType: ParamType, from data: Data) -> CrtpValue? {
        guard data.count >= paramType.size else { return nil }
        return data.withUnsafeBytes { (raw: UnsafeRawBufferPointer) -> CrtpValue? in
            guard let base = raw.baseAddress else { return nil }
            switch paramType {
            case .int8: return .int8(base.loadUnaligned(as: Int8.self))
            case .uint8: return .uint8(base.loadUnaligned(as: UInt8.self))
            case .int16: return .int16(base.loadUnaligned(as: Int16.self).littleEndian)
            case .uint16: return .uint16(base.loadUnaligned(as: UInt16.self).littleEndian)
            case .int32: return .int32(base.loadUnaligned(as: Int32.self).littleEndian)
            case .uint32: return .uint32(base.loadUnaligned(as: UInt32.self).littleEndian)
            case .int64: return .int64(base.loadUnaligned(as: Int64.self).littleEndian)
            case .uint64: return .uint64(base.loadUnaligned(as: UInt64.self).littleEndian)
            case .float: return .float(Float(bitPattern: base.loadUnaligned(as: UInt32.self).littleEndian))
            case .double: return .double(Double(bitPattern: base.loadUnaligned(as: UInt64.self).littleEndian))
            }
        }
    }

    static func decode(logType: LogType, from data: Data) -> CrtpValue? {
        guard data.count >= logType.size else { return nil }
        return data.withUnsafeBytes { (raw: UnsafeRawBufferPointer) -> CrtpValue? in
            guard let base = raw.baseAddress else { return nil }
            switch logType {
            case .uint8: return .uint8(base.loadUnaligned(as: UInt8.self))
            case .int8: return .int8(base.loadUnaligned(as: Int8.self))
            case .uint16: return .uint16(base.loadUnaligned(as: UInt16.self).littleEndian)
            case .int16: return .int16(base.loadUnaligned(as: Int16.self).littleEndian)
            case .uint32: return .uint32(base.loadUnaligned(as: UInt32.self).littleEndian)
            case .int32: return .int32(base.loadUnaligned(as: Int32.self).littleEndian)
            case .float: return .float(Float(bitPattern: base.loadUnaligned(as: UInt32.self).littleEndian))
            }
        }
    }
}

/// One complete CRTP packet: header byte plus the payload that follows it.
/// The transport checksum (UDP/SPP) is added and stripped by the link layer,
/// so everything above `CrtpDriver` works with raw CRTP bytes only.
struct CrtpPacket {
    let port: CrtpPort
    let channel: UInt8
    let payload: Data

    init(port: CrtpPort, channel: UInt8, payload: Data) {
        self.port = port
        self.channel = channel & 0x03
        self.payload = payload
    }

    /// Parses raw CRTP bytes (header included, no checksum).
    init?(raw: Data) {
        guard raw.count >= 1 else { return nil }
        let header = raw[raw.startIndex]
        guard let port = CrtpPort(rawValue: header >> 4) else { return nil }
        self.port = port
        self.channel = header & 0x03
        self.payload = raw.count > 1 ? raw.dropFirst() : Data()
    }

    var header: UInt8 { (port.rawValue << 4) | channel }

    /// Full CRTP bytes: header plus payload. This is what goes on the wire
    /// (plus a checksum byte on UDP/SPP).
    var raw: Data {
        var out = Data(capacity: payload.count + 1)
        out.append(header)
        out.append(payload)
        return out
    }
}

enum CrtpChecksum {
    /// Low byte of the sum of all CRTP bytes including the header (§1).
    static func of(_ crtp: Data) -> UInt8 {
        crtp.reduce(0) { UInt8(truncatingIfNeeded: Int($0) + Int($1)) }
    }

    /// Appends the checksum byte for UDP/SPP framing.
    static func append(to crtp: Data) -> Data {
        var out = crtp
        out.append(of(crtp))
        return out
    }

    /// Strips and verifies the trailing checksum. Returns the CRTP bytes.
    static func strip(from framed: Data) -> Data? {
        guard framed.count >= 2 else { return nil }
        let body = framed.dropLast()
        guard framed[framed.index(before: framed.endIndex)] == of(Data(body)) else { return nil }
        return Data(body)
    }
}
