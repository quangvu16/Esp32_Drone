//
//  Toc.swift
//  Crazyflie client
//
//  TOC fetch state machine for the parameter (port 2) and log (port 5) TOCs
//  (PROTOCOL.md §2): info request -> nItems + crc, then one element request
//  per id. The drone answers only existing ids, so a request for a missing id
//  is simply never acknowledged — the retransmit queue ages those out.
//
//  Retransmitted requests and duplicated replies must not corrupt state: an
//  element reply for an id already stored is ignored, and the info reply is
//  only acted on once (a second one just re-arms the same element walk).
//

import Foundation

/// One TOC entry: `type|access` byte split into the type code and read-only
/// flag, plus the two NUL-terminated strings (PROTOCOL.md §2).
struct TocElement: Equatable {
    let id: UInt8
    let group: String
    let name: String
    let typeCode: UInt8
    let isReadOnly: Bool

    /// Element byte width, derived from the type code. Parameter and log TOCs
    /// use different code tables, so the caller passes which one applies.
    func size(isLog: Bool) -> Int {
        if isLog {
            return LogType(rawValue: typeCode)?.size ?? 0
        }
        return ParamType(rawValue: typeCode)?.size ?? 0
    }

    var fullName: String { "\(group).\(name)" }
}

/// The elements of one TOC, keyed by id.
final class Toc {
    let isLog: Bool
    private(set) var elements: [UInt8: TocElement] = [:]
    private(set) var count: UInt8 = 0
    private(set) var crc: UInt32 = 0

    init(isLog: Bool) {
        self.isLog = isLog
    }

    var isComplete: Bool {
        guard count > 0 else { return false }
        for id in 0..<count where elements[id] == nil {
            return false
        }
        return true
    }

    func element(_ id: UInt8) -> TocElement? {
        elements[id]
    }

    func element(group: String, name: String) -> TocElement? {
        elements.values.first { $0.group == group && $0.name == name }
    }

    func apply(info nItems: UInt8, crc: UInt32) {
        count = nItems
        self.crc = crc
    }

    /// Idempotent: a duplicate reply for a stored id leaves the TOC unchanged
    /// and reports `false` so callers can ignore the repeat.
    @discardableResult
    func apply(element: TocElement) -> Bool {
        guard elements[element.id] == nil else { return false }
        elements[element.id] = element
        return true
    }
}

/// Parses TOC channel replies and drives the element walk.
///
/// The caller owns the wire: `requestInfo` / `requestElement` are sent through
/// the resend queue, and every inbound TOC-channel packet is handed to
/// `handle`. State transitions:
///
///     idle --start--> waitingInfo --info reply--> fetchingElements --done--> complete
final class TocFetcher {
    enum State {
        case idle
        case waitingInfo
        case fetchingElements
        case complete
    }

    private(set) var state: State = .idle
    let toc: Toc

    /// Sent when the whole TOC has been walked.
    var onCompleted: ((Toc) -> Void)?
    /// Called for each stored element (UI progress).
    var onElement: ((TocElement) -> Void)?

    /// The next id to request, or nil when the walk is finished.
    private(set) var nextId: UInt8?

    init(isLog: Bool) {
        toc = Toc(isLog: isLog)
    }

    /// Bytes for a TOC info request: `<1>` on channel 0.
    static var infoRequest: Data { Data([1]) }

    /// Bytes for an element request: `<0><id>`.
    static func elementRequest(_ id: UInt8) -> Data { Data([0, id]) }

    /// True while the fetcher wants the resend queue to keep the given request alive.
    func wants(_ kind: Request) -> Bool {
        switch (state, kind) {
        case (.waitingInfo, .info): return true
        case (.fetchingElements, .element(let id)): return id == nextId
        default: return false
        }
    }

    enum Request: Equatable {
        case info
        case element(UInt8)
    }

    /// Handles one packet from the TOC channel (the `<cmd>` byte is payload[0]).
    /// Returns the request that should be (re)sent next, if any.
    func handle(payload: Data) -> Request? {
        guard payload.count >= 1 else { return nil }
        switch payload[payload.startIndex] {
        case 1:                                     // CMD_TOC_INFO reply
            return handleInfo(payload: payload)
        case 0:                                     // CMD_TOC_ELEMENT reply
            return handleElement(payload: payload)
        default:
            return nil
        }
    }

    private func handleInfo(payload: Data) -> Request? {
        // <1><uint8 nItems><uint32 crc LE>
        guard payload.count >= 6 else { return nil }
        let nItems = payload[payload.index(payload.startIndex, offsetBy: 1)]
        let crc: UInt32 = payload.withUnsafeBytes { raw in
            raw.loadUnaligned(fromByteOffset: 2, as: UInt32.self).littleEndian
        }
        toc.apply(info: nItems, crc: crc)

        if nItems == 0 {
            state = .complete
            onCompleted?(toc)
            return nil
        }
        state = .fetchingElements
        // A repeated info reply restarts the walk from the first missing id,
        // which is where we already are — harmless.
        advance()
        return nextId.map { .element($0) }
    }

    private func handleElement(payload: Data) -> Request? {
        // <0><id><type|access><group\0><name\0>
        guard payload.count >= 4 else { return nil }
        let id = payload[payload.index(payload.startIndex, offsetBy: 1)]
        guard state == .fetchingElements, id == nextId else {
            // Duplicate or stale reply: already stored, or not what we asked
            // for. Either way it must not move the walk.
            return nil
        }

        let typeByte = payload[payload.index(payload.startIndex, offsetBy: 2)]
        let strings = payload.dropFirst(3)
        guard let nul = strings.firstIndex(of: 0) else { return nil }
        let group = String(data: strings.prefix(upTo: nul), encoding: .utf8) ?? ""
        let rest = strings[strings.index(after: nul)...]
        guard let nul2 = rest.firstIndex(of: 0) else { return nil }
        let name = String(data: rest.prefix(upTo: nul2), encoding: .utf8) ?? ""

        let element = TocElement(
            id: id,
            group: group,
            name: name,
            typeCode: typeByte & 0x0F,
            isReadOnly: (typeByte & 0x40) != 0
        )
        toc.apply(element: element)
        onElement?(element)

        advance()
        return nextId.map { .element($0) }
    }

    /// Points `nextId` at the first id not yet stored.
    private func advance() {
        var id: UInt16 = 0
        while id < UInt16(toc.count) {
            if toc.element(UInt8(id)) == nil {
                nextId = UInt8(id)
                return
            }
            id += 1
        }
        nextId = nil
        state = .complete
        onCompleted?(toc)
    }
}
