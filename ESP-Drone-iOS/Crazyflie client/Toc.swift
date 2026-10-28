//
//  Toc.swift
//  Crazyflie client
//
//  TOC fetch state machine for the parameter (port 2) and log (port 5) TOCs
//  (PROTOCOL.md §2): info request -> nItems + crc, then one element request
//  per id. The drone answers only existing ids, so a request for a missing id
//  is simply never acknowledged — the retransmit queue ages those out.
//  Máy trạng thái nạp TOC cho TOC tham số (port 2) và TOC log (port 5)
//  (PROTOCOL.md §2): yêu cầu info -> nItems + crc, rồi mỗi id một yêu cầu
//  element. Drone chỉ trả lời các id hiện có, nên yêu cầu cho id bị thiếu
//  đơn giản là không bao giờ được ack — hàng đợi gửi lại sẽ loại dần các yêu
//  cầu đó.
//
//  Retransmitted requests and duplicated replies must not corrupt state: an
//  element reply for an id already stored is ignored, and the info reply is
//  only acted on once (a second one just re-arms the same element walk).
//  Các yêu cầu được gửi lại và câu trả lời trùng lặp không được làm hỏng
//  trạng thái: câu trả lời element cho id đã lưu bị bỏ qua, và câu trả lời
//  info chỉ được xử lý một lần (lần thứ hai chỉ kích hoạt lại cùng một lượt
//  duyệt element).
//

import Foundation

/// One TOC entry: `type|access` byte split into the type code and read-only
/// flag, plus the two NUL-terminated strings (PROTOCOL.md §2).
/// Một mục TOC: byte `type|access` tách thành mã kiểu và cờ chỉ đọc, cộng
/// thêm hai chuỗi kết thúc bằng NUL (PROTOCOL.md §2).
struct TocElement: Equatable {
    let id: UInt8
    let group: String
    let name: String
    let typeCode: UInt8
    let isReadOnly: Bool

    /// Element byte width, derived from the type code. Parameter and log TOCs
    /// use different code tables, so the caller passes which one applies.
    /// Bề rộng byte của phần tử, suy ra từ mã kiểu. TOC tham số và TOC log dùng
    /// các bảng mã khác nhau, nên nơi gọi truyền vào bảng nào được áp dụng.
    func size(isLog: Bool) -> Int {
        if isLog {
            return LogType(rawValue: typeCode)?.size ?? 0
        }
        return ParamType(rawValue: typeCode)?.size ?? 0
    }

    var fullName: String { "\(group).\(name)" }
}

/// The elements of one TOC, keyed by id.
/// Các phần tử của một TOC, được đánh khóa theo id.
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
    /// Idempotent: một câu trả lời trùng cho id đã lưu giữ TOC không đổi và
    /// trả về `false` để nơi gọi có thể bỏ qua lần lặp lại.
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
/// Phân tích câu trả lời trên channel TOC và điều khiển lượt duyệt element.
///
/// Nơi gọi làm chủ đường truyền: `requestInfo` / `requestElement` được gửi qua
/// hàng đợi gửi lại, và mọi gói tin đến trên channel TOC được chuyển cho
/// `handle`. Các chuyển trạng thái xem sơ đồ trên.
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
    /// Được gọi khi đã duyệt xong toàn bộ TOC.
    var onCompleted: ((Toc) -> Void)?
    /// Called for each stored element (UI progress).
    /// Được gọi cho mỗi phần tử đã lưu (tiến trình trên UI).
    var onElement: ((TocElement) -> Void)?

    /// The next id to request, or nil when the walk is finished.
    /// Id tiếp theo cần yêu cầu, hoặc nil khi lượt duyệt đã kết thúc.
    private(set) var nextId: UInt8?

    init(isLog: Bool) {
        toc = Toc(isLog: isLog)
    }

    /// Bytes for a TOC info request: `<1>` on channel 0.
    /// Byte cho yêu cầu TOC info: `<1>` trên channel 0.
    static var infoRequest: Data { Data([1]) }

    /// Bytes for an element request: `<0><id>`.
    /// Byte cho yêu cầu element: `<0><id>`.
    static func elementRequest(_ id: UInt8) -> Data { Data([0, id]) }

    /// True while the fetcher wants the resend queue to keep the given request alive.
    /// Trả về true khi fetcher muốn hàng đợi gửi lại giữ cho yêu cầu đã cho còn hiệu lực.
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
    /// Xử lý một gói tin từ channel TOC (byte `<cmd>` là payload[0]).
    /// Trả về yêu cầu cần (gửi lại) tiếp theo, nếu có.
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
        // Câu trả lời info lặp lại sẽ khởi động lại lượt duyệt từ id bị thiếu
        // đầu tiên, mà đó chính là vị trí hiện tại — vô hại.
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
            // Câu trả lời trùng hoặc cũ: đã lưu rồi, hoặc không phải thứ ta yêu
            // cầu. Dù thế nào cũng không được làm dịch chuyển lượt duyệt.
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
    /// Trỏ `nextId` tới id đầu tiên chưa được lưu.
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
