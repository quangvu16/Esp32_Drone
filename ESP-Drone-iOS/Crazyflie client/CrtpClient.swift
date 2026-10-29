//
//  CrtpClient.swift
//  Crazyflie client
//
//  The CRTP receive path and client state machine. Everything above the
//  transport talks to this class: TOC fetch, parameter read/write, log block
//  setup and telemetry decode, flight-control and navigation commands.
//  Đường nhận CRTP và máy trạng thái của client. Mọi thứ phía trên tầng truyền
//  đều giao tiếp với lớp này: nạp TOC, đọc/ghi tham số, thiết lập khối log và
//  giải mã dữ liệu từ xa, lệnh điều khiển bay và điều hướng.
//
//  Reliability model (the drone side is written to tolerate it):
//  unacknowledged requests are retransmitted every 500 ms, so every handler
//  must be idempotent and a repeated reply must not corrupt state. Pending
//  requests are matched against inbound packets; a match retires the request,
//  a duplicate reply finds no pending entry and is handled as a repeat.
//  Mô hình tin cậy (phía firmware drone được viết để chịu được mô hình này):
//  các yêu cầu chưa có ack được gửi lại mỗi 500 ms, nên mọi hàm xử lý phải
//  idempotent và một câu trả lời lặp lại không được làm hỏng trạng thái. Các
//  yêu cầu đang chờ được khớp với gói tin vào; khớp thì loại yêu cầu khỏi danh
//  sách chờ, một câu trả lời trùng lặp không tìm thấy mục chờ nào và được xử
//  lý như một lần lặp lại.
//

import Foundation

/// One outstanding request, retransmitted until its reply arrives.
/// Một yêu cầu đang chờ, được gửi lại cho đến khi nhận được câu trả lời.
private final class PendingRequest {
    let packet: Data
    let matches: (CrtpPacket) -> Bool
    let onAck: (() -> Void)?
    let onGiveUp: (() -> Void)?
    var retriesLeft: Int

    init(packet: Data, retries: Int, matches: @escaping (CrtpPacket) -> Bool,
         onAck: (() -> Void)?, onGiveUp: (() -> Void)?) {
        self.packet = packet
        self.matches = matches
        self.onAck = onAck
        self.onGiveUp = onGiveUp
        self.retriesLeft = retries
    }
}

/// Transport-agnostic CRTP endpoint for the ESP32 drone.
/// Điểm cuối CRTP độc lập với tầng truyền, dùng cho drone ESP32.
final class CrtpClient {

    // MARK: - Subsystems

    let paramToc = TocFetcher(isLog: false)
    let logToc = TocFetcher(isLog: true)
    let logg = LogClient()

    // MARK: - Callbacks (all delivered on `queue`)

    var onConsole: ((String) -> Void)?
    var onParamTocReady: ((Toc) -> Void)?
    var onLogTocReady: ((Toc) -> Void)?
    var onFlightAck: ((UInt8, UInt8) -> Void)?       // (cmd, status)
    var onNavAck: ((UInt8, UInt8) -> Void)?          // (cmd, status)
    var onParamValue: ((UInt8, CrtpValue) -> Void)?  // (id, value) from read/write echoes
                                                     // (id, value) từ phần echo của lệnh đọc/ghi
    var onConnectionEvent: ((String) -> Void)?
    /// Raw `[cmd][args]` payload of every port 7 channel 2 packet (PROTOCOL.md
    /// §12) — the mission transfer's reply path.
    /// Phần dữ liệu `[cmd][args]` thô của mọi gói cổng 7 kênh 2 (PROTOCOL.md
    /// mục 12) — đường trả lời của phiên truyền nhiệm vụ.
    var onMissionPacket: ((Data) -> Void)?

    private let driver: CrtpDriver
    private let queue = DispatchQueue(label: "crtp.client")
    private var pending: [PendingRequest] = []
    private var resendTimer: DispatchSourceTimer?

    /// The firmware retransmission cadence from the Android client (500 ms).
    /// Chu kỳ gửi lại của firmware theo Android client (500 ms).
    private let resendInterval: TimeInterval = 0.5
    private let maxRetries = 6

    // MARK: - Init

    init(driver: CrtpDriver) {
        self.driver = driver
        driver.setPacketHandler { [weak self] raw in
            self?.queue.async {
                self?.handleInbound(raw)
            }
        }
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now() + resendInterval, repeating: resendInterval)
        timer.setEventHandler { [weak self] in
            self?.retransmit()
        }
        timer.resume()
        resendTimer = timer
    }

    deinit {
        resendTimer?.cancel()
    }

    // MARK: - Sending

    /// Fire-and-forget send (commander packets).
    /// Gửi không chờ phản hồi (các gói tin commander).
    func send(_ packet: CrtpPacket) {
        driver.sendPacket(CrtpChecksum.append(to: packet.raw), callback: nil)
    }

    /// Sends and retransmits every 500 ms until `matches` sees its reply.
    /// Gửi và gửi lại mỗi 500 ms cho đến khi `matches` thấy được câu trả lời.
    func sendReliable(_ packet: CrtpPacket,
                      matches: @escaping (CrtpPacket) -> Bool,
                      onAck: (() -> Void)? = nil,
                      onGiveUp: (() -> Void)? = nil) {
        queue.async {
            let pending = PendingRequest(packet: CrtpChecksum.append(to: packet.raw),
                                         retries: self.maxRetries,
                                         matches: matches,
                                         onAck: onAck, onGiveUp: onGiveUp)
            self.pending.append(pending)
            self.driver.sendPacket(pending.packet, callback: nil)
        }
    }

    private func retransmit() {
        guard !pending.isEmpty else { return }
        for request in pending {
            request.retriesLeft -= 1
            if request.retriesLeft <= 0 {
                request.onGiveUp?()
            } else {
                driver.sendPacket(request.packet, callback: nil)
            }
        }
        pending.removeAll { $0.retriesLeft <= 0 }
    }

    /// Retires every pending request matching the inbound packet. Runs before
    /// dispatch so a reply stops its own retransmissions first.
    /// Loại mọi yêu cầu đang chờ khớp với gói tin vào. Chạy trước khi phân
    /// phối để một câu trả lời ngừng việc gửi lại của chính nó trước.
    private func retire(matching packet: CrtpPacket) {
        let matched = pending.filter { $0.matches(packet) }
        pending.removeAll { $0.matches(packet) }
        matched.forEach { $0.onAck?() }
    }

    // MARK: - Inbound dispatch

    private func handleInbound(_ raw: Data) {
        guard let packet = CrtpPacket(raw: raw) else { return }
        retire(matching: packet)

        switch packet.port {
        case .console:
            if let text = String(data: packet.payload, encoding: .utf8) {
                onConsole?(text)
            }
        case .params:
            handleParams(packet)
        case .logging:
            handleLogging(packet)
        case .generic:
            handleGeneric(packet)
        case .commander:
            break                                // the drone never sends setpoints
                                                 // drone không bao giờ gửi setpoint
        }
    }

    // MARK: - Port 2: parameters

    private enum ParamChannel: UInt8 {
        case toc = 0
        case read = 1
        case write = 2
    }

    private func handleParams(_ packet: CrtpPacket) {
        guard let channel = ParamChannel(rawValue: packet.channel) else { return }
        switch channel {
        case .toc:
            if let next = paramToc.handle(payload: packet.payload) {
                requestTocElement(next, fetcher: paramToc, port: .params)
            }
            if paramToc.state == .complete {
                onParamTocReady?(paramToc.toc)
            }
        case .read, .write:
            // <id><value LE>, echoed on the same channel
            // <id><value LE>, được echo lại trên cùng channel
            let payload = packet.payload
            guard payload.count >= 2 else { return }
            let id = payload[payload.startIndex]
            guard let element = paramToc.toc.element(id),
                  let type = ParamType(rawValue: element.typeCode) else { return }
            let valueBytes = payload.dropFirst(2)
            if let value = CrtpValue.decode(paramType: type, from: valueBytes) {
                onParamValue?(id, value)
            }
        }
    }

    // MARK: - Port 5: logging

    private enum LogChannel: UInt8 {
        case toc = 0
        case settings = 1
        case logData = 2
    }

    private func handleLogging(_ packet: CrtpPacket) {
        guard let channel = LogChannel(rawValue: packet.channel) else { return }
        switch channel {
        case .toc:
            if let next = logToc.handle(payload: packet.payload) {
                requestTocElement(next, fetcher: logToc, port: .logging)
            }
            if logToc.state == .complete {
                onLogTocReady?(logToc.toc)
            }
        case .settings:
            logg.handleSettings(payload: packet.payload)
        case .logData:
            logg.handleData(payload: packet.payload)
        }
    }

    // MARK: - Port 7: flight control (ch 0), navigation (ch 1), missions (ch 2)

    private func handleGeneric(_ packet: CrtpPacket) {
        let payload = packet.payload
        if packet.channel == MissionPacket.channel {
            // Mission replies are `[cmd][args]` (PROTOCOL.md §12) — forward the
            // raw payload; the transfer decodes its own commands.
            // Câu trả lời nhiệm vụ là `[cmd][args]` (PROTOCOL.md mục 12) — chuyển
            // nguyên phần dữ liệu; phiên truyền tự giải mã lệnh của nó.
            guard !payload.isEmpty else { return }
            onMissionPacket?(payload)
            return
        }
        guard payload.count >= 2 else { return }
        // Flight/nav replies are `{hdr, cmd, status}` — after the header is
        // stripped the command is at payload[0], the status at payload[1].
        // Câu trả lời điều khiển bay/định hướng là `{hdr, cmd, status}` — sau khi
        // bỏ header, lệnh nằm ở payload[0], status ở payload[1].
        let command = payload[payload.startIndex]
        let status = payload[payload.index(payload.startIndex, offsetBy: 1)]
        switch packet.channel {
        case 0: onFlightAck?(command, status)
        case 1: onNavAck?(command, status)
        default: break
        }
    }

    // MARK: - TOC fetch

    /// Starts both TOC walks. Safe to call again — a completed fetcher is
    /// skipped and an in-flight one just keeps going.
    /// Bắt đầu hai lượt duyệt TOC. Gọi lại vẫn an toàn — fetcher đã hoàn
    /// thành bị bỏ qua, fetcher đang chạy thì cứ tiếp tục.
    func fetchTocs() {
        queue.async {
            self.startFetch(self.paramToc, port: .params)
            self.startFetch(self.logToc, port: .logging)
        }
    }

    private func startFetch(_ fetcher: TocFetcher, port: CrtpPort) {
        guard fetcher.state == .idle else { return }
        // Channel 0 — the CRTP header byte is (port << 4) | channel.
        // Channel 0 — byte header CRTP là (port << 4) | channel.
        sendReliable(CrtpPacket(port: port, channel: 0, payload: TocFetcher.infoRequest),
                     matches: { p in p.port == port && p.channel == 0 && p.payload.first == 1 },
                     onGiveUp: { [weak self] in
                         self?.onConnectionEvent?("TOC info request timed out on \(port)")
                     })
    }

    private func requestTocElement(_ request: TocFetcher.Request, fetcher: TocFetcher, port: CrtpPort) {
        guard case .element(let id) = request else { return }
        sendReliable(CrtpPacket(port: port, channel: 0, payload: TocFetcher.elementRequest(id)),
                     matches: { p in
                         p.port == port && p.channel == 0 && p.payload.first == 0 &&
                         p.payload.count >= 2 && p.payload[p.payload.index(p.payload.startIndex, offsetBy: 1)] == id
                     },
                     onGiveUp: { [weak self] in
                         // A missing id is never answered (§2); treat it as a hole
                         // and keep walking rather than stalling the fetch.
                         // Một id bị thiếu sẽ không bao giờ được trả lời (§2); coi nó
                         // như một lỗ trống và cứ tiếp tục duyệt thay vì làm nghẽn việc nạp.
                         self?.queue.async {
                             if fetcher.state == .fetchingElements, fetcher.nextId == id {
                                 if let next = fetcher.handle(payload: Data([0, id])) {
                                     self?.requestTocElement(next, fetcher: fetcher, port: port)
                                 }
                                 if fetcher.state == .complete {
                                     if fetcher === self?.paramToc { self?.onParamTocReady?(fetcher.toc) }
                                     if fetcher === self?.logToc { self?.onLogTocReady?(fetcher.toc) }
                                 }
                             }
                         }
                     })
    }

    // MARK: - Parameters (§3)

    /// Reads one parameter; the value arrives via `onParamValue`.
    /// Đọc một tham số; giá trị đến qua `onParamValue`.
    func readParam(id: UInt8) {
        let packet = CrtpPacket(port: .params, channel: ParamChannel.read.rawValue, payload: Data([id]))
        sendReliable(packet,
                     matches: { p in
                         p.port == .params && p.channel == ParamChannel.read.rawValue &&
                         p.payload.first == id
                     })
    }

    /// Writes one parameter; the drone echoes the stored value via `onParamValue`.
    /// Ghi một tham số; drone echo lại giá trị đã lưu qua `onParamValue`.
    func writeParam(id: UInt8, value: CrtpValue) {
        var payload = Data([id])
        payload.append(value.encoded)
        let packet = CrtpPacket(port: .params, channel: ParamChannel.write.rawValue, payload: payload)
        sendReliable(packet,
                     matches: { p in
                         p.port == .params && p.channel == ParamChannel.write.rawValue &&
                         p.payload.first == id
                     })
    }

    /// Reads every parameter in the TOC (drives the editor's "robot:" labels).
    /// Đọc mọi tham số trong TOC (cung cấp dữ liệu cho nhãn "robot:" của trình chỉnh sửa).
    func readAllParams() {
        for id in 0..<paramToc.toc.count {
            readParam(id: id)
        }
    }

    /// Convenience for the typed parameter table in PROTOCOL.md §3.
    /// Hàm tiện lợi cho bảng tham số có kiểu trong PROTOCOL.md §3.
    func writeFloat(group: String, name: String, _ value: Float) {
        guard let element = paramToc.toc.element(group: group, name: name) else { return }
        writeParam(id: element.id, value: .float(value))
    }

    func writeUInt16(group: String, name: String, _ value: UInt16) {
        guard let element = paramToc.toc.element(group: group, name: name) else { return }
        writeParam(id: element.id, value: .uint16(value))
    }

    // MARK: - Log blocks (§4)

    /// Creates a block on the drone and registers it locally once acked.
    /// Variables are checked against the 25-byte value budget first.
    /// Tạo một khối log trên drone và đăng ký cục bộ khi đã có ack.
    /// Các biến được kiểm tra với ngân sách 25 byte giá trị trước.
    func createBlock(id: UInt8, variables: [LogVariable],
                     periodMs: Int = 100,
                     onDone: ((Bool, LogClient.ReplyError) -> Void)? = nil) {
        let block = LogBlock(id: id, variables: variables)
        guard block.fitsBudget else {
            onDone?(false, .tooLarge)
            return
        }
        let packet = CrtpPacket(port: .logging, channel: LogChannel.settings.rawValue,
                                payload: logg.createRequest(block))
        sendReliable(packet,
                     matches: { p in self.isLogReply(p, cmd: .create, id: id) },
                     onAck: { [weak self] in
                         self?.logg.register(block)
                         self?.startBlock(id: id, periodMs: periodMs)
                         onDone?(true, .ok)
                     },
                     onGiveUp: {
                         onDone?(false, .noMemory)
                     })
    }

    func startBlock(id: UInt8, periodMs: Int) {
        let packet = CrtpPacket(port: .logging, channel: LogChannel.settings.rawValue,
                                payload: logg.startRequest(id, periodMs: periodMs))
        sendReliable(packet, matches: { p in self.isLogReply(p, cmd: .start, id: id) })
    }

    func stopBlock(id: UInt8) {
        let packet = CrtpPacket(port: .logging, channel: LogChannel.settings.rawValue,
                                payload: logg.stopRequest(id))
        sendReliable(packet, matches: { p in self.isLogReply(p, cmd: .stop, id: id) })
    }

    func deleteBlock(id: UInt8) {
        let packet = CrtpPacket(port: .logging, channel: LogChannel.settings.rawValue,
                                payload: logg.deleteRequest(id))
        sendReliable(packet, matches: { p in self.isLogReply(p, cmd: .delete, id: id) })
    }

    private func isLogReply(_ p: CrtpPacket, cmd: LogClient.Command, id: UInt8) -> Bool {
        p.port == .logging && p.channel == LogChannel.settings.rawValue &&
        p.payload.count >= 3 &&
        p.payload[p.payload.startIndex] == cmd.rawValue &&
        p.payload[p.payload.index(p.payload.startIndex, offsetBy: 1)] == id
    }

    // MARK: - Flight control (§5)

    enum FlightCommand: UInt8 {
        case setMode = 0x01
        case takeoff = 0x02
        case land = 0x03
        case setHover = 0x04
        case emergency = 0x05
        case setHeading = 0x06
        case arm = 0x07
        case disarm = 0x08
        case save = 0x09
        case setDefaults = 0x0A
        case load = 0x0B
        case motorTest = 0x0C
        case link = 0x10
    }

    enum FlightMode: UInt8 {
        case manual = 0
        case hold = 1
        case auto = 2
        case follow = 3
        case rth = 4
    }

    private func sendFlight(_ command: FlightCommand, args: Data = Data()) {
        var payload = Data([command.rawValue])
        payload.append(args)
        let packet = CrtpPacket(port: .generic, channel: 0, payload: payload)
        // The firmware replies [cmd][status] (PROTOCOL.md §5), so the command is at
        // payload[0] and the status at payload[1] — the same indices handleGeneric reads.
        // Firmware trả lời [cmd][status] (PROTOCOL.md §5), nên lệnh ở payload[0] và status
        // ở payload[1] — đúng các chỉ số mà handleGeneric đọc.
        sendReliable(packet,
                     matches: { p in
                         p.port == .generic && p.channel == 0 &&
                         p.payload.count >= 2 && p.payload.first == command.rawValue
                     })
    }

    /// Bench motor test (§5): fire-and-forget, so the caller must drive it from a
    /// timer at ≤300 ms to hold the firmware watchdog. `sendReliable` retransmits at
    /// 500 ms — too slow — so this deliberately does not use it.
    /// Thử động cơ trên bàn (§5): gửi không chờ, nên người gọi phải chạy từ một timer
    /// ở ≤300 ms để giữ watchdog của firmware. `sendReliable` gửi lại ở 500 ms — quá chậm —
    /// nên lệnh này chủ động không dùng nó.
    func motorTest(enable: Bool, m1: UInt16, m2: UInt16, m3: UInt16, m4: UInt16) {
        var payload = Data([FlightCommand.motorTest.rawValue, enable ? 1 : 0])
        for thrust in [m1, m2, m3, m4] {
            withUnsafeBytes(of: thrust.littleEndian) { payload.append(contentsOf: $0) }
        }
        send(CrtpPacket(port: .generic, channel: 0, payload: payload))
    }

    /// Stops a running bench motor test (enable=0). Send once on stop/disappear.
    /// Dừng thử động cơ đang chạy (enable=0). Gửi một lần khi dừng/ra khỏi màn hình.
    func motorTestStop() {
        motorTest(enable: false, m1: 1000, m2: 1000, m3: 1000, m4: 1000)
    }

    func setFlightMode(_ mode: FlightMode) {
        sendFlight(.setMode, args: Data([mode.rawValue]))
    }

    func takeoff(hoverThrust: UInt16) {
        var args = Data()
        withUnsafeBytes(of: hoverThrust.littleEndian) { args.append(contentsOf: $0) }
        sendFlight(.takeoff, args: args)
    }

    func land() { sendFlight(.land) }

    func setHoverThrust(_ thrust: UInt16) {
        var args = Data()
        withUnsafeBytes(of: thrust.littleEndian) { args.append(contentsOf: $0) }
        sendFlight(.setHover, args: args)
    }

    /// Always honoured by the firmware — props off now.
    /// Firmware luôn thực hiện lệnh này — tắt cánh quạt ngay lập tức.
    func emergency() {
        sendFlight(.emergency)
    }

    func setHeading(_ degrees: Float) {
        var args = Data()
        withUnsafeBytes(of: degrees.bitPattern.littleEndian) { args.append(contentsOf: $0) }
        sendFlight(.setHeading, args: args)
    }

    func arm() { sendFlight(.arm) }
    func disarm() { sendFlight(.disarm) }
    func saveGains() { sendFlight(.save) }
    func restoreDefaults() { sendFlight(.setDefaults) }
    func loadGains() { sendFlight(.load) }

    /// Stores the transport profile; the board reboots in ~1.5 s (§8).
    /// Lưu hồ sơ của tầng truyền; bo mạch khởi động lại trong ~1,5 s (§8).
    func setLinkProfile(_ mode: UInt8) {
        sendFlight(.link, args: Data([mode]))
    }

    // MARK: - Navigation (§6)

    enum NavCommand: UInt8 {
        case target = 0x20
        case reset = 0x21
        case align = 0x22
        case origin = 0x23
    }

    private func sendNav(_ command: NavCommand, args: Data = Data()) {
        var payload = Data([command.rawValue])
        payload.append(args)
        let packet = CrtpPacket(port: .generic, channel: 1, payload: payload)
        // The reply echoes the command at payload[0] with a status byte after
        // it (`{NAV_HDR, cmd, status}` on the wire).
        // Câu trả lời lặp lại lệnh tại payload[0] kèm byte status phía sau
        // (`{NAV_HDR, cmd, status}` trên đường truyền).
        sendReliable(packet,
                     matches: { p in
                         p.port == .generic && p.channel == 1 &&
                         p.payload.count >= 2 && p.payload.first == command.rawValue
                     })
    }

    /// Target offset from the launch point, north/east metres (§9).
    /// Độ lệch của mục tiêu so với điểm cất cánh, theo mét hướng bắc/đông (§9).
    func sendNavTarget(north: Float, east: Float) {
        var args = Data()
        withUnsafeBytes(of: north.bitPattern.littleEndian) { args.append(contentsOf: $0) }
        withUnsafeBytes(of: east.bitPattern.littleEndian) { args.append(contentsOf: $0) }
        sendNav(.target, args: args)
    }

    func resetNav() {
        sendNav(.reset)
    }

    /// Compass bearing of the nose at launch — the one frame the drone cannot
    /// measure itself (no magnetometer).
    /// Phương hướng của mũi drone khi cất cánh theo la bàn — tham chiếu duy
    /// nhất mà drone không thể tự đo (không có từ kế).
    func sendNavAlign(alphaDegrees: Float) {
        var args = Data()
        withUnsafeBytes(of: alphaDegrees.bitPattern.littleEndian) { args.append(contentsOf: $0) }
        sendNav(.align, args: args)
    }

    /// The mission home in WGS-84 degrees (§12) — the only thing that converts
    /// mission items and MAVLink telemetry to and from the launch frame. Pair
    /// it with ALIGN before Upload/Start.
    /// Vị trí home của nhiệm vụ theo độ WGS-84 (mục 12) — thứ duy nhất chuyển
    /// đổi giữa mục nhiệm vụ/telemetry MAVLink và khung cất cánh. Gửi kèm ALIGN
    /// trước Upload/Start.
    func sendNavOrigin(latitude: Double, longitude: Double) {
        var args = Data()
        withUnsafeBytes(of: latitude.bitPattern.littleEndian) { args.append(contentsOf: $0) }
        withUnsafeBytes(of: longitude.bitPattern.littleEndian) { args.append(contentsOf: $0) }
        sendNav(.origin, args: args)
    }
}
