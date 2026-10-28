//
//  CrtpClient.swift
//  Crazyflie client
//
//  The CRTP receive path and client state machine. Everything above the
//  transport talks to this class: TOC fetch, parameter read/write, log block
//  setup and telemetry decode, flight-control and navigation commands.
//
//  Reliability model (the drone side is written to tolerate it):
//  unacknowledged requests are retransmitted every 500 ms, so every handler
//  must be idempotent and a repeated reply must not corrupt state. Pending
//  requests are matched against inbound packets; a match retires the request,
//  a duplicate reply finds no pending entry and is handled as a repeat.
//

import Foundation

/// One outstanding request, retransmitted until its reply arrives.
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
    var onConnectionEvent: ((String) -> Void)?

    private let driver: CrtpDriver
    private let queue = DispatchQueue(label: "crtp.client")
    private var pending: [PendingRequest] = []
    private var resendTimer: DispatchSourceTimer?

    /// The firmware retransmission cadence from the Android client (500 ms).
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
    func send(_ packet: CrtpPacket) {
        driver.sendPacket(CrtpChecksum.append(to: packet.raw), callback: nil)
    }

    /// Sends and retransmits every 500 ms until `matches` sees its reply.
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

    // MARK: - Port 7: flight control (ch 0) and navigation (ch 1)

    private func handleGeneric(_ packet: CrtpPacket) {
        let payload = packet.payload
        guard payload.count >= 2 else { return }
        let command = payload[payload.index(payload.startIndex, offsetBy: 1)]
        let status = payload.count >= 3 ? payload[payload.index(payload.startIndex, offsetBy: 2)] : 1
        switch packet.channel {
        case 0: onFlightAck?(command, status)
        case 1: onNavAck?(command, status)
        default: break
        }
    }

    // MARK: - TOC fetch

    /// Starts both TOC walks. Safe to call again — a completed fetcher is
    /// skipped and an in-flight one just keeps going.
    func fetchTocs() {
        queue.async {
            self.startFetch(self.paramToc, port: .params)
            self.startFetch(self.logToc, port: .logging)
        }
    }

    private func startFetch(_ fetcher: TocFetcher, port: CrtpPort) {
        guard fetcher.state == .idle else { return }
        let header = (port.rawValue << 4) | 0     // channel 0
        let packet = CrtpPacket(port: port, channel: 0, payload: TocFetcher.infoRequest)
        sendReliable(CrtpPacket(port: port, channel: 0, payload: TocFetcher.infoRequest),
                     matches: { p in p.port == port && p.channel == 0 && p.payload.first == 1 },
                     onGiveUp: { [weak self] in
                         self?.onConnectionEvent?("TOC info request timed out on \(port)")
                     })
        _ = header
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
    func readParam(id: UInt8) {
        let packet = CrtpPacket(port: .params, channel: ParamChannel.read.rawValue, payload: Data([id]))
        sendReliable(packet,
                     matches: { p in
                         p.port == .params && p.channel == ParamChannel.read.rawValue &&
                         p.payload.first == id
                     })
    }

    /// Writes one parameter; the drone echoes the stored value via `onParamValue`.
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
    func readAllParams() {
        for id in 0..<paramToc.toc.count {
            readParam(id: id)
        }
    }

    /// Convenience for the typed parameter table in PROTOCOL.md §3.
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
        sendReliable(packet,
                     matches: { p in
                         p.port == .generic && p.channel == 0 &&
                         // reply payload is <cmd><status>: the command is the FIRST byte
                         p.payload.count >= 2 && p.payload.first == command.rawValue
                     })
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
    func setLinkProfile(_ mode: UInt8) {
        sendFlight(.link, args: Data([mode]))
    }

    // MARK: - Navigation (§6)

    enum NavCommand: UInt8 {
        case target = 0x20
        case reset = 0x21
        case align = 0x22
    }

    private func sendNav(_ command: NavCommand, args: Data = Data()) {
        var payload = Data([command.rawValue])
        payload.append(args)
        let packet = CrtpPacket(port: .generic, channel: 1, payload: payload)
        sendReliable(packet,
                     matches: { p in
                         p.port == .generic && p.channel == 1 &&
                         // reply payload is <cmd><status>: the command is the FIRST byte
                         p.payload.count >= 2 && p.payload.first == command.rawValue
                     })
    }

    /// Target offset from the launch point, north/east metres (§9).
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
    func sendNavAlign(alphaDegrees: Float) {
        var args = Data()
        withUnsafeBytes(of: alphaDegrees.bitPattern.littleEndian) { args.append(contentsOf: $0) }
        sendNav(.align, args: args)
    }
}
