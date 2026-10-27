//
//  TelemetryStore.swift
//  Crazyflie client
//
//  Fan-out for decoded log samples plus a short history for charts and
//  statistics. Log block ids are allocated here so the screens cannot fight
//  over the firmware's 3-block budget (PROTOCOL.md §4).
//

import Foundation

final class TelemetryStore {

    /// Latest value per `group.name`, from whatever block carried it.
    private(set) var latest: [String: Double] = [:]

    /// Recent samples for statistics windows (most recent last).
    private(set) var history: [LogSample] = []
    private let historyLimit = 600

    /// Subscribers (charts, auto-tuner, status labels). Called on the main queue.
    var listeners: [(LogSample) -> Void] = []

    /// Block ids. The firmware allows 3 concurrent blocks.
    enum BlockId: UInt8 {
        case status = 1        // flight status + battery
        case tune = 2          // per-axis PID telemetry
        case nav = 5           // dead-reckoned position / target
    }

    /// The status block (15 bytes of values — within the 25-byte budget).
    static let statusVariables: [LogVariable] = [
        LogVariable(tocId: 0, logType: .float, name: "pm.vbat"),
        LogVariable(tocId: 11, logType: .uint8, name: "flight.mode"),
        LogVariable(tocId: 13, logType: .uint8, name: "flight.armed"),
        LogVariable(tocId: 12, logType: .uint8, name: "flight.auto_state"),
        LogVariable(tocId: 1, logType: .float, name: "stabilizer.roll"),
        LogVariable(tocId: 2, logType: .float, name: "stabilizer.pitch"),
    ]

    func ingest(_ sample: LogSample) {
        for (index, name) in sample.names.enumerated() where sample.values.indices.contains(index) {
            latest[name] = sample.values[index].asDouble
        }
        history.append(sample)
        if history.count > historyLimit {
            history.removeFirst(history.count - historyLimit)
        }
        DispatchQueue.main.async {
            for listener in self.listeners {
                listener(sample)
            }
        }
    }

    func value(_ name: String) -> Double? {
        latest[name]
    }
}

/// Owns the CrazyFlie + CrtpClient pair for one connection. The screens all
/// talk to this, and a transport change rebuilds it.
final class DroneSession {
    private(set) var crazyFlie: CrazyFlie
    let telemetry = TelemetryStore()

    /// Console text from the drone (CRTP port 0).
    var onConsole: ((String) -> Void)?

    init(crazyFlie: CrazyFlie) {
        self.crazyFlie = crazyFlie
        wire(crazyFlie.crtp)
    }

    /// Rebuilds the session with the transport currently in LinkSettings.
    func reconnect(delegate: CrazyFlieDelegate) {
        crazyFlie.disconnect()
        let fresh = CrazyFlie(crtpDriver: LinkSettings.makeDriver(), delegate: delegate)
        crazyFlie = fresh
        wire(fresh.crtp)
        fresh.connect(nil)
    }

    private func wire(_ client: CrtpClient) {
        client.logg.onSample = { [weak self] sample in
            self?.telemetry.ingest(sample)
        }
        client.onConsole = { [weak self] text in
            self?.onConsole?(text)
        }
    }
}
