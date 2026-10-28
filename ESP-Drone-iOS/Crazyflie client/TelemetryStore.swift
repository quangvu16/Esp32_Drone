//
//  TelemetryStore.swift
//  Crazyflie client
//
//  Fan-out for decoded log samples plus a short history for charts and
//  statistics. Log block ids are allocated here so the screens cannot fight
//  over the firmware's 3-block budget (PROTOCOL.md §4).
//
//  Phân phối các mẫu log đã giải mã, kèm một lịch sử ngắn cho biểu đồ và
//  thống kê. Các id khối log được cấp phát tại đây để các màn hình không
//  giành nhau trong ngân sách 3 khối của firmware (PROTOCOL.md §4).
//

import Foundation

final class TelemetryStore {

    /// Latest value per `group.name`, from whatever block carried it.
    /// Giá trị mới nhất theo `group.name`, lấy từ bất kỳ khối nào mang nó.
    private(set) var latest: [String: Double] = [:]

    /// Recent samples for statistics windows (most recent last).
    /// Các mẫu gần đây cho cửa sổ thống kê (mẫu mới nhất ở cuối).
    private(set) var history: [LogSample] = []
    private let historyLimit = 600

    /// Subscribers (charts, auto-tuner, status labels). Called on the main queue.
    /// Các listener (biểu đồ, auto-tuner, nhãn trạng thái). Được gọi trên hàng đợi chính.
    var listeners: [(LogSample) -> Void] = []

    /// Block ids. The firmware allows 3 concurrent blocks.
    /// Id khối. Firmware cho phép 3 khối đồng thời.
    enum BlockId: UInt8 {
        case status = 1        // flight status + battery
                               // trạng thái bay + pin
        case tune = 2          // per-axis PID telemetry
                               // dữ liệu từ xa PID theo từng trục
        case nav = 5           // dead-reckoned position / target
                               // vị trí tự dẫn đường quán tính / mục tiêu
    }

    /// The status block (15 bytes of values — within the 25-byte budget).
    /// Khối trạng thái (15 byte giá trị — nằm trong ngân sách 25 byte).
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
/// Sở hữu cặp CrazyFlie + CrtpClient cho một kết nối. Mọi màn hình đều
/// làm việc với đối tượng này, và thay đổi tầng truyền sẽ dựng lại nó.
final class DroneSession {
    private(set) var crazyFlie: CrazyFlie
    let telemetry = TelemetryStore()

    /// Console text from the drone (CRTP port 0).
    /// Chữ console từ drone (CRTP port 0).
    var onConsole: ((String) -> Void)?

    init(crazyFlie: CrazyFlie) {
        self.crazyFlie = crazyFlie
        wire(crazyFlie.crtp)
    }

    /// Rebuilds the session with the transport currently in LinkSettings.
    /// Dựng lại phiên kết nối với tầng truyền hiện có trong LinkSettings.
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
