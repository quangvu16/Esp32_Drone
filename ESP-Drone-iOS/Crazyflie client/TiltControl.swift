//
//  TiltControl.swift
//  Crazyflie client
//
//  Phone IMU tilt-to-fly (PROTOCOL.md §9). CoreMotion CMDeviceMotion attitude
//  maps onto the existing CommanderPacket — this is not SensorServer.
//
//  Bay bằng nghiêng điện thoại qua IMU (PROTOCOL.md §9). Tư thế từ
//  CMDeviceMotion của CoreMotion được ánh xạ lên CommanderPacket hiện có —
//  đây không phải SensorServer.
//
//  Landscape, phone flat = level drone. A deadzone near flat keeps the drone
//  still, and the tilt scale saturates at `maxTiltDegrees` so the firmware's
//  own ±25° clamp is never the first limit hit. Thrust stays on the existing
//  stick/slider — tilt only carries attitude.
//
//  Nằm ngang, điện thoại phẳng = drone thăng bằng. Vùng chết quanh vị trí
//  phẳng giữ drone đứng yên, và thang nghiêng bão hòa tại `maxTiltDegrees`
//  để mức kẹp ±25° của chính firmware không bao giờ là giới hạn chạm trước.
//  Lực đẩy vẫn do cần/thanh trượt hiện có đảm nhận — nghiêng chỉ mang tư thế.
//
//  Pitch sign: the commander packet's pitch is the negative of the UI pitch
//  (see SimpleCrazyFlieCommander.pitch, which multiplies by -1). This class
//  applies the same sign so "tilt away → pitch forward" works unchanged.
//
//  Dấu pitch: pitch trong gói tin commander là âm của pitch trên UI (xem
//  SimpleCrazyFlieCommander.pitch, nơi nhân với -1). Lớp này áp dụng cùng
//  quy tắc dấu để "nghiêng ra xa → pitch tiến tới" hoạt động y nguyên.
//

import CoreMotion
import Foundation

final class TiltControl: CrazyFlieCommander {

    /// Attitude is read at 20 Hz to match the commander timer.
    /// Tư thế được đọc ở 20 Hz để khớp với timer của commander.
    private let motion = CMMotionManager()
    private let queue = OperationQueue()

    /// Set when tilt is actively feeding the commander.
    /// Được đặt khi chế độ nghiêng đang thực sự cấp dữ liệu cho commander.
    private(set) var isActive = false

    /// Near-flat deadzone in degrees. Inside this the drone is commanded level.
    /// Vùng chết quanh vị trí phẳng, đơn vị độ. Trong vùng này drone được lệnh giữ mức phẳng.
    var deadzoneDegrees: Double = 3.0

    /// Tilt at or beyond this maps to the full ±`maxOutputDegrees` command.
    /// Nghiêng tới hoặc vượt giá trị này sẽ ánh xạ tới toàn bộ lệnh ±`maxOutputDegrees`.
    var maxTiltDegrees: Double = 30.0

    /// Command range sent to the drone (firmware clamps to ±25°).
    /// Phạm vi lệnh gửi tới drone (firmware kẹp trong khoảng ±25°).
    var maxOutputDegrees: Float = 20.0

    /// Yaw rate at full stick, deg/s (firmware clamps to ±200°/s).
    /// Tốc độ yaw khi gạt hết cần, độ/s (firmware kẹp trong khoảng ±200°/s).
    var maxYawRate: Float = 100.0

    /// Thrust is owned by the existing stick/slider; this is its passthrough.
    /// Lực đẩy do cần/thanh trượt hiện có nắm giữ; thuộc tính này chỉ truyền lực đẩy đi xuyên qua.
    var thrustProvider: () -> Float = { 0 }

    // Latest attitude, exposed for the "tilt control active" indicator.
    // Tư thế mới nhất, được công khai cho chỉ báo "tilt control active".
    private(set) var rollDegrees: Double = 0
    private(set) var pitchDegrees: Double = 0
    private(set) var yawDegreesPerSecond: Double = 0

    private var lastYaw: Double?
    private var lastYawTime: Date?

    // MARK: - Lifecycle

    func start() {
        guard motion.isDeviceMotionAvailable, !motion.isDeviceMotionActive else { return }
        motion.deviceMotionUpdateInterval = 1.0 / 20.0
        // xArbitraryZVertical: flat landscape is our zero. We do not need
        // north — yaw is only used as a rate.
        // xArbitraryZVertical: thế nằm ngang là mốc không của ta. Ta không
        // cần hướng bắc — yaw chỉ được dùng như một tốc độ.
        motion.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: queue) { [weak self] motion, _ in
            guard let self = self, let attitude = motion?.attitude else { return }
            self.consume(attitude)
        }
        isActive = true
    }

    func stop() {
        if motion.isDeviceMotionActive {
            motion.stopDeviceMotionUpdates()
        }
        isActive = false
        rollDegrees = 0
        pitchDegrees = 0
        yawDegreesPerSecond = 0
        lastYaw = nil
        lastYawTime = nil
    }

    private func consume(_ attitude: CMAttitude) {
        // CMAttitude gives radians. With xArbitraryZVertical, roll/pitch are
        // already in a frame where flat landscape is zero.
        // CMAttitude trả về radian. Với xArbitraryZVertical, roll/pitch đã ở
        // trong khung mà thế nằm ngang là mốc không.
        rollDegrees = applyDeadzone(attitude.roll * 180 / .pi)
        pitchDegrees = applyDeadzone(attitude.pitch * 180 / .pi)

        // Yaw rate from a finite difference — attitude.yaw drifts, a rate does not.
        // Tốc độ yaw lấy từ sai phân hữu hạn — attitude.yaw bị trôi, còn tốc độ thì không.
        let now = Date()
        if let prev = lastYaw, let t = lastYawTime {
            let dt = now.timeIntervalSince(t)
            if dt > 1e-3 {
                var delta = (attitude.yaw - prev) * 180 / .pi
                if delta > 180 { delta -= 360 }
                if delta < -180 { delta += 360 }
                yawDegreesPerSecond = max(-Double(maxYawRate), min(Double(maxYawRate), delta / dt))
            }
        }
        lastYaw = attitude.yaw
        lastYawTime = now
    }

    private func applyDeadzone(_ degrees: Double) -> Double {
        guard abs(degrees) > deadzoneDegrees else { return 0 }
        let sign: Double = degrees >= 0 ? 1 : -1
        // Re-zero past the deadzone so the command is continuous at the edge.
        // Đặt lại mốc không sau khi trừ vùng chết để lệnh liên tục tại mép vùng.
        return sign * (abs(degrees) - deadzoneDegrees)
    }

    private func scaled(_ degrees: Double) -> Float {
        let limited = max(-maxTiltDegrees, min(maxTiltDegrees, degrees))
        return Float(limited / maxTiltDegrees) * maxOutputDegrees
    }

    // MARK: - CrazyFlieCommander
    //
    // The existing stick commander negates UI pitch before packing the packet
    // (SimpleCrazyFlieCommander.pitch). We do the same here so both control
    // modes share one wire convention.
    // Commander cần gạt hiện có đảo dấu pitch trên UI trước khi đóng gói tin
    // (SimpleCrazyFlieCommander.pitch). Ta làm tương tự ở đây để cả hai chế
    // độ điều khiển dùng chung một quy ước trên đường truyền.

    var roll: Float { scaled(rollDegrees) }
    var pitch: Float { -scaled(pitchDegrees) }
    var yaw: Float {
        Float(max(-Double(maxYawRate), min(Double(maxYawRate), yawDegreesPerSecond)))
    }
    var thrust: Float { thrustProvider() }

    func prepareData() {
        // Nothing buffered — attitude is updated by the motion queue.
        // Không có gì trong bộ đệm — tư thế do hàng đợi chuyển động cập nhật.
    }
}
