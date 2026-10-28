//
//  GpsNavCore.swift
//  Crazyflie client
//
//  Phone-side GPS mission navigation (PROTOCOL.md §10). The phone runs the
//  mission; the drone chases one offset at a time. This is pure Foundation so
//  it is unit-testable in isolation.
//  Dẫn đường sứ mệnh GPS phía điện thoại (PROTOCOL.md §10). Điện thoại điều
//  khiển sứ mệnh; drone bám theo từng offset một. Đây là code thuần Foundation
//  nên có thể unit-test độc lập.
//
//  Ported from OpenBot's GpsNavCore (GeoMath, Waypoint, Mission, Navigator) —
//  NOT GuidanceController / guidance.py. The drone has its own tilt controller
//  (`navSetpoints`, §6) and only needs to be told where to go.
//  Chuyển từ GpsNavCore của OpenBot (GeoMath, Waypoint, Mission, Navigator) —
//  KHÔNG phải GuidanceController / guidance.py. Drone có bộ điều khiển nghiêng
//  riêng (`navSetpoints`, §6) và chỉ cần được bảo đi đâu.
//
//  Launch frame (PROTOCOL.md §6, pinned): +x = nose at ALIGN (bearing alpha),
//  +y = 90° LEFT (alpha − 90°). The (n,e)→(x,y) mapping is
//  Khung cất cánh (PROTOCOL.md §6, cố định): +x = mũi tại ALIGN (phương hướng
//  alpha), +y = BÊN TRÁI 90° (alpha − 90°). Phép ánh xạ (n,e)→(x,y) là
//
//      x = n·cos(alpha) + e·sin(alpha)
//      y = n·sin(alpha) − e·cos(alpha)
//
//  and is proven by Esp32_drone/test/. NAV_FWD_SIGN / NAV_LEFT_SIGN only scale
//  the dead reckoning (`nav.pos_*`), never the target rotation — expect one to
//  need flipping after the first flight.
//  và được chứng minh bởi Esp32_drone/test/. NAV_FWD_SIGN / NAV_LEFT_SIGN chỉ
//  dùng để tỷ lệ hóa tự dẫn đường quán tính (`nav.pos_*`), không bao giờ cho
//  phép xoay mục tiêu — hãy dự kiến một trong hai sẽ phải đảo dấu sau chuyến
//  bay đầu tiên.
//

import Foundation

// MARK: - Geo math

/// Geographic helpers. All angles in degrees, all distances in metres.
/// Các hàm địa lý. Mọi góc tính bằng độ, mọi khoảng cách tính bằng mét.
enum GeoMath {
    /// Mean earth radius used by the haversine formula (PROTOCOL.md §9).
    /// Bán kính trung bình của Trái Đất dùng trong công thức haversine (PROTOCOL.md §9).
    static let earthRadiusMeters = 6371008.8

    private static let degToRad = Double.pi / 180.0
    private static let radToDeg = 180.0 / Double.pi

    /// Great-circle distance between two WGS-84 points, in metres.
    /// Khoảng cách đường tròn lớn giữa hai điểm WGS-84, tính bằng mét.
    static func distanceMeters(_ lat1: Double, _ lon1: Double, _ lat2: Double, _ lon2: Double) -> Double {
        let dLat = (lat2 - lat1) * degToRad
        let dLon = (lon2 - lon1) * degToRad
        let a = sin(dLat / 2) * sin(dLat / 2)
            + cos(lat1 * degToRad) * cos(lat2 * degToRad) * sin(dLon / 2) * sin(dLon / 2)
        return 2 * earthRadiusMeters * asin(min(1.0, sqrt(a)))
    }

    /// Initial great-circle bearing from the first point to the second.
    /// Result is degrees clockwise from true north, wrapped to `[-180, 180)`.
    /// Phương hướng đường tròn lớn ban đầu từ điểm thứ nhất đến điểm thứ hai.
    /// Kết quả là độ tính theo chiều kim đồng hồ từ chính bắc, gói về `[-180, 180)`.
    static func bearingDegrees(_ lat1: Double, _ lon1: Double, _ lat2: Double, _ lon2: Double) -> Double {
        let phi1 = lat1 * degToRad
        let phi2 = lat2 * degToRad
        let dLon = (lon2 - lon1) * degToRad
        let y = sin(dLon) * cos(phi2)
        let x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLon)
        return wrapDegrees(atan2(y, x) * radToDeg)
    }

    /// Wraps an angle to the range `[-180, 180)`.
    /// Gói một góc về miền `[-180, 180)`.
    static func wrapDegrees(_ degrees: Double) -> Double {
        let shifted = (degrees + 180.0).truncatingRemainder(dividingBy: 360.0)
        return (shifted + 360.0).truncatingRemainder(dividingBy: 360.0) - 180.0
    }

    /// Smallest signed turn from a heading onto a target bearing.
    /// Positive means the target is to the right (clockwise).
    /// Lượt rẽ có dấu nhỏ nhất từ hướng mũi sang phương hướng mục tiêu.
    /// Dương nghĩa là mục tiêu ở bên phải (chiều kim đồng hồ).
    static func turnErrorDegrees(_ headingDegrees: Double, _ targetBearingDegrees: Double) -> Double {
        wrapDegrees(targetBearingDegrees - headingDegrees)
    }

    /// Moves a point along a great circle by the given distance and initial bearing.
    /// Dịch một điểm theo đường tròn lớn với khoảng cách và phương hướng ban đầu cho trước.
    static func destinationPoint(_ latitude: Double, _ longitude: Double,
                                 _ bearingDegrees: Double, _ distanceMeters: Double)
        -> (latitude: Double, longitude: Double) {
        let angular = distanceMeters / earthRadiusMeters
        let theta = bearingDegrees * degToRad
        let phi1 = latitude * degToRad
        let lambda1 = longitude * degToRad

        let sinPhi2 = sin(phi1) * cos(angular) + cos(phi1) * sin(angular) * cos(theta)
        let phi2 = asin(max(-1.0, min(1.0, sinPhi2)))
        let lambda2 = lambda1 + atan2(
            sin(theta) * sin(angular) * cos(phi1),
            cos(angular) - sin(phi1) * sinPhi2
        )
        return (phi2 * radToDeg, wrapDegrees(lambda2 * radToDeg))
    }

    /// North/east metres from the launch point to a lat/lon (PROTOCOL.md §9).
    /// Số mét bắc/đông từ điểm cất cánh đến một vĩ/kinh độ (PROTOCOL.md §9).
    static func northEastOffset(fromLat: Double, fromLon: Double,
                                toLat: Double, toLon: Double)
        -> (north: Double, east: Double) {
        let d = distanceMeters(fromLat, fromLon, toLat, toLon)
        let b = bearingDegrees(fromLat, fromLon, toLat, toLon) * degToRad
        return (north: d * cos(b), east: d * sin(b))
    }
}

// MARK: - Launch frame

/// The pinned body/launch frame from PROTOCOL.md §6.
///
/// `+x` is where the nose pointed at ALIGN (compass bearing `alpha`);
/// `+y` is 90° LEFT of that. The drone reports `nav.pos_x/pos_y` and
/// `tgt_x/tgt_y` in this one frame, so waypoints are compared against the
/// drone's reported position directly — metres, no lat/lon round trip.
/// Khung thân/khung cất cánh cố định từ PROTOCOL.md §6.
///
/// `+x` là hướng mũi tại ALIGN (phương hướng la bàn `alpha`);
/// `+y` là BÊN TRÁI 90° so với hướng đó. Drone báo `nav.pos_x/pos_y` và
/// `tgt_x/tgt_y` trong cùng một khung này, nên các điểm hành trình được so với
/// vị trí drone tự báo trực tiếp — bằng mét, không vòng vèo qua vĩ/kinh độ.
enum LaunchFrame {

    /// (north, east) metres → launch-frame (x, y) metres.
    ///
    ///     x = n·cos(alpha) + e·sin(alpha)
    ///     y = n·sin(alpha) − e·cos(alpha)
    /// (bắc, đông) mét → (x, y) mét theo khung cất cánh.
    static func toLaunchFrame(north: Double, east: Double, alphaDegrees: Double)
        -> (x: Double, y: Double) {
        let a = alphaDegrees * Double.pi / 180.0
        let ca = cos(a)
        let sa = sin(a)
        return (x: north * ca + east * sa,
                y: north * sa - east * ca)
    }

    /// Launch-frame (x, y) metres → (north, east) metres. Inverse of the above.
    /// (x, y) mét theo khung cất cánh → (bắc, đông) mét. Nghịch đảo của phép trên.
    static func toNorthEast(x: Double, y: Double, alphaDegrees: Double)
        -> (north: Double, east: Double) {
        let a = alphaDegrees * Double.pi / 180.0
        let ca = cos(a)
        let sa = sin(a)
        return (north: x * ca + y * sa,
                east: x * sa - y * ca)
    }
}

// MARK: - Waypoint

/// A single GPS waypoint. Only the horizontal target plus an acceptance
/// radius — no altitude leg.
/// Một điểm hành trình GPS. Chỉ có mục tiêu ngang cộng một bán kính chấp nhận
/// — không có chặng độ cao.
struct Waypoint: Equatable {
    let latitude: Double
    let longitude: Double
    var name: String
    /// Radius in metres at which this waypoint counts as reached. Floor 0.5 m.
    /// Bán kính (mét) mà trong đó điểm hành trình được coi là đã tới. Sàn 0,5 m.
    var acceptanceRadiusMeters: Double

    /// PROTOCOL.md §10 pins 2.5 m as the default.
    /// PROTOCOL.md §10 cố định 2,5 m làm mặc định.
    static let defaultAcceptanceRadiusMeters = 2.5

    init(latitude: Double, longitude: Double, name: String = "",
         acceptanceRadiusMeters: Double = Waypoint.defaultAcceptanceRadiusMeters) {
        self.latitude = latitude
        self.longitude = longitude
        self.name = name
        self.acceptanceRadiusMeters = max(0.5, acceptanceRadiusMeters)
    }

    func distanceFrom(_ latitude: Double, _ longitude: Double) -> Double {
        GeoMath.distanceMeters(latitude, longitude, self.latitude, self.longitude)
    }

    func bearingFrom(_ latitude: Double, _ longitude: Double) -> Double {
        GeoMath.bearingDegrees(latitude, longitude, self.latitude, self.longitude)
    }

    /// Launch-frame offset of this waypoint from the given lat/lon.
    /// Offset theo khung cất cánh của điểm hành trình này từ vĩ/kinh độ cho trước.
    func launchOffset(fromLat: Double, fromLon: Double, alphaDegrees: Double)
        -> (x: Double, y: Double) {
        let ne = GeoMath.northEastOffset(fromLat: fromLat, fromLon: fromLon,
                                         toLat: latitude, toLon: longitude)
        return LaunchFrame.toLaunchFrame(north: ne.north, east: ne.east, alphaDegrees: alphaDegrees)
    }
}

// MARK: - Mission

enum MissionError: Error, Equatable {
    case emptyFile
    case unsupportedHeader(String)
    case malformedRow(String)
}

/// An ordered GPS mission: a home position plus waypoints to visit.
/// Serialises to the Mission Planner `.waypoints` format (`QGC WPL 110`).
/// Sứ mệnh GPS có thứ tự: một vị trí home cộng các điểm hành trình cần ghé.
/// Tuần tự hóa theo định dạng `.waypoints` của Mission Planner (`QGC WPL 110`).
class Mission {
    static let wplHeader = "QGC WPL 110"
    static let cmdNavWaypoint = 16

    private(set) var waypoints: [Waypoint] = []
    var homeLatitude: Double?
    var homeLongitude: Double?

    var isEmpty: Bool { waypoints.isEmpty }
    var count: Int { waypoints.count }
    var hasHome: Bool { homeLatitude != nil && homeLongitude != nil }

    // MARK: bookkeeping

    func clearWaypoints() {
        waypoints.removeAll()
    }

    func addWaypoint(_ waypoint: Waypoint, at index: Int? = nil) {
        if let index = index {
            waypoints.insert(waypoint, at: max(0, min(index, waypoints.count)))
        } else {
            waypoints.append(waypoint)
        }
    }

    func removeWaypoint(at index: Int) {
        guard waypoints.indices.contains(index) else { return }
        waypoints.remove(at: index)
    }

    func moveWaypoint(from fromIndex: Int, to toIndex: Int) {
        guard waypoints.indices.contains(fromIndex), waypoints.indices.contains(toIndex) else { return }
        waypoints.insert(waypoints.remove(at: fromIndex), at: toIndex)
    }

    func setWaypoints(_ replacement: [Waypoint]) {
        waypoints = replacement
    }

    func waypoint(at index: Int) -> Waypoint? {
        waypoints.indices.contains(index) ? waypoints[index] : nil
    }

    func setHome(_ latitude: Double, _ longitude: Double) {
        homeLatitude = latitude
        homeLongitude = longitude
    }

    /// Total along-track length in metres, home to last waypoint.
    /// Tổng chiều dài dọc theo đường bay, tính bằng mét, từ home đến điểm cuối.
    func totalDistanceMeters() -> Double {
        guard !waypoints.isEmpty else { return 0.0 }

        var total = 0.0
        var previous: (latitude: Double, longitude: Double)?
        if let homeLat = homeLatitude, let homeLon = homeLongitude {
            previous = (homeLat, homeLon)
        }

        for waypoint in waypoints {
            if let prev = previous {
                total += GeoMath.distanceMeters(
                    prev.latitude, prev.longitude,
                    waypoint.latitude, waypoint.longitude
                )
            }
            previous = (waypoint.latitude, waypoint.longitude)
        }
        return total
    }

    // MARK: Mission Planner .waypoints codec

    /// Serialises to the Mission Planner / QGC waypoint format.
    /// Index 0 is the home position (`current` flag set). When no home has been
    /// recorded yet the first waypoint is used so the file stays valid.
    /// Tuần tự hóa theo định dạng điểm hành trình của Mission Planner / QGC.
    /// Chỉ số 0 là vị trí home (có cờ `current`). Khi chưa ghi nhận home nào,
    /// điểm hành trình đầu tiên được dùng để file vẫn hợp lệ.
    func toWaypointsFile() -> String {
        var lines = [Mission.wplHeader]
        if hasHome || !waypoints.isEmpty {
            let lat: Double
            let lon: Double
            if let homeLat = homeLatitude, let homeLon = homeLongitude {
                lat = homeLat
                lon = homeLon
            } else {
                lat = waypoints[0].latitude
                lon = waypoints[0].longitude
            }
            lines.append(Mission.formatWplLine(index: 0, current: 1, latitude: lat, longitude: lon))
        }
        for (offset, waypoint) in waypoints.enumerated() {
            lines.append(Mission.formatWplLine(
                index: offset + 1, current: 0,
                latitude: waypoint.latitude, longitude: waypoint.longitude
            ))
        }
        return lines.joined(separator: "\n") + "\n"
    }

    /// Parses a Mission Planner / QGroundControl waypoint file.
    /// Row 0 is the home position; subsequent `NAV_WAYPOINT` (cmd 16) rows are
    /// waypoints. Non-navigation commands are skipped so files planned for a
    /// rover with extra `DO_` commands still load their waypoints.
    /// Phân tích một file điểm hành trình của Mission Planner / QGroundControl.
    /// Dòng 0 là vị trí home; các dòng `NAV_WAYPOINT` (cmd 16) theo sau là các
    /// điểm hành trình. Các lệnh không phải điều hướng bị bỏ qua nên file lập
    /// kế hoạch cho xe mặt đất với các lệnh `DO_` phụ vẫn nạp được điểm hành trình.
    static func parseWaypointsFile(_ contents: String) throws -> Mission {
        let mission = Mission()
        let lines = contents.split(separator: "\n", omittingEmptySubsequences: false).map(String.init)
        guard let first = lines.first, !first.trimmingCharacters(in: .whitespaces).isEmpty else {
            throw MissionError.emptyFile
        }

        let header = first.trimmingCharacters(in: .whitespaces)
        guard header.hasPrefix("QGC WPL") else {
            throw MissionError.unsupportedHeader(header)
        }

        for (rowIndex, rawLine) in lines.dropFirst().enumerated() {
            let line = rawLine.trimmingCharacters(in: .whitespaces)
            if line.isEmpty { continue }

            var fields = line.components(separatedBy: "\t")
            if fields.count < 11 {
                fields = line.components(separatedBy: .whitespaces).filter { !$0.isEmpty }
            }
            guard fields.count >= 11,
                  let command = Int(fields[3]),
                  let latitude = Double(fields[8]),
                  let longitude = Double(fields[9])
            else {
                throw MissionError.malformedRow(line)
            }

            if rowIndex == 0 {
                mission.setHome(latitude, longitude)
            } else if command == Mission.cmdNavWaypoint {
                mission.addWaypoint(Waypoint(latitude: latitude, longitude: longitude))
            }
        }
        return mission
    }

    private static func formatWplLine(index: Int, current: Int, latitude: Double, longitude: Double) -> String {
        // index, current, frame, command, p1, p2, p3, p4, x, y, z, autocontinue
        String(
            format: "%d\t%d\t3\t%d\t0\t0\t0\t0\t%.8f\t%.8f\t0\t1",
            index, current, Mission.cmdNavWaypoint, latitude, longitude
        )
    }
}

// MARK: - Navigator

enum NavMode: String {
    case idle
    case mission
    case guided
    case rtl
}

/// Outcome of one navigation tick.
///
/// `target` is the launch-frame offset the drone should chase (what goes in
/// `TARGET`). `droneX/Y` is the drone's own dead-reckoned `nav.pos_*`, used
/// for arrival — never the phone's GPS.
/// Kết quả của một nhịp điều hướng.
///
/// `target` là offset theo khung cất cánh mà drone cần bám (thứ được đưa vào
/// `TARGET`). `droneX/Y` là `nav.pos_*` tự dẫn đường quán tính của chính drone,
/// dùng để xét đã tới — không bao giờ là GPS của điện thoại.
struct NavStatus {
    let mode: NavMode
    /// Launch-frame target offset (metres). `nil` when idle/paused/holding.
    /// Offset mục tiêu theo khung cất cánh (mét). `nil` khi nghỉ/tạm dừng/giữ.
    let target: (x: Double, y: Double)?
    /// The drone's reported position in the launch frame.
    /// Vị trí drone tự báo trong khung cất cánh.
    let droneX: Double?
    let droneY: Double?
    let distanceMeters: Double?
    let bearingDegrees: Double?
    let activeWaypointIndex: Int?
    let arrived: Bool
    let message: String
}

/// Runs waypoint missions against the drone's reported position.
///
/// The phone is the pilot's handset; if the pilot walks away, a phone-position
/// arrival check would fire while the drone is nowhere near the waypoint. So
/// arrival and progress use `nav.pos_x/pos_y`, and the phone's GPS only defines
/// the plan and the launch frame. FOLLOW is the one asymmetry: there the
/// target *is* the phone, so the phone's GPS is the source of the target.
/// Chạy sứ mệnh điểm hành trình dựa trên vị trí drone tự báo.
///
/// Điện thoại là thiết bị cầm tay của người lái; nếu người lái đi nơi khác,
/// phép xét đã-tới theo vị trí điện thoại sẽ kích hoạt trong khi drone còn ở
/// rất xa điểm hành trình. Vì vậy việc xét đến nơi và tiến độ dùng
/// `nav.pos_x/pos_y`, còn GPS của điện thoại chỉ định nghĩa kế hoạch và khung
/// cất cánh. FOLLOW là ngoại lệ duy nhất: ở đó mục tiêu *chính là* điện thoại,
/// nên GPS của điện thoại là nguồn của mục tiêu.
class Navigator {

    /// Hold rather than navigate on a fix worse than this (PROTOCOL.md §10).
    /// Giữ nguyên thay vì dẫn đường khi fix tệ hơn mức này (PROTOCOL.md §10).
    static let maxPositionAccuracyMeters = 15.0

    var mission: Mission
    private(set) var mode: NavMode = .idle
    private(set) var isPaused = false
    private(set) var activeIndex = 0
    private(set) var guidedTarget: Waypoint?
    private(set) var lastStatus: NavStatus?

    /// Compass bearing of the nose at ALIGN, degrees. Drives the launch frame.
    /// Phương hướng la bàn của mũi tại ALIGN, tính bằng độ. Quyết định khung cất cánh.
    private(set) var alphaDegrees: Double = 0
    /// Lat/lon the frame is anchored to (the launch point).
    /// Vĩ/kinh độ mà khung neo vào (điểm cất cánh).
    private(set) var launchLatitude: Double = 0
    private(set) var launchLongitude: Double = 0

    init(mission: Mission = Mission()) {
        self.mission = mission
    }

    // MARK: mission bookkeeping

    var isNavigating: Bool { mode != .idle }

    func markHome(_ latitude: Double, _ longitude: Double) {
        mission.setHome(latitude, longitude)
    }

    /// Records the launch point and the ALIGN bearing. Resets the frame.
    /// Ghi nhận điểm cất cánh và phương hướng lúc ALIGN. Đặt lại khung.
    func align(launchLatitude: Double, launchLongitude: Double, alphaDegrees: Double) {
        self.launchLatitude = launchLatitude
        self.launchLongitude = launchLongitude
        self.alphaDegrees = GeoMath.wrapDegrees(alphaDegrees)
    }

    func loadMission(_ mission: Mission) {
        self.mission = mission
        activeIndex = 0
    }

    func loadMissionFile(_ contents: String) throws -> Mission {
        let loaded = try Mission.parseWaypointsFile(contents)
        loadMission(loaded)
        return loaded
    }

    func saveMissionFile() -> String {
        mission.toWaypointsFile()
    }

    /// The waypoint being sought right now, or `nil` when idle.
    /// Điểm hành trình đang tìm lúc này, hoặc `nil` khi nghỉ.
    func currentTarget() -> Waypoint? {
        if mode == .mission {
            return mission.waypoint(at: activeIndex)
        }
        return guidedTarget
    }

    // MARK: mode control

    /// MISSION: visit every waypoint in order. False if there are none.
    /// MISSION: ghé mọi điểm hành trình theo thứ tự. False nếu không có điểm nào.
    @discardableResult
    func startMission() -> Bool {
        guard !mission.isEmpty else { return false }
        activeIndex = 0
        guidedTarget = nil
        mode = .mission
        isPaused = false
        return true
    }

    /// GUIDED: one tapped "go to here" point. Arrival disarms it.
    /// GUIDED: một điểm "đi tới đây" do người dùng chạm chọn. Đến nơi là tự ngắt.
    func startGuided(latitude: Double, longitude: Double,
                     acceptanceRadiusMeters: Double = Waypoint.defaultAcceptanceRadiusMeters) {
        guidedTarget = Waypoint(
            latitude: latitude, longitude: longitude,
            name: "", acceptanceRadiusMeters: acceptanceRadiusMeters
        )
        mode = .guided
        isPaused = false
    }

    /// RTL: return to the recorded home/launch position. One-shot.
    /// RTL: trở về vị trí home/cất cánh đã ghi nhận. Chỉ có hiệu lực một lần.
    @discardableResult
    func startRTL(acceptanceRadiusMeters: Double = Waypoint.defaultAcceptanceRadiusMeters) -> Bool {
        guard let homeLat = mission.homeLatitude, let homeLon = mission.homeLongitude else {
            return false
        }
        guidedTarget = Waypoint(
            latitude: homeLat, longitude: homeLon,
            name: "Home", acceptanceRadiusMeters: acceptanceRadiusMeters
        )
        mode = .rtl
        isPaused = false
        return true
    }

    /// Pause: stop sending a new target. The drone holds after 3 s (§6).
    /// Tạm dừng: ngưng gửi mục tiêu mới. Drone giữ nguyên sau 3 giây (§6).
    func pause() {
        if isNavigating { isPaused = true }
    }

    func resume() {
        isPaused = false
    }

    /// Disarms navigation and returns to manual/idle control.
    /// Ngắt điều hướng và trở về điều khiển thủ công/nghỉ.
    func stop() {
        mode = .idle
        isPaused = false
        guidedTarget = nil
        activeIndex = 0
    }

    // MARK: stepping

    /// One navigation tick.
    /// Một nhịp điều hướng.
    ///
    /// - Parameters:
    ///   - droneX/droneY: the drone's `nav.pos_x`/`nav.pos_y` in the launch
    ///     frame. This is the source of truth for arrival.
    ///   - phoneLatitude/phoneLongitude: used only to compute the target
    ///     offset (FOLLOW/MISSION/RTL all resolve to a lat/lon first).
    ///   - accuracyMeters: phone GPS horizontal accuracy. A fix worse than
    ///     `maxPositionAccuracyMeters` holds the drone with a message.
    ///   - droneX/droneY: `nav.pos_x`/`nav.pos_y` của drone trong khung cất
    ///     cánh. Đây là nguồn sự thật để xét đã tới.
    ///   - phoneLatitude/phoneLongitude: chỉ dùng để tính offset mục tiêu
    ///     (FOLLOW/MISSION/RTL đều quy về một vĩ/kinh độ trước).
    ///   - accuracyMeters: độ chính xác ngang của GPS điện thoại. Một fix tệ
    ///     hơn `maxPositionAccuracyMeters` sẽ giữ drone kèm thông báo.
    func step(droneX: Double, droneY: Double,
              phoneLatitude: Double, phoneLongitude: Double,
              accuracyMeters: Double? = nil) -> NavStatus {

        if !isNavigating || isPaused {
            return record(NavStatus(
                mode: mode, target: nil,
                droneX: droneX, droneY: droneY,
                distanceMeters: nil, bearingDegrees: nil,
                activeWaypointIndex: nil, arrived: false,
                message: isPaused ? "paused" : "idle"
            ))
        }

        if let accuracy = accuracyMeters, accuracy > Navigator.maxPositionAccuracyMeters {
            // Navigating on a bad fix is worse than waiting: hold and say so.
            // Dẫn đường trên fix tệ còn tệ hơn là chờ: giữ nguyên và nói rõ.
            return record(NavStatus(
                mode: mode, target: nil,
                droneX: droneX, droneY: droneY,
                distanceMeters: nil, bearingDegrees: nil,
                activeWaypointIndex: activeIndex, arrived: false,
                message: String(format: "GPS accuracy %.1f m, holding", accuracy)
            ))
        }

        guard let target = currentTarget() else {
            stop()
            return record(NavStatus(
                mode: .idle, target: nil,
                droneX: droneX, droneY: droneY,
                distanceMeters: nil, bearingDegrees: nil,
                activeWaypointIndex: nil, arrived: false,
                message: "no target"
            ))
        }

        // Waypoint offset in the launch frame — this is what TARGET carries.
        // Offset của điểm hành trình trong khung cất cánh — đây là thứ TARGET mang.
        let offset = target.launchOffset(
            fromLat: launchLatitude, fromLon: launchLongitude,
            alphaDegrees: alphaDegrees
        )

        // Arrival is checked against the drone's own position, in the same
        // launch frame, metres. No lat/lon round trip.
        // Việc xét đã tới dựa trên vị trí của chính drone, trong cùng khung
        // cất cánh, bằng mét. Không vòng vèo qua vĩ/kinh độ.
        let dx = offset.x - droneX
        let dy = offset.y - droneY
        let distance = sqrt(dx * dx + dy * dy)
        let bearing = target.bearingFrom(phoneLatitude, phoneLongitude)
        let reachedIndex = activeIndex

        if distance <= target.acceptanceRadiusMeters {
            onArrived()
            return record(NavStatus(
                mode: mode, target: offset,
                droneX: droneX, droneY: droneY,
                distanceMeters: distance, bearingDegrees: bearing,
                activeWaypointIndex: reachedIndex,
                arrived: true,
                message: "waypoint reached"
            ))
        }

        return record(NavStatus(
            mode: mode, target: offset,
            droneX: droneX, droneY: droneY,
            distanceMeters: distance, bearingDegrees: bearing,
            activeWaypointIndex: reachedIndex,
            arrived: false,
            message: ""
        ))
    }

    private func onArrived() {
        if mode == .mission {
            activeIndex += 1
            if activeIndex >= mission.count {
                // Last waypoint: mission complete, back to idle.
                // Điểm hành trình cuối: sứ mệnh hoàn tất, trở về nghỉ.
                stop()
            }
        } else {
            // GUIDED and RTL are one-shot: arriving disarms them.
            // GUIDED và RTL chỉ có hiệu lực một lần: đến nơi là tự ngắt.
            stop()
        }
    }

    @discardableResult
    private func record(_ status: NavStatus) -> NavStatus {
        lastStatus = status
        return status
    }
}
