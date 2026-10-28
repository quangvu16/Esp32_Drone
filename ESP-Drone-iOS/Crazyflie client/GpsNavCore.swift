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

// MARK: - Map annotation

/// A map annotation: something on the map the mission must be aware of that
/// cannot be reliably auto-determined. Ported from Valetudo's map annotations
/// (`hypfer/valetudo` 2026.07.0, `ValetudoMapAnnotation`) — the same generic
/// concept of `type` + `points[]` (at least two), re-specified for a drone.
/// Một chú thích bản đồ: một thứ trên bản đồ mà sứ mệnh cần biết nhưng không
/// thể tự đoán một cách đáng tin cậy. Chuyển thể từ map annotations của Valetudo
/// (`hypfer/valetudo` 2026.07.0, `ValetudoMapAnnotation`) — cùng khái niệm
/// `type` + `points[]` (ít nhất hai điểm), định nghĩa lại cho drone.
///
/// These things make the runner behave differently where they are (§10): a
/// keep-out zone holds a drone that drifts into it, a fly-through corridor is
/// the only sanctioned crossing (legs re-route through it), a wind zone demands
/// arrival confirmation before the runner advances.
/// Những thứ này khiến bộ chạy cư xử khác đi tại chỗ chúng (§10): vùng cấm bay
/// giữ drone lạc vào trong, hành lang cho phép bay qua là lối băng duy nhất
/// được chấp thuận (chặng bay được định tuyến lại qua nó), vùng gió đòi hỏi xác
/// nhận đã tới trước khi bộ chạy chuyển tiếp.
struct MapAnnotation {

    enum AnnotationType: Int {
        /// Area the drone must not fly over (people, trees, unseen structures).
        /// Vùng drone không được bay qua (người, cây, công trình không nhìn thấy).
        case keepout = 0
        /// Line: the gate/gap that exists but the map cannot show.
        /// Đường: khe hở/cổng tồn tại nhưng bản đồ không hiển thị được.
        case corridor = 1
        /// Area of unsteady air the tiles look calm in.
        /// Vùng khí không ổn định trong khi bản đồ trông phẳng lặng.
        case wind = 2

        /// Name used on the map, the menus and every status message.
        /// Tên dùng trên bản đồ, trong menu và mọi thông báo trạng thái.
        var label: String {
            switch self {
            case .keepout: return "Keep-out zone"
            case .corridor: return "Fly-through corridor"
            case .wind: return "Wind zone"
            }
        }

        /// Areas carry 4 rect corners; lines carry 2 endpoints.
        /// Vùng mang 4 góc của hình chữ nhật; đường mang 2 đầu mút.
        var isArea: Bool { self != .corridor }
    }

    /// A `{latitude, longitude}` pair in degrees.
    /// Một cặp `{vĩ độ, kinh độ}` tính bằng độ.
    struct Point: Equatable {
        let latitude: Double
        let longitude: Double
    }

    let type: AnnotationType
    /// At least 2 points; exactly 4 (rect corners) for the area types.
    /// Ít nhất 2 điểm; đúng 4 (các góc hình chữ nhật) với các kiểu vùng.
    let points: [Point]

    init(type: AnnotationType, points: [Point]) {
        // Valetudo's rule, kept verbatim: annotations require at least 2 points.
        // Quy tắc của Valetudo, giữ nguyên: chú thích cần ít nhất 2 điểm.
        precondition(points.count >= 2, "annotations require at least 2 points")
        self.type = type
        self.points = points
    }

    /// Two-tap line (corridor): the taps are the two endpoints.
    /// Đường từ hai lần chạm (hành lang): hai lần chạm là hai đầu mút.
    static func line(type: AnnotationType,
                     _ lat0: Double, _ lon0: Double,
                     _ lat1: Double, _ lon1: Double) -> MapAnnotation {
        MapAnnotation(type: type, points: [
            Point(latitude: lat0, longitude: lon0),
            Point(latitude: lat1, longitude: lon1),
        ])
    }

    /// Two-tap area: the taps are opposite corners; the rect is axis-aligned
    /// and expanded to the 4-corner order the areas carry.
    /// Vùng từ hai lần chạm: hai lần chạm là hai góc đối diện; hình chữ nhật
    /// thẳng trục và được mở rộng sang thứ tự 4 góc mà các vùng mang.
    static func rectFromCorners(type: AnnotationType,
                                _ lat0: Double, _ lon0: Double,
                                _ lat1: Double, _ lon1: Double) -> MapAnnotation {
        MapAnnotation(type: type, points: [
            Point(latitude: min(lat0, lat1), longitude: min(lon0, lon1)),
            Point(latitude: max(lat0, lat1), longitude: min(lon0, lon1)),
            Point(latitude: max(lat0, lat1), longitude: max(lon0, lon1)),
            Point(latitude: min(lat0, lat1), longitude: max(lon0, lon1)),
        ])
    }

    /// Centre of the geometry — the via point a leg is routed through.
    /// Tâm của hình học — điểm trung gian mà chặng bay được định tuyến qua.
    func center() -> Point {
        var lat = 0.0
        var lon = 0.0
        for p in points {
            lat += p.latitude
            lon += p.longitude
        }
        return Point(latitude: lat / Double(points.count), longitude: lon / Double(points.count))
    }

    /// Whether (lat, lon) lies inside this annotation. Areas only.
    /// Có nằm trong chú thích này hay không. Chỉ với vùng.
    func containsPoint(_ latitude: Double, _ longitude: Double) -> Bool {
        guard type.isArea else { return false }
        let p = MapAnnotation.toXY(latitude, longitude, latRef: latitude)
        return MapAnnotation.pointInPolygon(p, polygonXY(latRef: latitude))
    }

    /// Whether the straight leg between the two points touches this annotation.
    /// Chặng thẳng giữa hai điểm có chạm vào chú thích này hay không.
    func crossesSegment(_ lat0: Double, _ lon0: Double,
                        _ lat1: Double, _ lon1: Double) -> Bool {
        segmentCrossing(lat0, lon0, lat1, lon1) != nil
    }

    /// First point where the leg enters this annotation, or nil. For corridors
    /// that is where the leg crosses the gate line (the via point).
    /// Điểm mà chặng bay đi vào chú thích này, hoặc nil. Với hành lang đó là nơi
    /// chặng cắt qua đường cổng (điểm trung gian).
    func segmentCrossing(_ lat0: Double, _ lon0: Double,
                         _ lat1: Double, _ lon1: Double) -> Point? {
        let latRef = (lat0 + lat1) / 2.0
        let a = MapAnnotation.toXY(lat0, lon0, latRef: latRef)
        let b = MapAnnotation.toXY(lat1, lon1, latRef: latRef)

        if type == .corridor {
            let g = MapAnnotation.toXY(points[0].latitude, points[0].longitude, latRef: latRef)
            let h = MapAnnotation.toXY(points[1].latitude, points[1].longitude, latRef: latRef)
            guard let hit = MapAnnotation.segmentIntersection(a, b, g, h) else { return nil }
            return MapAnnotation.fromXY(hit, latRef: latRef)
        }

        // Area: the first polygon edge the leg crosses, or the leg starts inside.
        // Vùng: cạnh đầu tiên của đa giác mà chặng cắt qua, hoặc chặng bắt đầu bên trong.
        if containsPoint(lat0, lon0) {
            return Point(latitude: lat0, longitude: lon0)
        }
        let poly = polygonXY(latRef: latRef)
        var best: (x: Double, y: Double)?
        var bestT = Double.greatestFiniteMagnitude
        for i in 0..<poly.count {
            let g = poly[i]
            let h = poly[(i + 1) % poly.count]
            if let hit = MapAnnotation.segmentIntersection(a, b, g, h) {
                let t = (hit.x - a.x) * (b.x - a.x) + (hit.y - a.y) * (b.y - a.y)
                if t < bestT {
                    bestT = t
                    best = hit
                }
            }
        }
        guard let hit = best else { return nil }
        return MapAnnotation.fromXY(hit, latRef: latRef)
    }

    /// Whether the point is within `toleranceMeters` of this annotation — the
    /// map's tap hit-test (tap inside an area or near a corridor line selects).
    /// Điểm có nằm trong `toleranceMeters` của chú thích này hay không — phép
    /// thử chạm của bản đồ (chạm trong vùng hoặc gần đường hành lang để chọn).
    func nearPoint(_ latitude: Double, _ longitude: Double, toleranceMeters: Double) -> Bool {
        let latRef = latitude
        let p = MapAnnotation.toXY(latitude, longitude, latRef: latRef)
        let tol = toleranceMeters / MapAnnotation.metersPerDegree
        if type == .corridor {
            let g = MapAnnotation.toXY(points[0].latitude, points[0].longitude, latRef: latRef)
            let h = MapAnnotation.toXY(points[1].latitude, points[1].longitude, latRef: latRef)
            return MapAnnotation.pointSegmentDistance(p, g, h) <= tol
        }
        if containsPoint(latitude, longitude) {
            return true
        }
        let poly = polygonXY(latRef: latRef)
        for i in 0..<poly.count {
            let g = poly[i]
            let h = poly[(i + 1) % poly.count]
            if MapAnnotation.pointSegmentDistance(p, g, h) <= tol {
                return true
            }
        }
        return false
    }

    // MARK: planar geometry
    //
    // Zones are tens of metres across, so every test runs in a local plane
    // (longitude scaled by cos(latitude), like GeoMath). Good to centimetres.
    // Các vùng rộng vài chục mét, nên mọi phép thử chạy trong một mặt phẳng cục
    // bộ (kinh độ nhân cos(vĩ độ), như GeoMath). Chính xác đến centimét.

    /// Metres per degree on both local-plane axes.
    /// Mét mỗi độ trên cả hai trục mặt phẳng cục bộ.
    private static let metersPerDegree = 111320.0

    private static func toXY(_ latitude: Double, _ longitude: Double, latRef: Double)
        -> (x: Double, y: Double) {
        (x: longitude * cos(latRef * .pi / 180.0), y: latitude)
    }

    private static func fromXY(_ xy: (x: Double, y: Double), latRef: Double) -> Point {
        Point(latitude: xy.y,
              longitude: xy.x / cos(latRef * .pi / 180.0))
    }

    private func polygonXY(latRef: Double) -> [(x: Double, y: Double)] {
        points.map { MapAnnotation.toXY($0.latitude, $0.longitude, latRef: latRef) }
    }

    private static func pointInPolygon(_ p: (x: Double, y: Double),
                                       _ poly: [(x: Double, y: Double)]) -> Bool {
        var inside = false
        var j = poly.count - 1
        for i in 0..<poly.count {
            let pi = poly[i]
            let pj = poly[j]
            if (pi.y > p.y) != (pj.y > p.y),
               p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x {
                inside.toggle()
            }
            j = i
        }
        return inside
    }

    /// Intersection of segments ab and cd in the local plane, or nil.
    /// Giao của hai đoạn ab và cd trong mặt phẳng cục bộ, hoặc nil.
    private static func segmentIntersection(
        _ a: (x: Double, y: Double), _ b: (x: Double, y: Double),
        _ c: (x: Double, y: Double), _ d: (x: Double, y: Double)
    ) -> (x: Double, y: Double)? {
        let r0 = b.x - a.x
        let r1 = b.y - a.y
        let s0 = d.x - c.x
        let s1 = d.y - c.y
        let denom = r0 * s1 - r1 * s0
        if abs(denom) < 1e-12 {
            return nil // parallel (song song)
        }
        let t = ((c.x - a.x) * s1 - (c.y - a.y) * s0) / denom
        let u = ((c.x - a.x) * r1 - (c.y - a.y) * r0) / denom
        guard t >= 0.0, t <= 1.0, u >= 0.0, u <= 1.0 else { return nil }
        return (x: a.x + t * r0, y: a.y + t * r1)
    }

    /// Distance from point p to segment ab in the local plane.
    /// Khoảng cách từ điểm p đến đoạn ab trong mặt phẳng cục bộ.
    private static func pointSegmentDistance(
        _ p: (x: Double, y: Double),
        _ a: (x: Double, y: Double), _ b: (x: Double, y: Double)
    ) -> Double {
        let abx = b.x - a.x
        let aby = b.y - a.y
        let apx = p.x - a.x
        let apy = p.y - a.y
        let ab2 = abx * abx + aby * aby
        var t = ab2 == 0.0 ? 0.0 : (apx * abx + apy * aby) / ab2
        t = max(0.0, min(1.0, t))
        let dx = apx - t * abx
        let dy = apy - t * aby
        return (dx * dx + dy * dy).squareRoot()
    }

    // MARK: handle editing (Valetudo's LineClientStructure / RampClientStructure model)
    //
    // Valetudo edits annotations on the map itself: a ✕ handle deletes, a ✛ handle moves a
    // line's endpoint, ↻ rotates a quad and ↖↘ scales it; dragging the body translates.
    // Every drag snaps when released (postProcess): whole metres and a 5° angle.
    // Valetudo sửa chú thích ngay trên bản đồ: tay nắm ✕ để xoá, tay nắm ✛ để kéo đầu mút của
    // đường, ↻ để xoay khối và ↖↘ để co giãn; kéo thân hình để di chuyển. Mọi thao tác kéo được
    // hút lại khi nhả (postProcess): mét nguyên và góc bội của 5°.

    /// Shortest line we keep in metres (đường ngắn nhất giữ lại, theo mét).
    static let minLineMeters = 1.0

    /// Shortest area edge in metres (cạnh vùng ngắn nhất, theo mét).
    static let minAreaMeters = 2.0

    /// Body drag: move every point by a lat/lon delta in degrees — Valetudo's translate body branch.
    /// Kéo thân hình: dời mọi điểm theo chênh lệch vĩ/kinh độ — nhánh thân hình trong translate.
    func translated(dLat: Double, dLon: Double) -> MapAnnotation {
        MapAnnotation(type: type, points: points.map {
            Point(latitude: $0.latitude + dLat, longitude: $0.longitude + dLon)
        })
    }

    /// Endpoint drag (the ✛ handle): reposition one vertex.
    /// Kéo đầu mút (tay nắm ✛): đặt lại một đỉnh.
    func movingPoint(index: Int, to point: Point) -> MapAnnotation {
        var copy = points
        guard copy.indices.contains(index) else { return self }
        copy[index] = point
        return MapAnnotation(type: type, points: copy)
    }

    /// Rotate handle: rotate every point about the centroid by the angle the pointer swept —
    /// Valetudo's RampClientStructure rotate branch.
    /// Tay nắm xoay: xoay mọi điểm quanh tâm theo góc con trỏ đã quét — nhánh xoay của
    /// RampClientStructure trong Valetudo.
    func rotated(last: Point, current: Point) -> MapAnnotation {
        let c = center()
        let latRef = c.latitude
        let cXY = MapAnnotation.toXYMeters(c.latitude, c.longitude, latRef: latRef)
        let lastXY = MapAnnotation.toXYMeters(last.latitude, last.longitude, latRef: latRef)
        let curXY = MapAnnotation.toXYMeters(current.latitude, current.longitude, latRef: latRef)
        var delta = atan2(curXY.y - cXY.y, curXY.x - cXY.x)
            - atan2(lastXY.y - cXY.y, lastXY.x - cXY.x)
        if delta > .pi {
            delta -= 2 * .pi
        } else if delta < -.pi {
            delta += 2 * .pi
        }
        let cosA = cos(delta)
        let sinA = sin(delta)
        return MapAnnotation(type: type, points: points.map { p in
            let xy = MapAnnotation.toXYMeters(p.latitude, p.longitude, latRef: latRef)
            let dx = xy.x - cXY.x
            let dy = xy.y - cXY.y
            return MapAnnotation.fromXYMeters(
                (x: cXY.x + dx * cosA - dy * sinA, y: cXY.y + dx * sinA + dy * cosA),
                latRef: latRef)
        })
    }

    /// Scale handle: rebuild the far corners so the quad reaches the pointer — Valetudo's
    /// resize branch, each extent floored at `minAreaMeters`.
    /// Tay nắm co giãn: dựng lại các góc xa để khối chạm con trỏ — nhánh co giãn của Valetudo,
    /// mỗi chiều kẹp tối thiểu `minAreaMeters`.
    func resized(pointer: Point) -> MapAnnotation {
        guard type.isArea, points.count >= 4 else { return self }
        let latRef = center().latitude
        let p0 = MapAnnotation.toXYMeters(points[0].latitude, points[0].longitude, latRef: latRef)
        let p1 = MapAnnotation.toXYMeters(points[1].latitude, points[1].longitude, latRef: latRef)
        let p3 = MapAnnotation.toXYMeters(points[3].latitude, points[3].longitude, latRef: latRef)
        let ptr = MapAnnotation.toXYMeters(pointer.latitude, pointer.longitude, latRef: latRef)

        let lenW = hypot(p1.x - p0.x, p1.y - p0.y)
        let lenH = hypot(p3.x - p0.x, p3.y - p0.y)
        let uW: (x: Double, y: Double) = lenW < 1e-6
            ? (x: 1, y: 0)
            : (x: (p1.x - p0.x) / lenW, y: (p1.y - p0.y) / lenW)
        let uH: (x: Double, y: Double) = lenH < 1e-6
            ? (x: 0, y: 1)
            : (x: (p3.x - p0.x) / lenH, y: (p3.y - p0.y) / lenH)

        let mouseVecX = ptr.x - p0.x
        let mouseVecY = ptr.y - p0.y
        let newWidth = max(MapAnnotation.minAreaMeters, mouseVecX * uW.x + mouseVecY * uW.y)
        let newHeight = max(MapAnnotation.minAreaMeters, mouseVecX * uH.x + mouseVecY * uH.y)

        var rebuilt = [points[0]]
        rebuilt.append(MapAnnotation.fromXYMeters(
            (x: p0.x + newWidth * uW.x, y: p0.y + newWidth * uW.y), latRef: latRef))
        rebuilt.append(MapAnnotation.fromXYMeters(
            (x: p0.x + newWidth * uW.x + newHeight * uH.x,
             y: p0.y + newWidth * uW.y + newHeight * uH.y), latRef: latRef))
        rebuilt.append(MapAnnotation.fromXYMeters(
            (x: p0.x + newHeight * uH.x, y: p0.y + newHeight * uH.y), latRef: latRef))
        return MapAnnotation(type: type, points: rebuilt)
    }

    /// Snap when the drag ends (Valetudo's `postProcess`): round p0 to whole metres, snap the
    /// angle to 5° and the sizes to whole metres, and never leave a shape too small to edit.
    /// Hút lại khi thả tay (postProcess của Valetudo): làm tròn p0 về mét nguyên, hút góc về bội
    /// 5° và kích thước về mét nguyên, không để hình nhỏ quá mức còn sửa được.
    func snapped() -> MapAnnotation {
        let latRef = center().latitude
        return type.isArea && points.count >= 4
            ? snappedArea(latRef: latRef)
            : snappedLine(latRef: latRef)
    }

    /// Line snap (Valetudo's LineClientStructure.postProcess).
    /// Hút đường (postProcess của LineClientStructure).
    private func snappedLine(latRef: Double) -> MapAnnotation {
        var p0 = MapAnnotation.toXYMeters(points[0].latitude, points[0].longitude, latRef: latRef)
        var p1 = MapAnnotation.toXYMeters(points[1].latitude, points[1].longitude, latRef: latRef)
        p0 = (x: p0.x.rounded(), y: p0.y.rounded())

        var deltaX = abs(p0.x - p1.x)
        var deltaY = abs(p0.y - p1.y)
        var distance = hypot(deltaX, deltaY)

        // Keep the line long enough to grab again (Valetudo nudges and retries).
        // Giữ đường đủ dài để nắm lại được (Valetudo dịch nhẹ rồi thử lại).
        var guardCount = 0
        while distance < MapAnnotation.minLineMeters && guardCount < 8 {
            if p1.x > p0.x {
                p1.x += 1
            } else if p1.x < p0.x {
                p1.x -= 1
            } else {
                p1.x += 1
            }
            if p1.y > p0.y {
                p1.y += 1
            } else if p1.y < p0.y {
                p1.y -= 1
            } else {
                p1.y += 1
            }
            deltaX = abs(p0.x - p1.x)
            deltaY = abs(p0.y - p1.y)
            distance = hypot(deltaX, deltaY)
            guardCount += 1
        }

        let angle = atan2(deltaY, deltaX) * 180 / .pi
        let newAngle = (angle / 5).rounded() * 5
        let roundedDistance = max(MapAnnotation.minLineMeters, distance.rounded())

        var xOffset = roundedDistance * cos(newAngle * .pi / 180)
        var yOffset = roundedDistance * sin(newAngle * .pi / 180)
        if p0.x > p1.x {
            xOffset = -xOffset
        }
        if p0.y > p1.y {
            yOffset = -yOffset
        }
        return MapAnnotation(type: type, points: [
            MapAnnotation.fromXYMeters(p0, latRef: latRef),
            MapAnnotation.fromXYMeters((x: p0.x + xOffset, y: p0.y + yOffset), latRef: latRef),
        ])
    }

    /// Area snap (Valetudo's RampClientStructure.postProcess).
    /// Hút vùng (postProcess của RampClientStructure).
    private func snappedArea(latRef: Double) -> MapAnnotation {
        let xy = points.map {
            MapAnnotation.toXYMeters($0.latitude, $0.longitude, latRef: latRef)
        }
        let cx = (xy[0].x + xy[1].x + xy[2].x + xy[3].x) / 4
        let cy = (xy[0].y + xy[1].y + xy[2].y + xy[3].y) / 4

        let wTop = hypot(xy[1].x - xy[0].x, xy[1].y - xy[0].y)
        let wBot = hypot(xy[2].x - xy[3].x, xy[2].y - xy[3].y)
        let width = (wTop + wBot) / 2

        let hLeft = hypot(xy[3].x - xy[0].x, xy[3].y - xy[0].y)
        let hRight = hypot(xy[2].x - xy[1].x, xy[2].y - xy[1].y)
        let height = (hLeft + hRight) / 2

        var angleRad = atan2(xy[1].y - xy[0].y, xy[1].x - xy[0].x)
        let snappedDeg = (angleRad * 180 / .pi / 5).rounded() * 5
        angleRad = snappedDeg * .pi / 180

        let rW = max(MapAnnotation.minAreaMeters, width.rounded())
        let rH = max(MapAnnotation.minAreaMeters, height.rounded())

        let snappedLeft = (cx - rW / 2).rounded()
        let snappedTop = (cy - rH / 2).rounded()
        let rCx = snappedLeft + rW / 2
        let rCy = snappedTop + rH / 2

        let halfW = rW / 2
        let halfH = rH / 2
        let cosA = cos(angleRad)
        let sinA = sin(angleRad)

        func corner(_ u: Double, _ v: Double) -> Point {
            MapAnnotation.fromXYMeters(
                (x: rCx + (u * cosA - v * sinA), y: rCy + (u * sinA + v * cosA)),
                latRef: latRef)
        }
        return MapAnnotation(type: type, points: [
            corner(-halfW, -halfH),
            corner(halfW, -halfH),
            corner(halfW, halfH),
            corner(-halfW, halfH),
        ])
    }

    /// Local-plane x/y in metres (tọa độ x/y mặt phẳng cục bộ theo mét).
    private static func toXYMeters(_ latitude: Double, _ longitude: Double, latRef: Double)
        -> (x: Double, y: Double) {
        (x: longitude * cos(latRef * .pi / 180.0) * metersPerDegree,
         y: latitude * metersPerDegree)
    }

    /// Inverse of `toXYMeters` (nghịch đảo của `toXYMeters`).
    private static func fromXYMeters(_ xy: (x: Double, y: Double), latRef: Double) -> Point {
        Point(latitude: xy.y / metersPerDegree,
              longitude: xy.x / (metersPerDegree * cos(latRef * .pi / 180.0)))
    }
}

// MARK: - Mission

enum MissionError: Error, Equatable {
    case emptyFile
    case unsupportedHeader(String)
    case malformedRow(String)
}

/// One map annotation while its vertex rows are being collected out of a file.
/// Một chú thích bản đồ trong khi các dòng đỉnh của nó được gom từ file.
private final class StagedAnnotation {
    let type: MapAnnotation.AnnotationType
    let ordinal: Int
    /// Vertex index → point; sorted on assembly so the points rebuild in order.
    /// Chỉ số đỉnh → điểm; sắp xếp khi lắp ráp để các điểm dựng lại đúng thứ tự.
    var vertices: [Int: MapAnnotation.Point] = [:]

    init(type: MapAnnotation.AnnotationType, ordinal: Int) {
        self.type = type
        self.ordinal = ordinal
    }

    static func stage(_ staged: inout [StagedAnnotation],
                      type: MapAnnotation.AnnotationType, ordinal: Int,
                      vertex: Int, latitude: Double, longitude: Double) {
        if let existing = staged.first(where: { $0.type == type && $0.ordinal == ordinal }) {
            existing.vertices[vertex] = MapAnnotation.Point(latitude: latitude, longitude: longitude)
        } else {
            let created = StagedAnnotation(type: type, ordinal: ordinal)
            created.vertices[vertex] = MapAnnotation.Point(latitude: latitude, longitude: longitude)
            staged.append(created)
        }
    }

    func makeAnnotation() -> MapAnnotation {
        MapAnnotation(type: type, points: vertices.sorted { $0.key < $1.key }.map { $0.value })
    }
}

/// An ordered GPS mission: a home position plus waypoints to visit.
/// Serialises to the Mission Planner `.waypoints` format (`QGC WPL 110`).
/// Sứ mệnh GPS có thứ tự: một vị trí home cộng các điểm hành trình cần ghé.
/// Tuần tự hóa theo định dạng `.waypoints` của Mission Planner (`QGC WPL 110`).
class Mission {
    static let wplHeader = "QGC WPL 110"
    static let cmdNavWaypoint = 16

    /// Extension row carrying one map-annotation vertex (§10). Not a MAVLink
    /// command — it is ours by contract and other planners skip it like any
    /// unknown command. Columns: `p1` = type wire value, `p2` = ordinal of the
    /// annotation within its type, `p3` = vertex index, `x`/`y` = lat/lon.
    /// Dòng mở rộng mang một đỉnh của chú thích bản đồ (§10). Không phải lệnh
    /// MAVLink — theo hợp đồng nó là của ta và các trình lập kế hoạch khác bỏ qua
    /// nó như bất kỳ lệnh lạ nào. Cột: `p1` = giá trị truyền kiểu, `p2` = thứ tự
    /// của chú thích trong kiểu, `p3` = chỉ số đỉnh, `x`/`y` = vĩ/kinh độ.
    static let cmdMapAnnotation = 31000

    private(set) var waypoints: [Waypoint] = []

    /// Map annotations (Valetudo's `setMapAnnotations` replace-all semantics:
    /// mission files persist the whole set, never a partial write).
    /// Các chú thích bản đồ (ngữ nghĩa thay-toàn-bộ `setMapAnnotations` của
    /// Valetudo: tệp sứ mệnh lưu toàn bộ tập, không bao giờ ghi một phần).
    private(set) var annotations: [MapAnnotation] = []

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

    // MARK: map annotations

    func addAnnotation(_ annotation: MapAnnotation) {
        annotations.append(annotation)
    }

    func removeAnnotation(at index: Int) {
        guard annotations.indices.contains(index) else { return }
        annotations.remove(at: index)
    }

    /// Replace one annotation in place — `MapAnnotation` is a value type, so handle
    /// drags return a new instance and the mission has to write it back at the same
    /// index to keep the selection stable.
    /// Thay một chú thích tại chỗ — `MapAnnotation` là kiểu giá trị nên thao tác kéo
    /// tay nắm trả về bản mới và mission phải ghi đè đúng chỉ số để giữ lựa chọn ổn định.
    func updateAnnotation(_ annotation: MapAnnotation, at index: Int) {
        guard annotations.indices.contains(index) else { return }
        annotations[index] = annotation
    }

    func clearAnnotations() {
        annotations.removeAll()
    }

    func annotation(at index: Int) -> MapAnnotation? {
        annotations.indices.contains(index) ? annotations[index] : nil
    }

    /// Live count for the grouped Add menu ("Keep-out zone (2)").
    /// Số lượng trực tiếp cho menu Thêm gộp nhóm ("Keep-out zone (2)").
    func annotationCount(_ type: MapAnnotation.AnnotationType) -> Int {
        annotations.filter { $0.type == type }.count
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
    /// recorded yet the first waypoint is used so the file stays valid. Map
    /// annotations ride after the waypoints as `cmdMapAnnotation` extension
    /// rows, one row per vertex (§10).
    /// Tuần tự hóa theo định dạng điểm hành trình của Mission Planner / QGC.
    /// Chỉ số 0 là vị trí home (có cờ `current`). Khi chưa ghi nhận home nào,
    /// điểm hành trình đầu tiên được dùng để file vẫn hợp lệ. Các chú thích bản
    /// đồ đi sau các điểm hành trình dưới dạng dòng mở rộng `cmdMapAnnotation`,
    /// mỗi dòng một đỉnh (§10).
    func toWaypointsFile() -> String {
        var lines = [Mission.wplHeader]
        var rowIndex = 0
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
            lines.append(Mission.formatWplLine(index: rowIndex, current: 1, latitude: lat, longitude: lon))
            rowIndex += 1
        }
        for waypoint in waypoints {
            lines.append(Mission.formatWplLine(
                index: rowIndex, current: 0,
                latitude: waypoint.latitude, longitude: waypoint.longitude
            ))
            rowIndex += 1
        }
        // Annotations come last, in insertion order; p2 renumbers per type 0..n-1
        // like Valetudo's replace-all setMapAnnotations write.
        // Chú thích đi cuối, theo thứ tự chèn; p2 đánh số lại theo từng kiểu 0..n-1
        // như cách Valetudo ghi setMapAnnotations thay-toàn-bộ.
        for (index, annotation) in annotations.enumerated() {
            let ordinal = annotations[..<index].filter { $0.type == annotation.type }.count
            for (vertex, point) in annotation.points.enumerated() {
                lines.append(Mission.formatAnnotationLine(
                    index: rowIndex, typeWire: annotation.type.rawValue,
                    ordinal: ordinal, vertex: vertex,
                    latitude: point.latitude, longitude: point.longitude
                ))
                rowIndex += 1
            }
        }
        return lines.joined(separator: "\n") + "\n"
    }

    /// Parses a Mission Planner / QGroundControl waypoint file.
    /// Row 0 is the home position; subsequent `NAV_WAYPOINT` (cmd 16) rows are
    /// waypoints. Non-navigation commands are skipped so files planned for a
    /// rover with extra `DO_` commands still load their waypoints.
    /// `cmdMapAnnotation` rows assemble back into map annotations and never
    /// count as row 0 — an annotations-only file must not invent a home.
    /// Phân tích một file điểm hành trình của Mission Planner / QGroundControl.
    /// Dòng 0 là vị trí home; các dòng `NAV_WAYPOINT` (cmd 16) theo sau là các
    /// điểm hành trình. Các lệnh không phải điều hướng bị bỏ qua nên file lập
    /// kế hoạch cho xe mặt đất với các lệnh `DO_` phụ vẫn nạp được điểm hành trình.
    /// Các dòng `cmdMapAnnotation` được lắp ráp lại thành chú thích bản đồ và
    /// không bao giờ được tính là dòng 0 — file chỉ-có-chú-thích không được bịa
    /// ra một vị trí home.
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

        var rowIndex = 0
        var staged: [StagedAnnotation] = []
        for rawLine in lines.dropFirst() {
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

            if command == Mission.cmdMapAnnotation {
                // p1 = type wire value, p2 = ordinal within type, p3 = vertex index.
                // p1 = giá trị truyền kiểu, p2 = thứ tự trong kiểu, p3 = chỉ số đỉnh.
                guard let typeWire = Int(fields[4].trimmingCharacters(in: .whitespaces)),
                      let ordinal = Int(fields[5].trimmingCharacters(in: .whitespaces)),
                      let vertex = Int(fields[6].trimmingCharacters(in: .whitespaces)),
                      let type = MapAnnotation.AnnotationType(rawValue: typeWire)
                else {
                    throw MissionError.malformedRow(line)
                }
                StagedAnnotation.stage(
                    &staged, type: type, ordinal: ordinal, vertex: vertex,
                    latitude: latitude, longitude: longitude
                )
                continue
            }

            if rowIndex == 0 {
                mission.setHome(latitude, longitude)
            } else if command == Mission.cmdNavWaypoint {
                mission.addWaypoint(Waypoint(latitude: latitude, longitude: longitude))
            }
            rowIndex += 1
        }

        for annotation in staged {
            guard annotation.vertices.count >= 2 else {
                throw MissionError.malformedRow("map annotation with fewer than 2 points")
            }
            mission.addAnnotation(annotation.makeAnnotation())
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

    /// One annotation vertex: p1 = type, p2 = ordinal within type, p3 = vertex.
    /// Một đỉnh của chú thích: p1 = kiểu, p2 = thứ tự trong kiểu, p3 = đỉnh.
    private static func formatAnnotationLine(index: Int, typeWire: Int, ordinal: Int,
                                             vertex: Int, latitude: Double, longitude: Double) -> String {
        String(
            format: "%d\t%d\t3\t%d\t%d\t%d\t%d\t0\t%.8f\t%.8f\t0\t1",
            index, 0, Mission.cmdMapAnnotation, typeWire, ordinal, vertex, latitude, longitude
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

    /// The drone's last known lat/lon (launch-frame back-projection).
    /// Vĩ/kinh độ đã biết gần nhất của drone (dựng ngược từ khung cất cánh).
    private var droneLatitude: Double?
    private var droneLongitude: Double?

    /// Pending fly-through corridor crossing: when a leg crosses a corridor the
    /// drone aims at the gate first and continues after crossing it (§10).
    /// Điểm băng qua hành lang đang chờ: khi một chặng cắt qua hành lang, drone
    /// nhắm vào cổng trước rồi tiếp tục sau khi băng qua (§10).
    private var via: Waypoint?

    /// The drone drifted into a keep-out zone; targeting is suppressed.
    /// Drone lạc vào vùng cấm bay; việc nhắm mục tiêu bị ngưng.
    private(set) var isZoneHold = false

    /// Pilot pressed Resume while held: fly on anyway. Cleared on zone exit.
    /// Người lái bấm Tiếp tục khi đang giữ: cứ bay tiếp. Xoá khi rời vùng.
    private var zoneOverride = false

    /// Consecutive in-radius ticks a wind-zone arrival still needs.
    /// Các nhịp liên tiếp trong bán kính mà việc tới vùng gió còn cần.
    private var arrivalConfirmTicks = 0

    /// Ticks a wind-zone arrival must hold inside the radius before counting.
    /// The fix bounces in unsteady air; two consecutive ticks is the cheapest
    /// filter that notices.
    /// Số nhịp việc tới vùng gió phải nằm trong bán kính trước khi được tính. Vị
    /// trí GPS nhảy trong khí không ổn định; hai nhịp liên tiếp là bộ lọc rẻ
    /// nhất phát hiện điều đó.
    static let arrivalConfirmTickCount = 2

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

    /// The waypoint being sought right now, or `nil` when idle. A pending
    /// corridor gate wins.
    /// Điểm hành trình đang tìm lúc này, hoặc `nil` khi nghỉ. Cổng hành lang
    /// đang chờ được ưu tiên.
    func currentTarget() -> Waypoint? {
        if let via = via {
            return via
        }
        if mode == .mission {
            return mission.waypoint(at: activeIndex)
        }
        return guidedTarget
    }

    // MARK: map annotations (PROTOCOL.md §10)

    /// Non-nil refusal when a waypoint may not be placed here: it sits inside a
    /// keep-out zone. Checked on every map tap and every file load.
    /// Lời từ chối khác nil khi một điểm hành trình không được đặt ở đây: nó nằm
    /// trong vùng cấm bay. Được kiểm tra mỗi lần chạm bản đồ và mỗi lần nạp file.
    func checkWaypoint(_ latitude: Double, _ longitude: Double) -> String? {
        for annotation in mission.annotations
        where annotation.type == .keepout && annotation.containsPoint(latitude, longitude) {
            return "inside keep-out zone - move the waypoint"
        }
        return nil
    }

    /// Non-nil refusal when a planned leg may not be flown as drawn: it crosses
    /// a keep-out zone and no fly-through corridor sanctions the crossing.
    /// Lời từ chối khác nil khi một chặng được lên kế hoạch không được bay theo
    /// nét vẽ: nó cắt qua vùng cấm bay và không có hành lang nào chấp thuận lối băng.
    func checkLeg(_ fromLat: Double, _ fromLon: Double,
                  _ toLat: Double, _ toLon: Double) -> String? {
        for annotation in mission.annotations
        where annotation.type == .keepout && annotation.crossesSegment(fromLat, fromLon, toLat, toLon) {
            return corridorVia(fromLat, fromLon, toLat, toLon) == nil
                ? "leg crosses a keep-out zone with no fly-through corridor"
                : nil
        }
        return nil
    }

    /// The point where the leg crosses a fly-through corridor, or nil. That
    /// point is the via the runner targets before the real destination.
    /// Điểm mà chặng cắt qua hành lang cho phép bay qua, hoặc nil. Điểm đó là
    /// trung gian mà bộ chạy nhắm tới trước đích thật.
    func corridorVia(_ fromLat: Double, _ fromLon: Double,
                     _ toLat: Double, _ toLon: Double) -> MapAnnotation.Point? {
        for annotation in mission.annotations where annotation.type == .corridor {
            if let hit = annotation.segmentCrossing(fromLat, fromLon, toLat, toLon) {
                return hit
            }
        }
        return nil
    }

    /// Validates the whole plan the way `startMission()` will fly it: no
    /// waypoint inside a keep-out zone, no leg crossing one without a corridor.
    /// Returns the first refusal or nil.
    /// Kiểm tra toàn bộ kế hoạch đúng như `startMission()` sẽ bay: không điểm
    /// hành trình nào nằm trong vùng cấm bay, không chặng nào cắt qua vùng đó mà
    /// không có hành lang. Trả về lời từ chối đầu tiên hoặc nil.
    func checkMission() -> String? {
        guard !mission.isEmpty else { return "no waypoints to fly" }
        var previous: (latitude: Double, longitude: Double)?
        if let homeLat = mission.homeLatitude, let homeLon = mission.homeLongitude {
            previous = (homeLat, homeLon)
        }
        for (index, waypoint) in mission.waypoints.enumerated() {
            if let refusal = checkWaypoint(waypoint.latitude, waypoint.longitude) {
                return String(format: "waypoint %d: %@", index + 1, refusal)
            }
            if let prev = previous,
               let refusal = checkLeg(prev.latitude, prev.longitude,
                                      waypoint.latitude, waypoint.longitude) {
                return String(format: "leg to waypoint %d: %@", index + 1, refusal)
            }
            previous = (waypoint.latitude, waypoint.longitude)
        }
        return nil
    }

    /// The annotation of this type containing the point, or nil.
    /// Chú thích kiểu này chứa điểm đó, hoặc nil.
    private func annotationAt(_ type: MapAnnotation.AnnotationType,
                              _ latitude: Double, _ longitude: Double) -> MapAnnotation? {
        mission.annotations.first {
            $0.type == type && $0.containsPoint(latitude, longitude)
        }
    }

    /// Arms corridor via-routing for a leg: when the straight leg crosses a
    /// fly-through corridor the drone aims at the gate first (§10).
    /// Gắn định tuyến qua hành lang cho một chặng: khi chặng thẳng cắt qua hành
    /// lang, drone nhắm vào cổng trước (§10).
    private func armLeg(fromLat: Double, fromLon: Double, to: Waypoint?) {
        via = nil
        arrivalConfirmTicks = 0
        guard let to = to, !fromLat.isNaN, !fromLon.isNaN else { return }
        if let hit = corridorVia(fromLat, fromLon, to.latitude, to.longitude) {
            // A small acceptance radius: the gate is narrow and the point is a
            // crossing, not a stop.
            // Bán kính chấp nhận nhỏ: cổng hẹp và điểm này là nơi băng qua,
            // không phải điểm dừng.
            via = Waypoint(latitude: hit.latitude, longitude: hit.longitude,
                           name: "corridor", acceptanceRadiusMeters: 1.5)
        }
    }

    /// Where a new leg starts: the drone's last known position, else launch.
    /// Nơi một chặng mới bắt đầu: vị trí đã biết gần nhất của drone, nếu không
    /// thì điểm cất cánh.
    private var legOrigin: (latitude: Double, longitude: Double) {
        (latitude: droneLatitude ?? launchLatitude,
         longitude: droneLongitude ?? launchLongitude)
    }

    /// Launch-frame (x, y) back to lat/lon, for the annotation checks.
    /// (x, y) theo khung cất cánh quy ngược về vĩ/kinh độ, cho các phép thử chú thích.
    private func launchFrameToLatLon(x: Double, y: Double) -> (latitude: Double, longitude: Double) {
        let ne = LaunchFrame.toNorthEast(x: x, y: y, alphaDegrees: alphaDegrees)
        let distance = (ne.north * ne.north + ne.east * ne.east).squareRoot()
        let bearing = atan2(ne.east, ne.north) * 180.0 / .pi
        let dest = GeoMath.destinationPoint(launchLatitude, launchLongitude, bearing, distance)
        return (latitude: dest.latitude, longitude: dest.longitude)
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
        // The plan's first leg starts at home — the drone launches from there.
        // Chặng đầu của kế hoạch bắt đầu từ home — drone cất cánh từ đó.
        armLeg(fromLat: mission.homeLatitude ?? .nan,
               fromLon: mission.homeLongitude ?? .nan,
               to: mission.waypoint(at: 0))
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
        armLeg(fromLat: legOrigin.latitude, fromLon: legOrigin.longitude, to: guidedTarget)
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
        armLeg(fromLat: legOrigin.latitude, fromLon: legOrigin.longitude, to: guidedTarget)
        return true
    }

    /// Pause: stop sending a new target. The drone holds after 3 s (§6).
    /// Tạm dừng: ngưng gửi mục tiêu mới. Drone giữ nguyên sau 3 giây (§6).
    func pause() {
        if isNavigating { isPaused = true }
    }

    func resume() {
        isPaused = false
        if isZoneHold {
            // Pilot authority: they saw the keep-out hold and chose to fly on.
            // Cleared when the drone leaves the zone, so the next entry holds.
            // Quyền của người lái: họ thấy cảnh báo giữ trong vùng cấm bay và
            // chọn bay tiếp. Xoá khi drone rời vùng, nên lần vào sau sẽ giữ lại.
            zoneOverride = true
            isZoneHold = false
        }
    }

    /// Disarms navigation and returns to manual/idle control.
    /// Ngắt điều hướng và trở về điều khiển thủ công/nghỉ.
    func stop() {
        mode = .idle
        isPaused = false
        guidedTarget = nil
        activeIndex = 0
        via = nil
        isZoneHold = false
        zoneOverride = false
        arrivalConfirmTicks = 0
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

        // Where the drone is on the map, so the annotations can judge it (§10).
        // Drone đang ở đâu trên bản đồ, để các chú thích có thể đánh giá nó (§10).
        let droneLatLon = launchFrameToLatLon(x: droneX, y: droneY)
        droneLatitude = droneLatLon.latitude
        droneLongitude = droneLatLon.longitude

        // Keep-out zones make the runner behave differently where they are: a
        // drone that drifts into one is held there, not chased onward through
        // it. RTL is exempt — safety outranks the hint — and Resume is the
        // pilot's override.
        // Vùng cấm bay khiến bộ chạy cư xử khác đi tại chỗ chúng: drone lạc vào
        // trong sẽ bị giữ ở đó, không bị đuổi tiếp xuyên qua. RTL được miễn — an
        // toàn quan trọng hơn gợi ý — và nút Tiếp tục là quyền ghi đè của người lái.
        let keepOut = annotationAt(.keepout, droneLatLon.latitude, droneLatLon.longitude)
        if keepOut == nil {
            // Left the zone: the next entry holds again.
            // Đã rời vùng: lần vào sau sẽ giữ lại.
            zoneOverride = false
            isZoneHold = false
        } else if mode != .rtl, !zoneOverride {
            isZoneHold = true
            return record(NavStatus(
                mode: mode, target: nil,
                droneX: droneX, droneY: droneY,
                distanceMeters: nil, bearingDegrees: nil,
                activeWaypointIndex: activeIndex, arrived: false,
                message: "keep-out zone entered - holding"
            ))
        } else {
            isZoneHold = false
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
            // Wind zones demand a settled fix before the runner advances (§10).
            // Vùng gió đòi hỏi vị trí ổn định trước khi bộ chạy chuyển tiếp (§10).
            if annotationAt(.wind, target.latitude, target.longitude) != nil
                || annotationAt(.wind, droneLatLon.latitude, droneLatLon.longitude) != nil {
                arrivalConfirmTicks += 1
                if arrivalConfirmTicks < Navigator.arrivalConfirmTickCount {
                    return record(NavStatus(
                        mode: mode, target: offset,
                        droneX: droneX, droneY: droneY,
                        distanceMeters: distance, bearingDegrees: bearing,
                        activeWaypointIndex: reachedIndex, arrived: false,
                        message: "wind zone - confirm arrival"
                    ))
                }
            }
            arrivalConfirmTicks = 0

            if let viaTarget = via, target == viaTarget {
                // The corridor is a gate, not a stop: cross it and keep going.
                // Hành lang là cổng, không phải điểm dừng: băng qua rồi tiếp tục.
                via = nil
                return record(NavStatus(
                    mode: mode, target: offset,
                    droneX: droneX, droneY: droneY,
                    distanceMeters: distance, bearingDegrees: bearing,
                    activeWaypointIndex: reachedIndex, arrived: false,
                    message: "corridor crossed - continuing"
                ))
            }

            onArrived(reached: target)
            return record(NavStatus(
                mode: mode, target: offset,
                droneX: droneX, droneY: droneY,
                distanceMeters: distance, bearingDegrees: bearing,
                activeWaypointIndex: reachedIndex,
                arrived: true,
                message: "waypoint reached"
            ))
        }
        arrivalConfirmTicks = 0

        return record(NavStatus(
            mode: mode, target: offset,
            droneX: droneX, droneY: droneY,
            distanceMeters: distance, bearingDegrees: bearing,
            activeWaypointIndex: reachedIndex,
            arrived: false,
            message: ""
        ))
    }

    private func onArrived(reached: Waypoint) {
        if mode == .mission {
            activeIndex += 1
            if activeIndex >= mission.count {
                // Last waypoint: mission complete, back to idle.
                // Điểm hành trình cuối: sứ mệnh hoàn tất, trở về nghỉ.
                stop()
            } else {
                // The next leg starts here — arm it so a corridor gate gets found.
                // Chặng tiếp theo bắt đầu ở đây — gắn nó để cổng hành lang được nhận ra.
                armLeg(fromLat: reached.latitude, fromLon: reached.longitude,
                       to: mission.waypoint(at: activeIndex))
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

/// Shared plan state for the Mission and Indoor Map screens (PROTOCOL.md §10):
/// one Navigator and one pending "Go to" target, so the plan survives switching
/// between the two screens — the vacuum-map-card's model keeps its selections
/// in one place for the whole card, and this is the same idea at app scale.
/// Trạng thái kế hoạch dùng chung cho các màn hình Mission và Indoor Map
/// (PROTOCOL.md §10): một Navigator và một mục tiêu "Go to" đang chờ, để kế
/// hoạch còn nguyên khi đổi qua lại hai màn hình — model của vacuum-map-card
/// giữ các lựa chọn tại một chỗ cho cả thẻ, và đây là cùng ý tưởng ở phạm vi app.
final class NavStore {
    static let shared = NavStore()

    /// The one plan: waypoints, annotations and the runner.
    /// Kế hoạch duy nhất: các điểm hành trình, chú thích và bộ chạy.
    let navigator = Navigator()

    /// Pending "Go to" tap target (the card's MANUAL_POINT selection) before
    /// GUIDED starts. Shared so the Indoor Map page can set it and the Mission
    /// page can fly it.
    /// Mục tiêu chạm "Go to" đang chờ (lựa chọn MANUAL_POINT của thẻ) trước khi
    /// GUIDED bắt đầu. Dùng chung để trang Indoor Map đặt và trang Mission bay nó.
    var tapTarget: Waypoint?

    private init() {}
}
