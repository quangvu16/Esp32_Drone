//
//  GpsNavCoreTests.swift
//  Crazyflie clientTests
//
//  Ports of OpenBot's GpsNavCoreTests: geo math, the Mission Planner .waypoints
//  codec, mission edits and Navigator sequencing. Also pins the launch-frame
//  rotation from PROTOCOL.md §6 — the one formula the whole GPS feature rests
//  on.
//
//  Bản chuyển (port) từ GpsNavCoreTests của OpenBot: toán địa lý, codec
//  file .waypoints của Mission Planner, các thao tác chỉnh sứ mệnh và trình
//  tự xử lý của Navigator. Cũng ghim cố định phép quay của khung cất cánh
//  (launch frame) từ PROTOCOL.md §6 — công thức duy nhất mà toàn bộ tính
//  năng GPS dựa vào.
//

import XCTest
// The app target's PRODUCT_MODULE_NAME is ESP_Drone (PRODUCT_NAME = "ESP-Drone"),
// not the target name — `@testable import Crazyflie_client` fails to resolve.
// PRODUCT_MODULE_NAME của app target là ESP_Drone (PRODUCT_NAME = "ESP-Drone"),
// không phải tên target — `@testable import Crazyflie_client` không resolve được.
@testable import ESP_Drone

final class GpsNavCoreTests: XCTestCase {

    // MARK: - Geo math

    func testDistanceZero() {
        XCTAssertEqual(GeoMath.distanceMeters(0, 0, 0, 0), 0, accuracy: 1e-6)
    }

    func testDistanceOneDegreeLatitude() {
        // 1° of latitude ≈ 111.2 km anywhere on the sphere.
        // 1° vĩ độ ≈ 111.2 km tại bất kỳ đâu trên mặt cầu.
        let d = GeoMath.distanceMeters(0, 0, 1, 0)
        XCTAssertEqual(d, 111_194.9, accuracy: 50)
    }

    func testBearingDueNorth() {
        let b = GeoMath.bearingDegrees(0, 0, 1, 0)
        XCTAssertEqual(b, 0, accuracy: 1e-6)
    }

    func testBearingDueEast() {
        let b = GeoMath.bearingDegrees(0, 0, 0, 1)
        XCTAssertEqual(b, 90, accuracy: 1e-6)
    }

    func testWrapDegrees() {
        XCTAssertEqual(GeoMath.wrapDegrees(0), 0, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.wrapDegrees(180), -180, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.wrapDegrees(-180), -180, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.wrapDegrees(270), -90, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.wrapDegrees(-270), 90, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.wrapDegrees(360), 0, accuracy: 1e-9)
    }

    func testTurnError() {
        XCTAssertEqual(GeoMath.turnErrorDegrees(0, 90), 90, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.turnErrorDegrees(0, -90), -90, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.turnErrorDegrees(350, 10), 20, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.turnErrorDegrees(10, 350), -20, accuracy: 1e-9)
    }

    func testDestinationRoundTrip() {
        let dest = GeoMath.destinationPoint(48.0, 11.0, 45, 1000)
        let d = GeoMath.distanceMeters(48.0, 11.0, dest.latitude, dest.longitude)
        XCTAssertEqual(d, 1000, accuracy: 1)
        let b = GeoMath.bearingDegrees(48.0, 11.0, dest.latitude, dest.longitude)
        XCTAssertEqual(b, 45, accuracy: 0.1)
    }

    func testNorthEastOffset() {
        // Due north of the start is pure +north.
        // Hướng bắc đúng từ điểm xuất phát là độ lệch thuần +north.
        let ne = GeoMath.northEastOffset(fromLat: 0, fromLon: 0, toLat: 0.001, toLon: 0)
        XCTAssertEqual(ne.north, 111.19, accuracy: 1)
        XCTAssertEqual(ne.east, 0, accuracy: 1)
    }

    // MARK: - Launch frame (PROTOCOL.md §6)

    /// The pinned rotation. +x = nose at ALIGN (bearing alpha), +y = 90° LEFT.
    /// Phép quay được ghim cố định. +x = hướng mũi drone tại ALIGN
    /// (phương hướng alpha), +y = 90° TRÁI.
    func testLaunchFrameRotation() {
        // Alpha 0: nose points north. A pure-north offset is pure +x; a pure
        // east offset is pure −y (east is 90° RIGHT, so it is −y).
        // Alpha 0: mũi hướng bắc. Độ lệch thuần bắc là thuần +x; độ lệch thuần
        // đông là thuần −y (đông là 90° PHẢI, nên là −y).
        let northTarget = LaunchFrame.toLaunchFrame(north: 10, east: 0, alphaDegrees: 0)
        XCTAssertEqual(northTarget.x, 10, accuracy: 1e-9)
        XCTAssertEqual(northTarget.y, 0, accuracy: 1e-9)

        let eastTarget = LaunchFrame.toLaunchFrame(north: 0, east: 10, alphaDegrees: 0)
        XCTAssertEqual(eastTarget.x, 0, accuracy: 1e-9)
        XCTAssertEqual(eastTarget.y, -10, accuracy: 1e-9)
    }

    func testLaunchFrameAlpha90() {
        // Alpha 90: nose points east. A pure-east offset is pure +x.
        // Alpha 90: mũi hướng đông. Độ lệch thuần đông là thuần +x.
        let t = LaunchFrame.toLaunchFrame(north: 0, east: 10, alphaDegrees: 90)
        XCTAssertEqual(t.x, 10, accuracy: 1e-9)
        XCTAssertEqual(t.y, 0, accuracy: 1e-9)

        // A pure-north offset is now +y (north is 90° LEFT of east).
        // Độ lệch thuần bắc giờ là +y (bắc ở bên TRÁI đông 90°).
        let t2 = LaunchFrame.toLaunchFrame(north: 10, east: 0, alphaDegrees: 90)
        XCTAssertEqual(t2.x, 0, accuracy: 1e-9)
        XCTAssertEqual(t2.y, 10, accuracy: 1e-9)
    }

    func testLaunchFrameRoundTrip() {
        let frame = LaunchFrame.toLaunchFrame(north: 3, east: -4, alphaDegrees: 37)
        let back = LaunchFrame.toNorthEast(x: frame.x, y: frame.y, alphaDegrees: 37)
        XCTAssertEqual(back.north, 3, accuracy: 1e-9)
        XCTAssertEqual(back.east, -4, accuracy: 1e-9)
    }

    // MARK: - Mission

    func testWaypointAcceptanceFloor() {
        let wp = Waypoint(latitude: 0, longitude: 0, acceptanceRadiusMeters: 0.01)
        XCTAssertEqual(wp.acceptanceRadiusMeters, 0.5)
    }

    func testDefaultAcceptanceRadius() {
        // PROTOCOL.md §10 pins 2.5 m.
        // PROTOCOL.md §10 ghim cố định 2.5 m.
        XCTAssertEqual(Waypoint.defaultAcceptanceRadiusMeters, 2.5, accuracy: 1e-9)
    }

    func testMissionAddRemoveReorder() {
        let mission = Mission()
        mission.addWaypoint(Waypoint(latitude: 1, longitude: 1, name: "A"))
        mission.addWaypoint(Waypoint(latitude: 2, longitude: 2, name: "B"))
        mission.addWaypoint(Waypoint(latitude: 3, longitude: 3, name: "C"))
        XCTAssertEqual(mission.count, 3)

        mission.moveWaypoint(from: 0, to: 2)
        XCTAssertEqual(mission.waypoint(at: 0)?.name, "B")
        XCTAssertEqual(mission.waypoint(at: 2)?.name, "A")

        mission.removeWaypoint(at: 1)
        XCTAssertEqual(mission.count, 2)
        XCTAssertEqual(mission.waypoint(at: 1)?.name, "A")
    }

    func testTotalDistance() {
        let mission = Mission()
        mission.setHome(0, 0)
        mission.addWaypoint(Waypoint(latitude: 0.001, longitude: 0))   // ~111 m north
                                                                     // ~111 m về phía bắc
        mission.addWaypoint(Waypoint(latitude: 0.002, longitude: 0))   // ~222 m north
                                                                     // ~222 m về phía bắc
        XCTAssertEqual(mission.totalDistanceMeters(), 222.4, accuracy: 2)
    }

    func testWaypointsFileRoundTrip() throws {
        let mission = Mission()
        mission.setHome(48.12345678, 11.87654321)
        mission.addWaypoint(Waypoint(latitude: 48.2, longitude: 11.9, name: "WP1"))
        mission.addWaypoint(Waypoint(latitude: 48.3, longitude: 11.95, name: "WP2"))

        let file = mission.toWaypointsFile()
        XCTAssertTrue(file.hasPrefix("QGC WPL 110"))

        let parsed = try Mission.parseWaypointsFile(file)
        XCTAssertEqual(parsed.homeLatitude ?? 0, 48.12345678, accuracy: 1e-7)
        XCTAssertEqual(parsed.homeLongitude ?? 0, 11.87654321, accuracy: 1e-7)
        XCTAssertEqual(parsed.count, 2)
        XCTAssertEqual(parsed.waypoint(at: 0)?.latitude ?? 0, 48.2, accuracy: 1e-7)
    }

    func testParseSkipsNonNavCommands() throws {
        let file = """
        QGC WPL 110
        0\t1\t3\t16\t0\t0\t0\t0\t48.0\t11.0\t0\t1
        1\t0\t3\t22\t0\t0\t0\t0\t0\t0\t10\t1
        2\t0\t3\t16\t0\t0\t0\t0\t48.1\t11.1\t0\t1
        """
        let mission = try Mission.parseWaypointsFile(file)
        XCTAssertEqual(mission.count, 1)
        XCTAssertEqual(mission.waypoint(at: 0)?.latitude ?? 0, 48.1, accuracy: 1e-9)
    }

    func testParseRejectsBadHeader() {
        XCTAssertThrowsError(try Mission.parseWaypointsFile("NOT A WPL FILE\n"))
    }

    // MARK: - Navigator

    private func makeNavigator() -> Navigator {
        let nav = Navigator()
        nav.align(launchLatitude: 0, launchLongitude: 0, alphaDegrees: 0)
        nav.markHome(0, 0)
        return nav
    }

    func testMissionArrivalAdvances() {
        let nav = makeNavigator()
        nav.mission.addWaypoint(Waypoint(latitude: 0.001, longitude: 0, name: "A"))
        nav.mission.addWaypoint(Waypoint(latitude: 0.002, longitude: 0, name: "B"))
        XCTAssertTrue(nav.startMission())
        XCTAssertEqual(nav.mode, .mission)

        // Drone is already at waypoint A (launch frame +x ≈ 111 m north).
        // Drone đã ở điểm hành trình A (khung cất cánh: +x ≈ 111 m về phía bắc).
        let status = nav.step(droneX: 111, droneY: 0,
                              phoneLatitude: 0, phoneLongitude: 0)
        XCTAssertTrue(status.arrived)
        XCTAssertEqual(nav.activeIndex, 1)

        // Now at waypoint B.
        // Giờ ở điểm hành trình B.
        let status2 = nav.step(droneX: 222, droneY: 0,
                               phoneLatitude: 0, phoneLongitude: 0)
        XCTAssertTrue(status2.arrived)
        // Last waypoint: mission ends.
        // Điểm hành trình cuối: sứ mệnh kết thúc.
        XCTAssertEqual(nav.mode, .idle)
    }

    func testGuidedIsOneShot() {
        let nav = makeNavigator()
        nav.startGuided(latitude: 0.001, longitude: 0)
        XCTAssertEqual(nav.mode, .guided)

        let status = nav.step(droneX: 111, droneY: 0,
                              phoneLatitude: 0, phoneLongitude: 0)
        XCTAssertTrue(status.arrived)
        XCTAssertEqual(nav.mode, .idle)
    }

    func testRTLReturnsHome() {
        let nav = makeNavigator()
        nav.mission.addWaypoint(Waypoint(latitude: 0.002, longitude: 0))
        nav.startMission()
        XCTAssertTrue(nav.startRTL())
        XCTAssertEqual(nav.mode, .rtl)

        // Home is the origin in the launch frame.
        // Home là gốc tọa độ trong khung cất cánh.
        let status = nav.step(droneX: 0, droneY: 0,
                              phoneLatitude: 0, phoneLongitude: 0)
        XCTAssertTrue(status.arrived)
        XCTAssertEqual(nav.mode, .idle)
    }

    func testPauseHolds() {
        let nav = makeNavigator()
        nav.mission.addWaypoint(Waypoint(latitude: 0.001, longitude: 0))
        nav.startMission()
        nav.pause()

        let status = nav.step(droneX: 0, droneY: 0,
                              phoneLatitude: 0, phoneLongitude: 0)
        XCTAssertNil(status.target)
        XCTAssertEqual(status.message, "paused")
    }

    func testBadAccuracyHolds() {
        let nav = makeNavigator()
        nav.mission.addWaypoint(Waypoint(latitude: 0.001, longitude: 0))
        nav.startMission()

        let status = nav.step(droneX: 0, droneY: 0,
                              phoneLatitude: 0, phoneLongitude: 0,
                              accuracyMeters: 25)
        XCTAssertNil(status.target)
        XCTAssertTrue(status.message.contains("holding"))
    }

    /// The target the navigator emits must be the launch-frame waypoint offset
    /// — the same frame `nav.pos_x/pos_y` is reported in. That is what lets
    /// arrival skip the lat/lon round trip.
    /// Mục tiêu mà Navigator phát ra phải là độ lệch của điểm hành trình
    /// trong khung cất cánh — đúng khung mà `nav.pos_x/pos_y` được báo cáo.
    /// Nhờ vậy mà bước kiểm tra đến nơi có thể bỏ qua lượt đi-về lat/lon.
    func testTargetMatchesLaunchFrame() {
        let nav = makeNavigator()
        nav.mission.addWaypoint(Waypoint(latitude: 0.001, longitude: 0.001))
        nav.startMission()

        let status = nav.step(droneX: 0, droneY: 0,
                              phoneLatitude: 0, phoneLongitude: 0)
        guard let target = status.target else {
            return XCTFail("expected a target")
        }
        // Alpha 0: target x = north, y = −east.
        // Alpha 0: mục tiêu x = bắc, y = −đông.
        let ne = GeoMath.northEastOffset(fromLat: 0, fromLon: 0,
                                         toLat: 0.001, toLon: 0.001)
        XCTAssertEqual(target.x, ne.north, accuracy: 1e-6)
        XCTAssertEqual(target.y, -ne.east, accuracy: 1e-6)
    }
}
