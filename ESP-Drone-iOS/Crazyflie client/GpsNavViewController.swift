//
//  GpsNavViewController.swift
//  Crazyflie client
//
//  GPS mission navigation screen (PROTOCOL.md §10). MapKit shows the plan and
//  the drone's estimated track; the phone runs the mission and the drone chases
//  one launch-frame offset at a time.
//  Màn hình dẫn đường sứ mệnh GPS (PROTOCOL.md §10). MapKit hiển thị kế hoạch
//  và đường bay ước tính của drone; điện thoại chạy sứ mệnh và drone bám theo
//  từng offset của khung cất cánh một.
//
//  The drone marker is labelled "estimated" — it is accelerometer dead
//  reckoning (§6), not GPS tracking, and NAV_FWD_SIGN / NAV_LEFT_SIGN are
//  still unverified hardware constants.
//  Marker của drone được gắn nhãn "estimated" — đó là tự dẫn đường quán tính
//  bằng gia tốc kế (§6), không phải theo dõi GPS, và NAV_FWD_SIGN / NAV_LEFT_SIGN
//  vẫn là các hằng số phần cứng chưa được kiểm chứng.
//

import UIKit
import MapKit
import CoreLocation
import UniformTypeIdentifiers

final class GpsNavViewController: UIViewController {

    private let client: CrtpClient
    private let telemetry: TelemetryStore

    private let mapView = MKMapView()
    private let statusLabel = UILabel()
    private let accuracyLabel = UILabel()
    private let modeControl = UISegmentedControl(items: ["MISSION", "GUIDED", "RTL"])
    private let armAlignButton = UIButton(type: .system)
    private let startButton = UIButton(type: .system)
    private let pauseButton = UIButton(type: .system)
    private let stopButton = UIButton(type: .system)
    private let loadFileButton = UIButton(type: .system)
    private let clearButton = UIButton(type: .system)
    /// Set by the Fly screen: opens another section after this screen closes.
    /// Do màn hình Fly đặt: mở một khu vực khác sau khi màn hình này đóng.
    var onSwitchSection: ((AppSection) -> Void)?
    private var sectionHeader: SectionHeader?

    private let locationManager = CLLocationManager()
    /// The plan is shared with the Indoor Map page (NavStore) so edits made
    /// there are the ones this screen flies.
    /// Kế hoạch được chia sẻ với trang Indoor Map (NavStore) để các chỉnh sửa
    /// làm ở đó chính là cái màn hình này bay.
    private let navigator = NavStore.shared.navigator

    // Map annotations (PROTOCOL.md §10) are drawn here read-only: the plan
    // must show the zones and corridors that veto legs and hold the drone, but
    // the Valetudo editing UI lives on the Indoor Map page.
    // Các chú thích bản đồ (PROTOCOL.md §10) được vẽ ở đây chỉ để xem: kế hoạch
    // phải hiện các vùng và hành lang phủ quyết chặng và giữ drone, nhưng giao
    // diện sửa của Valetudo nằm ở trang Indoor Map.

    /// TARGET is sent at ~5 Hz while FOLLOW/RTH/mission is active (§9).
    /// TARGET được gửi ở ~5 Hz khi FOLLOW/RTH/sứ mệnh đang hoạt động (§9).
    private var sendTimer: Timer?
    private var listener: ((LogSample) -> Void)?

    /// Current phone fix. Used only to define the plan and the launch frame.
    /// Fix hiện tại của điện thoại. Chỉ dùng để định nghĩa kế hoạch và khung cất cánh.
    private var phoneFix: CLLocation?
    private var alphaDegrees: Double = 0
    private var aligned = false

    // MARK: - Init

    init(client: CrtpClient, telemetry: TelemetryStore) {
        self.client = client
        self.telemetry = telemetry
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not supported")
    }

    deinit {
        sendTimer?.invalidate()
        if let listener = listener {
            telemetry.listeners.removeAll { $0 as AnyObject === listener as AnyObject }
        }
    }

    // MARK: - View

    /// Mission is a portrait screen (the map reads best upright, like the Android screen).
    /// Mission là màn hình dọc (bản đồ đọc tốt nhất khi thẳng đứng, như màn Android).
    override var supportedInterfaceOrientations: UIInterfaceOrientationMask { .portrait }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = "Mission"
        navigationItem.leftBarButtonItem = UIBarButtonItem(
            barButtonSystemItem: .done, target: self, action: #selector(close))
        // Shared header; CUT MOTORS moved up here from the bottom row (same place on
        // every screen). One tap on this screen, as on Android's Mission screen.
        // Header dùng chung; CUT MOTORS được chuyển lên đây từ hàng dưới cùng (cùng
        // vị trí trên mọi màn hình). Chạm một lần trên màn hình này, như màn hình
        // Mission trên Android.
        let header = SectionHeader(host: self, selected: .mission, telemetry: telemetry, twoTapCut: false) { [weak self] in
            self?.cutMotors()
        }
        header.leaveGuard = { [weak self] in
            guard let self = self, self.navigator.mode != .idle else { return nil }
            return "A mission is flying. End it, land, or use CUT MOTORS before switching."
        }
        header.prepareToLeave = { [weak self] in
            self?.stopForLeaving()
        }
        header.onSwitch = onSwitchSection
        header.install()
        sectionHeader = header

        mapView.translatesAutoresizingMaskIntoConstraints = false
        mapView.delegate = self
        mapView.showsUserLocation = true
        view.addSubview(mapView)

        let stack = UIStackView()
        stack.axis = .vertical
        stack.spacing = 8
        stack.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(stack)

        NSLayoutConstraint.activate([
            mapView.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            mapView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            mapView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            mapView.bottomAnchor.constraint(equalTo: stack.topAnchor, constant: -8),
            stack.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 12),
            stack.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -12),
            stack.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -12),
        ])

        statusLabel.numberOfLines = 0
        statusLabel.font = .monospacedSystemFont(ofSize: 12, weight: .regular)
        accuracyLabel.font = .systemFont(ofSize: 12)
        accuracyLabel.textColor = .secondaryLabel
        accuracyLabel.numberOfLines = 0

        modeControl.selectedSegmentIndex = 0

        // Flight commands: icon above text so four fit across a portrait phone
        // (11 pt is what "Start mission" needs to stay whole in a quarter width).
        // Lệnh bay: biểu tượng nằm trên chữ để bốn nút vừa ngang một chiếc điện thoại
        // dọc (11 pt là cỡ để "Start mission" còn nguyên trong một phần tư chiều ngang).
        let rowFont = UIFont.systemFont(ofSize: 11, weight: .semibold)
        armAlignButton.applyLabel("Arm + ALIGN", symbol: "airplane.departure", font: rowFont, imageOnTop: true)
        armAlignButton.addTarget(self, action: #selector(armAndAlign), for: .touchUpInside)

        startButton.applyLabel("Start mission", symbol: "point.topleft.down.curvedto.point.bottomright.up",
                               font: rowFont, imageOnTop: true)
        startButton.addTarget(self, action: #selector(startNav), for: .touchUpInside)

        setPauseLabel(paused: false)
        pauseButton.addTarget(self, action: #selector(togglePause), for: .touchUpInside)

        // Was a red "Stop", which looked like the motor cut. It ends the mission and
        // hands control back to the sticks; red is kept for CUT MOTORS only.
        // Trước đây là nút "Stop" màu đỏ, dễ nhầm với cắt động cơ. Nó kết thúc sứ
        // mệnh và trả quyền điều khiển về cần; màu đỏ chỉ dành cho CUT MOTORS.
        stopButton.applyLabel("End mission", symbol: "stop.fill", font: rowFont, imageOnTop: true)
        stopButton.addTarget(self, action: #selector(stopNav), for: .touchUpInside)

        // Routine tools: icon-only, words in the VoiceOver label.
        // Công cụ thường ngày: chỉ có biểu tượng, chữ nằm trong nhãn VoiceOver.
        loadFileButton.applyLabel(nil, symbol: "folder", accessibilityText: "Load waypoints file")
        loadFileButton.addTarget(self, action: #selector(loadFile), for: .touchUpInside)

        clearButton.applyLabel(nil, symbol: "trash", accessibilityText: "Clear plan")
        clearButton.addTarget(self, action: #selector(clearPlan), for: .touchUpInside)

        let row1 = UIStackView(arrangedSubviews: [armAlignButton, startButton, pauseButton, stopButton])
        row1.axis = .horizontal
        row1.distribution = .fillEqually
        for b in [loadFileButton, clearButton] {
            b.widthAnchor.constraint(equalToConstant: 52).isActive = true
        }
        let row2 = UIStackView(arrangedSubviews: [loadFileButton, clearButton, UIView()])
        row2.axis = .horizontal
        row2.distribution = .fill
        row2.spacing = 8
        row2.heightAnchor.constraint(equalToConstant: 52).isActive = true

        stack.addArrangedSubview(accuracyLabel)
        stack.addArrangedSubview(modeControl)
        stack.addArrangedSubview(row1)
        stack.addArrangedSubview(row2)
        stack.addArrangedSubview(statusLabel)

        // Tap the map to drop a GUIDED target.
        // Chạm vào bản đồ để đặt mục tiêu GUIDED.
        let tap = UITapGestureRecognizer(target: self, action: #selector(mapTapped(_:)))
        mapView.addGestureRecognizer(tap)

        // Long-press to add a mission waypoint.
        // Nhấn giữ để thêm điểm hành trình vào sứ mệnh.
        let longPress = UILongPressGestureRecognizer(target: self, action: #selector(mapLongPressed(_:)))
        mapView.addGestureRecognizer(longPress)

        // Location
        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.requestWhenInUseAuthorization()
        locationManager.startUpdatingLocation()
        locationManager.startUpdatingHeading()

        // Telemetry listener for nav.pos_* and flight state.
        // Listener dữ liệu từ xa cho nav.pos_* và trạng thái bay.
        let listener: (LogSample) -> Void = { [weak self] _ in
            self?.tickFromTelemetry()
        }
        self.listener = listener
        telemetry.listeners.append(listener)

        // TARGET sender at 5 Hz.
        // Bộ gửi TARGET ở 5 Hz.
        sendTimer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { [weak self] _ in
            self?.sendTargetIfNeeded()
        }

        statusLabel.text = "Drop waypoints (long-press) or tap for a GUIDED target.\n" +
            "The drone marker is ESTIMATED dead reckoning, not GPS."
        refreshOverlays()
    }

    // MARK: - Actions

    @objc private func close() {
        stopForLeaving()
        dismiss(animated: true)
    }

    private func stopForLeaving() {
        sendTimer?.invalidate()
        sendTimer = nil
        locationManager.stopUpdatingLocation()
        locationManager.stopUpdatingHeading()
    }

    @objc private func armAndAlign() {
        guard let fix = phoneFix else {
            flash("No phone GPS fix yet — cannot ALIGN.")
            return
        }
        // Compass bearing of the nose at launch. `locationManager.heading` is
        // magnetic; the drone's SET_HEADING is also a compass bearing (§6).
        // Phương hướng la bàn của mũi lúc cất cánh. `locationManager.heading` là
        // hướng từ; SET_HEADING của drone cũng là phương hướng la bàn (§6).
        let heading = locationManager.heading?.magneticHeading ?? 0
        alphaDegrees = heading
        navigator.align(launchLatitude: fix.coordinate.latitude,
                        launchLongitude: fix.coordinate.longitude,
                        alphaDegrees: heading)
        aligned = true
        navigator.markHome(fix.coordinate.latitude, fix.coordinate.longitude)

        client.arm()
        client.sendNavAlign(alphaDegrees: Float(heading))
        flash(String(format: "ALIGN α=%.1f°  launch %.5f,%.5f", heading,
                    fix.coordinate.latitude, fix.coordinate.longitude))
        refreshOverlays()
    }

    @objc private func startNav() {
        guard aligned else {
            flash("Arm + ALIGN first.")
            return
        }
        switch modeControl.selectedSegmentIndex {
        case 0:
            // The annotations veto the plan before it flies (§10).
            // Các chú thích phủ quyết kế hoạch trước khi bay (§10).
            if let refusal = navigator.checkMission() {
                flash("Cannot start mission: \(refusal)")
            } else if !navigator.startMission() {
                flash("No waypoints to fly.")
            } else {
                // MISSION → drone FOLLOW (0x03).
                client.setFlightMode(.follow)
                flash("Mission started — FOLLOW.")
            }
        case 1:
            if let t = NavStore.shared.tapTarget {
                if let refusal = navigator.checkWaypoint(t.latitude, t.longitude) {
                    flash("Cannot go there: \(refusal)")
                } else {
                    navigator.startGuided(latitude: t.latitude, longitude: t.longitude)
                    client.setFlightMode(.follow)
                    flash("GUIDED → FOLLOW.")
                }
            } else {
                flash("Tap the map for a GUIDED target first.")
            }
        case 2:
            if navigator.startRTL() {
                // RTL → drone RTH (0x04).
                client.setFlightMode(.rth)
                flash("RTL → RTH.")
            } else {
                flash("No home recorded — Arm + ALIGN first.")
            }
        default:
            break
        }
        refreshOverlays()
    }

    @objc private func togglePause() {
        if navigator.isPaused || navigator.isZoneHold {
            // Resume doubles as the pilot's keep-out override.
            // Nút Tiếp tục đồng thời là quyền ghi đè vùng cấm bay của người lái.
            navigator.resume()
            setPauseLabel(paused: false)
            flash("Resumed.")
        } else {
            navigator.pause()
            setPauseLabel(paused: true)
            flash("Paused — drone holds after 3 s.")
        }
    }

    @objc private func stopNav() {
        navigator.stop()
        client.setFlightMode(.manual)
        flash("Mission ended — back to manual sticks.")
        refreshOverlays()
    }

    @objc private func cutMotors() {
        navigator.stop()
        client.emergency()
        UINotificationFeedbackGenerator().notificationOccurred(.error)
        flash("CUT MOTORS — motors off.")
        refreshOverlays()
    }

    /// The icon shows what a tap will do, like the text.
    /// Biểu tượng cho biết chạm vào sẽ làm gì, giống như chữ.
    private func setPauseLabel(paused: Bool) {
        pauseButton.applyLabel(paused ? "Resume" : "Pause", symbol: paused ? "play.fill" : "pause.fill",
                               font: .systemFont(ofSize: 11, weight: .semibold), imageOnTop: true)
    }

    @objc private func loadFile() {
        // `asCopy: true` is the old `.import` mode: take a copy of the file
        // rather than hold security-scoped access to the original.
        // `asCopy: true` là chế độ `.import` cũ: lấy một bản sao của file
        // thay vì giữ quyền truy cập phạm vi bảo mật vào file gốc.
        let picker = UIDocumentPickerViewController(forOpeningContentTypes: [.plainText],
                                                    asCopy: true)
        picker.delegate = self
        present(picker, animated: true)
    }

    @objc private func clearPlan() {
        navigator.mission.clearWaypoints()
        NavStore.shared.tapTarget = nil
        refreshOverlays()
    }

    @objc private func mapTapped(_ gesture: UITapGestureRecognizer) {
        let point = gesture.location(in: mapView)
        let coordinate = mapView.convert(point, toCoordinateFrom: mapView)

        // Keep-out zones veto the target outright (§10).
        // Vùng cấm bay phủ quyết mục tiêu ngay (§10).
        if let refusal = navigator.checkWaypoint(coordinate.latitude, coordinate.longitude) {
            flash("Cannot set target: \(refusal)")
            return
        }

        NavStore.shared.tapTarget = Waypoint(latitude: coordinate.latitude,
                                             longitude: coordinate.longitude, name: "Guided")
        // Drop a pin so the user sees what they tapped.
        // Ghim một điểm để người dùng thấy chỗ mình vừa chạm.
        mapView.removeAnnotations(mapView.annotations.filter { $0.title ?? "" == "Guided" })
        let ann = MKPointAnnotation()
        ann.title = "Guided"
        ann.coordinate = coordinate
        mapView.addAnnotation(ann)
        flash("GUIDED target set. Tap Start to fly there.")
    }

    @objc private func mapLongPressed(_ gesture: UILongPressGestureRecognizer) {
        guard gesture.state == .began else { return }
        let point = gesture.location(in: mapView)
        let coordinate = mapView.convert(point, toCoordinateFrom: mapView)
        if let refusal = navigator.checkWaypoint(coordinate.latitude, coordinate.longitude) {
            flash("Cannot add waypoint: \(refusal)")
            return
        }
        let wp = Waypoint(latitude: coordinate.latitude, longitude: coordinate.longitude,
                          name: "WP\(navigator.mission.count + 1)")
        navigator.mission.addWaypoint(wp)
        flash(String(format: "Added %@" , wp.name))
        refreshOverlays()
    }

    private func flash(_ text: String) {
        statusLabel.text = text
    }

    // MARK: - Nav loop

    private func tickFromTelemetry() {
        // Drone position from the nav log block (ids 15/16), launch frame.
        // Vị trí drone từ khối log điều hướng (ids 15/16), theo khung cất cánh.
        let droneX = telemetry.value("nav.pos_x") ?? 0
        let droneY = telemetry.value("nav.pos_y") ?? 0
        guard let fix = phoneFix else { return }

        let status = navigator.step(
            droneX: droneX, droneY: droneY,
            phoneLatitude: fix.coordinate.latitude,
            phoneLongitude: fix.coordinate.longitude,
            accuracyMeters: fix.horizontalAccuracy
        )
        statusLabel.text = status.message.isEmpty
            ? String(format: "dist %.1f m  bearing %.0f°",
                     status.distanceMeters ?? 0, status.bearingDegrees ?? 0)
            : status.message
        // A keep-out hold reads as "Resume" on the pause button too.
        // Trạng thái giữ trong vùng cấm bay cũng hiển thị "Resume" trên nút tạm dừng.
        setPauseLabel(paused: navigator.isPaused || navigator.isZoneHold)
        refreshOverlays()
    }

    private func sendTargetIfNeeded() {
        // A keep-out hold suppresses TARGET the same way a pause does (§10).
        // Việc giữ trong vùng cấm bay ngưng TARGET giống như khi tạm dừng (§10).
        guard navigator.isNavigating, !navigator.isPaused, !navigator.isZoneHold else { return }
        guard let target = navigator.currentTarget() else { return }
        // TARGET is a north/east offset from launch (§6) — the firmware does
        // the launch-frame rotation. ~5 Hz is the keepalive; the drone holds
        // after 3 s of silence.
        // TARGET là offset bắc/đông từ điểm cất cánh (§6) — firmware làm phép
        // xoay sang khung cất cánh. ~5 Hz là tín hiệu giữ kết nối; drone giữ
        // nguyên sau 3 giây im lặng.
        guard phoneFix != nil else { return }
        let ne = GeoMath.northEastOffset(
            fromLat: navigator.launchLatitude, fromLon: navigator.launchLongitude,
            toLat: target.latitude, toLon: target.longitude
        )
        client.sendNavTarget(north: Float(ne.north), east: Float(ne.east))
    }

    // MARK: - Overlays

    private func refreshOverlays() {
        mapView.removeOverlays(mapView.overlays)

        // Mission legs.
        // Các chặng của sứ mệnh.
        let mission = navigator.mission
        var coords: [CLLocationCoordinate2D] = []
        if let lat = mission.homeLatitude, let lon = mission.homeLongitude {
            coords.append(CLLocationCoordinate2D(latitude: lat, longitude: lon))
        }
        for wp in mission.waypoints {
            coords.append(CLLocationCoordinate2D(latitude: wp.latitude, longitude: wp.longitude))
        }
        if coords.count > 1 {
            let polyline = MKPolyline(coordinates: &coords, count: coords.count)
            mapView.addOverlay(polyline)
        }

        // Map annotations (§10): zones and corridors as their own overlays, the
        // tapped one highlighted.
        // Chú thích bản đồ (§10): các vùng và hành lang dưới dạng overlay riêng,
        // cái được chạm được làm nổi bật.
        refreshAnnotationOverlays()

        // Waypoint annotations.
        // Các annotation điểm hành trình.
        let existing = mapView.annotations.filter { !($0 is MKUserLocation) }
        mapView.removeAnnotations(existing)
        if let lat = mission.homeLatitude, let lon = mission.homeLongitude {
            let ann = MKPointAnnotation()
            ann.title = "Home"
            ann.coordinate = CLLocationCoordinate2D(latitude: lat, longitude: lon)
            mapView.addAnnotation(ann)
        }
        for (i, wp) in mission.waypoints.enumerated() {
            let ann = MKPointAnnotation()
            ann.title = wp.name.isEmpty ? "WP\(i + 1)" : wp.name
            ann.coordinate = CLLocationCoordinate2D(latitude: wp.latitude, longitude: wp.longitude)
            mapView.addAnnotation(ann)
        }

        // Drone estimated marker. We only have the launch-frame position, so
        // convert it back to lat/lon via the ALIGN frame. Label is "estimated".
        // Marker ước tính của drone. Ta chỉ có vị trí theo khung cất cánh, nên
        // quy ngược về vĩ/kinh độ qua khung ALIGN. Nhãn là "estimated".
        if aligned, let fix = phoneFix {
            let x = telemetry.value("nav.pos_x") ?? 0
            let y = telemetry.value("nav.pos_y") ?? 0
            let ne = LaunchFrame.toNorthEast(x: x, y: y, alphaDegrees: alphaDegrees)
            let dest = GeoMath.destinationPoint(
                fix.coordinate.latitude, fix.coordinate.longitude,
                0, ne.north
            )
            // east offset from the north-shifted point
            // offset về phía đông từ điểm đã dịch về phía bắc
            let final = GeoMath.destinationPoint(
                dest.latitude, dest.longitude,
                90, ne.east
            )
            let droneAnn = MKPointAnnotation()
            droneAnn.title = "Drone (estimated)"
            droneAnn.coordinate = CLLocationCoordinate2D(latitude: final.latitude, longitude: final.longitude)
            mapView.addAnnotation(droneAnn)

            accuracyLabel.text = String(
                format: "Drone marker is ESTIMATED dead reckoning (not GPS). NAV_FWD_SIGN/NAV_LEFT_SIGN unverified — expect to flip one after first flight."
            )
        }
    }

    /// Replaces only the annotation shapes — the pins and the mission polyline
    /// stay put so a handle drag does not flicker.
    /// Chỉ thay các hình chú thích — các ghim và đường sứ mệnh đứng yên để thao
    /// tác kéo tay nắm không bị nhấp nháy.
    private func refreshAnnotationOverlays() {
        let shapes = mapView.overlays.compactMap { $0 as? MapAnnotationShape }
        if !shapes.isEmpty {
            mapView.removeOverlays(shapes)
        }
        for annotation in navigator.mission.annotations {
            // Read-only here: no selection, so no editing handles are drawn.
            // Chỉ để xem ở đây: không có lựa chọn nên không vẽ tay nắm sửa.
            mapView.addOverlay(MapAnnotationShape(annotation: annotation, selected: false))
        }
    }
}

// MARK: - MKMapViewDelegate

extension GpsNavViewController: MKMapViewDelegate {
    func mapView(_ mapView: MKMapView, rendererFor overlay: MKOverlay) -> MKOverlayRenderer {
        if let polyline = overlay as? MKPolyline {
            let r = MKPolylineRenderer(polyline: polyline)
            r.strokeColor = UIColor.systemBlue
            r.lineWidth = 3
            return r
        }
        if let shape = overlay as? MapAnnotationShape {
            return MapAnnotationRenderer(overlay: shape)
        }
        return MKOverlayRenderer(overlay: overlay)
    }

    func mapView(_ mapView: MKMapView, viewFor annotation: MKAnnotation) -> MKAnnotationView? {
        if annotation is MKUserLocation { return nil }
        let id = "pin"
        let view = mapView.dequeueReusableAnnotationView(withIdentifier: id) as? MKMarkerAnnotationView
            ?? MKMarkerAnnotationView(annotation: annotation, reuseIdentifier: id)
        view.annotation = annotation
        if annotation.title ?? "" == "Drone (estimated)" {
            view.markerTintColor = .systemOrange
            view.displayPriority = .required
        } else if annotation.title ?? "" == "Guided" {
            view.markerTintColor = .systemPurple
        } else {
            view.markerTintColor = .systemBlue
        }
        return view
    }
}

// MARK: - CLLocationManagerDelegate

extension GpsNavViewController: CLLocationManagerDelegate {
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let fix = locations.last else { return }
        // Ignore fixes worse than ~15 m — navigating on a bad fix is worse
        // than waiting (PROTOCOL.md §10).
        // Bỏ qua fix tệ hơn ~15 m — dẫn đường trên fix tệ còn tệ hơn là chờ
        // (PROTOCOL.md §10).
        if fix.horizontalAccuracy > Navigator.maxPositionAccuracyMeters {
            accuracyLabel.text = String(
                format: "GPS accuracy %.1f m, holding — waiting for a better fix.",
                fix.horizontalAccuracy
            )
            return
        }
        phoneFix = fix
        accuracyLabel.text = String(format: "GPS accuracy %.1f m", fix.horizontalAccuracy)
    }

    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        // Kept for the ALIGN capture; no continuous use — the drone holds its
        // own heading after SET_HEADING.
        // Giữ lại để bắt hướng lúc ALIGN; không dùng liên tục — drone tự giữ
        // hướng mũi sau SET_HEADING.
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        accuracyLabel.text = "Location error: \(error.localizedDescription)"
    }

    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        if manager.authorizationStatus == .authorizedWhenInUse
            || manager.authorizationStatus == .authorizedAlways {
            manager.startUpdatingLocation()
            manager.startUpdatingHeading()
        }
    }
}

// MARK: - UIDocumentPickerDelegate

extension GpsNavViewController: UIDocumentPickerDelegate {
    func documentPicker(_ controller: UIDocumentPickerViewController, didPickDocumentsAt urls: [URL]) {
        guard let url = urls.first else { return }
        do {
            let contents = try String(contentsOf: url, encoding: .utf8)
            let mission = try navigator.loadMissionFile(contents)
            // A loaded plan can violate the annotations; warn but keep it loaded.
            // Kế hoạch nạp vào có thể vi phạm các chú thích; cảnh báo nhưng vẫn giữ nó.
            if !mission.isEmpty, let refusal = navigator.checkMission() {
                flash("Loaded \(mission.count) waypoints — warning: \(refusal)")
            } else {
                flash(String(format: "Loaded %d waypoints (%.0f m)",
                            mission.count, mission.totalDistanceMeters()))
            }
            refreshOverlays()
        } catch {
            flash("Could not load file: \(error)")
        }
    }
}

// MARK: - Map annotation overlays

/// One annotation as a MapKit overlay (PROTOCOL.md §10).
/// Shared with the Indoor Map page — it draws the same shapes with handles.
/// Một chú thích dưới dạng overlay của MapKit (PROTOCOL.md §10).
/// Dùng chung với trang Indoor Map — trang đó vẽ cùng các hình kèm tay nắm.
final class MapAnnotationShape: NSObject, MKOverlay {
    let annotation: MapAnnotation
    let selected: Bool

    init(annotation: MapAnnotation, selected: Bool) {
        self.annotation = annotation
        self.selected = selected
        super.init()
    }

    var coordinate: CLLocationCoordinate2D {
        let center = annotation.center()
        return CLLocationCoordinate2D(latitude: center.latitude, longitude: center.longitude)
    }

    var boundingMapRect: MKMapRect {
        var rect = MKMapRect.null
        for point in annotation.points {
            let mapPoint = MKMapPoint(CLLocationCoordinate2D(
                latitude: point.latitude, longitude: point.longitude))
            rect = rect.union(MKMapRect(x: mapPoint.x, y: mapPoint.y, width: 0, height: 0))
        }
        return rect
    }
}

/// The three annotation styles are ported from Valetudo's map client
/// structures: `ThresholdClientStructure` (grooved green bar) for corridors,
/// `NoGoAreaClientStructure` (red block) for keep-out zones and
/// `RampClientStructure` (black halo under a cyan stroke) plus
/// `CurtainClientStructure` (tapered sine waves) for wind zones.
/// Ba kiểu vẽ chú thích được chuyển thể từ các cấu trúc client bản đồ của
/// Valetudo: `ThresholdClientStructure` (thanh xanh lá có rãnh) cho hành lang,
/// `NoGoAreaClientStructure` (khối đỏ) cho vùng cấm bay và
/// `RampClientStructure` (viền đen dưới nét cyan) cùng
/// `CurtainClientStructure` (sóng sine thon dần ở hai đầu) cho vùng gió.
final class MapAnnotationRenderer: MKOverlayRenderer {

    override func draw(_ mapRect: MKMapRect, zoomScale: MKZoomScale, in context: CGContext) {
        guard let shape = overlay as? MapAnnotationShape else { return }
        let points = shape.annotation.points.map {
            point(for: MKMapPoint(CLLocationCoordinate2D(latitude: $0.latitude, longitude: $0.longitude)))
        }
        if shape.annotation.type == .corridor {
            drawCorridor(points, in: context)
        } else {
            drawArea(shape.annotation, points, selected: shape.selected, in: context)
        }
        // The active shape wears Valetudo's editing handles (drawRotatedIcon).
        // Hình đang active mang các tay nắm sửa của Valetudo (drawRotatedIcon).
        if shape.selected {
            drawHandles(shape.annotation, points, in: context)
        }
    }

    /// Valetudo's `*_zone.svg` handles — ✕ at p0, ✛ at p1 for a line, ↻ at p1 and
    /// ↖↘ at p2 for a quad — each rotated to the shape's angle.
    /// Các tay nắm `*_zone.svg` của Valetudo — ✕ tại p0, ✛ tại p1 với đường, ↻ tại
    /// p1 và ↖↘ tại p2 với khối tứ giác — mỗi cái xoay theo góc của hình.
    private func drawHandles(_ annotation: MapAnnotation, _ points: [CGPoint],
                             in context: CGContext) {
        guard points.count >= 2 else { return }
        let angle = atan2(points[1].y - points[0].y, points[1].x - points[0].x) * 180 / .pi
        var spots: [(kind: Int, point: CGPoint)] = [
            (ValetudoHandles.deleteKind, points[0]),
        ]
        if annotation.type.isArea && points.count >= 4 {
            spots.append((ValetudoHandles.rotateKind, points[1]))
            spots.append((ValetudoHandles.scaleKind, points[2]))
        } else {
            spots.append((ValetudoHandles.moveKind, points[1]))
        }
        for spot in spots {
            ValetudoHandles.draw(context, kind: spot.kind, at: spot.point,
                                 size: ValetudoHandles.size, angleDegrees: angle)
        }
    }

    /// Keep-out / wind areas, ported from Valetudo's `RampClientStructure`: black
    /// halo under a lw-5 border that goes solid + dashed [8,6] while the shape is
    /// active, over a 0.15 (active) / 0.10 fill.
    /// Vùng cấm bay / vùng gió, chuyển thể từ `RampClientStructure` của Valetudo:
    /// viền đen nằm dưới biên lw-5 chuyển thành đặc + đứt nét [8,6] khi hình đang
    /// active, trên nền 0.15 (active) / 0.10.
    private func drawArea(_ annotation: MapAnnotation, _ points: [CGPoint],
                          selected: Bool, in context: CGContext) {
        guard let first = points.first else { return }
        let path = CGMutablePath()
        path.move(to: first)
        for p in points.dropFirst() {
            path.addLine(to: p)
        }
        path.closeSubpath()

        let keepOut = annotation.type == .keepout
        // NoGoArea red rgb(239,68,68) / curtain cyan rgb(6,182,212).
        // Đỏ của NoGoArea rgb(239,68,68) / cyan của curtain rgb(6,182,212).
        let color = keepOut
            ? UIColor(red: 239 / 255, green: 68 / 255, blue: 68 / 255, alpha: 1)
            : UIColor(red: 6 / 255, green: 182 / 255, blue: 212 / 255, alpha: 1)

        context.setFillColor(color.withAlphaComponent(selected ? 0.15 : 0.10).cgColor)
        context.addPath(path)
        context.fillPath()

        // The halo lies under the coloured border (viền đen nằm dưới biên màu).
        context.setStrokeColor(UIColor(white: 0, alpha: 0.6).cgColor)
        context.setLineWidth(7)
        context.setLineDash(phase: 0, lengths: [])
        context.addPath(path)
        context.strokePath()

        context.setLineWidth(5)
        if selected {
            context.setStrokeColor(color.cgColor)
            context.setLineDash(phase: 0, lengths: [8, 6])
        } else {
            context.setStrokeColor(color.withAlphaComponent(0.7).cgColor)
            context.setLineDash(phase: 0, lengths: [])
        }
        context.addPath(path)
        context.strokePath()
        context.setLineDash(phase: 0, lengths: [])

        if !keepOut {
            drawCurtainWaves(points, clipPath: path, in: context)
        }
    }

    /// CurtainClientStructure's signature waves, tiled across a wind zone: sine
    /// strokes with Valetudo's per-end fade — smoothstep over 0.35× the wavelength
    /// at both ends — and the black halo under the cyan stroke.
    /// Những sóng đặc trưng của CurtainClientStructure, lát khắp vùng gió: các nét
    /// sine với độ mờ hai đầu của Valetudo — smoothstep trên 0.35× bước sóng ở mỗi
    /// đầu — và viền đen nằm dưới nét cyan.
    private func drawCurtainWaves(_ points: [CGPoint], clipPath: CGPath, in context: CGContext) {
        guard points.count >= 2 else { return }
        var left = CGFloat.greatestFiniteMagnitude
        var top = CGFloat.greatestFiniteMagnitude
        var right = -CGFloat.greatestFiniteMagnitude
        var bottom = -CGFloat.greatestFiniteMagnitude
        for p in points {
            left = min(left, p.x)
            top = min(top, p.y)
            right = max(right, p.x)
            bottom = max(bottom, p.y)
        }
        let waveLength: CGFloat = 24 // Valetudo's 6 map units at our scale (6 đơn vị bản đồ ở tỷ lệ của ta)
        let amplitude: CGFloat = 4 // ...and its 1 map unit of amplitude (...và 1 đơn vị biên độ)
        let rowGap: CGFloat = 12
        let fadeLength = waveLength * 0.35 // Valetudo's fadeLength (chiều dài độ mờ)
        let step: CGFloat = 2 // Valetudo's considerHiDPI(2)

        context.saveGState()
        context.addPath(clipPath)
        context.clip()

        let cyan = UIColor(red: 6 / 255, green: 182 / 255, blue: 212 / 255, alpha: 1)
        let halo = UIColor(white: 0, alpha: 0.6)
        let width = max(1, right - left)

        var y = top + rowGap / 2
        while y < bottom {
            let wave = CGMutablePath()
            var started = false
            var x: CGFloat = 0
            while x <= width + 0.5 {
                let rawSine = CGFloat(sin(Double(x / waveLength) * 2 * .pi))
                let distToEdge = min(x, width - x)
                let fadeFactor = max(0 as CGFloat, min(1, distToEdge / fadeLength))
                let smoothScale = fadeFactor * fadeFactor * (3 - 2 * fadeFactor)
                let wy = y + rawSine * amplitude * smoothScale
                let px = left + x
                if started {
                    wave.addLine(to: CGPoint(x: px, y: wy))
                } else {
                    wave.move(to: CGPoint(x: px, y: wy))
                    started = true
                }
                x += step
            }
            context.setLineCap(.round)
            context.setStrokeColor(halo.cgColor)
            context.setLineWidth(5)
            context.addPath(wave)
            context.strokePath()
            context.setStrokeColor(cyan.cgColor)
            context.setLineWidth(3)
            context.addPath(wave)
            context.strokePath()
            y += rowGap
        }
        context.restoreGState()
    }

    /// Corridors: ThresholdClientStructure's grooved green bar along the gate line
    /// — fill rgba(16,185,129,0.6), border rgb(16,185,129) and 8 grooves PARALLEL
    /// to the line in rgba(16,185,129,0.9).
    /// Hành lang: thanh xanh lá có rãnh của ThresholdClientStructure dọc theo
    /// đường cổng — nền rgba(16,185,129,0.6), viền rgb(16,185,129) và 8 rãnh
    /// SONG SONG với đường, màu rgba(16,185,129,0.9).
    private func drawCorridor(_ points: [CGPoint], in context: CGContext) {
        guard points.count >= 2 else { return }
        let p0 = points[0]
        let p1 = points[1]
        let dx = p1.x - p0.x
        let dy = p1.y - p0.y
        let length = (dx * dx + dy * dy).squareRoot()
        guard length >= 1 else { return }
        let nx = -dy / length
        let ny = dx / length
        let half: CGFloat = 4 // the bar's half width (nửa chiều rộng thanh)

        let bar = CGMutablePath()
        bar.move(to: CGPoint(x: p0.x + nx * half, y: p0.y + ny * half))
        bar.addLine(to: CGPoint(x: p1.x + nx * half, y: p1.y + ny * half))
        bar.addLine(to: CGPoint(x: p1.x - nx * half, y: p1.y - ny * half))
        bar.addLine(to: CGPoint(x: p0.x - nx * half, y: p0.y - ny * half))
        bar.closeSubpath()

        // ThresholdClientStructure: fill rgba(16,185,129,0.6), stroke rgb(16,185,129).
        context.setFillColor(UIColor(red: 16 / 255, green: 185 / 255, blue: 129 / 255, alpha: 0.6).cgColor)
        context.addPath(bar)
        context.fillPath()

        context.setStrokeColor(UIColor(red: 16 / 255, green: 185 / 255, blue: 129 / 255, alpha: 1).cgColor)
        context.setLineWidth(1)
        context.setLineDash(phase: 0, lengths: [])
        context.addPath(bar)
        context.strokePath()

        // 8 grooves running the length of the bar (8 rãnh chạy dọc thanh).
        context.setStrokeColor(UIColor(red: 16 / 255, green: 185 / 255, blue: 129 / 255, alpha: 0.9).cgColor)
        context.setLineWidth(max(1, 0.5))
        let step = (2 * half) / 9
        for i in 1...8 {
            let offset = -half + step * CGFloat(i)
            context.move(to: CGPoint(x: p0.x + nx * offset, y: p0.y + ny * offset))
            context.addLine(to: CGPoint(x: p1.x + nx * offset, y: p1.y + ny * offset))
        }
        context.strokePath()
    }
}

// MARK: - Valetudo icon set (SVG path data)

/// Minimal SVG path-data reader so the app can draw Valetudo's icon files
/// verbatim without bundling the SVGs: absolute and relative M/L/H/V/C/S/Q/T/A/Z,
/// implicit command repeats, and arc→cubic conversion for CGContext.
/// Trình đọc dữ liệu đường SVG tối giản để app vẽ nguyên các file biểu tượng của
/// Valetudo mà không cần đóng gói SVG: M/L/H/V/C/S/Q/T/A/Z tuyệt đối và tương
/// đối, lặp lệnh ngầm, và chuyển cung tròn thành cubic cho CGContext.
private enum SvgPathReader {

    static func makePath(_ data: String) -> CGPath {
        let chars = Array(data)
        let path = CGMutablePath()
        var i = 0
        // The last command seen; implicit repeats reuse it (M drops to L).
        // Lệnh vừa thấy; các lần lặp ngầm dùng lại nó (M tụt xuống L).
        var cmd: Character = "M"
        var cx: CGFloat = 0
        var cy: CGFloat = 0
        var sx: CGFloat = 0
        var sy: CGFloat = 0
        // Last control points for the S/T smooth commands.
        // Các điểm điều khiển cuối cho các lệnh mượt S/T.
        var lastC: CGPoint?
        var lastQ: CGPoint?

        func skipSeparators() {
            while i < chars.count {
                let c = chars[i]
                if c == " " || c == "," || c == "\n" || c == "\r" || c == "\t" {
                    i += 1
                } else {
                    break
                }
            }
        }

        func parseNumber() -> CGFloat? {
            skipSeparators()
            guard i < chars.count else { return nil }
            let start = i
            if chars[i] == "+" || chars[i] == "-" { i += 1 }
            var seenDigit = false
            while i < chars.count, chars[i].isNumber {
                i += 1
                seenDigit = true
            }
            if i < chars.count, chars[i] == "." {
                i += 1
                while i < chars.count, chars[i].isNumber {
                    i += 1
                    seenDigit = true
                }
            }
            if i < chars.count, chars[i] == "e" || chars[i] == "E" {
                var j = i + 1
                if j < chars.count, chars[j] == "+" || chars[j] == "-" { j += 1 }
                if j < chars.count, chars[j].isNumber {
                    i = j
                    while i < chars.count, chars[i].isNumber { i += 1 }
                }
            }
            guard seenDigit, let value = Double(String(chars[start..<i])) else { return nil }
            return CGFloat(value)
        }

        /// Arc flags are bare 0/1 and may abut the next number ("…11 3 3").
        /// Cờ cung là 0/1 trần và có thể dính liền số sau ("…11 3 3").
        func parseFlag() -> CGFloat? {
            skipSeparators()
            guard i < chars.count else { return nil }
            if chars[i] == "0" {
                i += 1
                return 0
            }
            if chars[i] == "1" {
                i += 1
                return 1
            }
            return nil
        }

        func hasArgument() -> Bool {
            var j = i
            while j < chars.count, " \t\r\n,".contains(chars[j]) { j += 1 }
            guard j < chars.count else { return false }
            let c = chars[j]
            return c.isNumber || c == "-" || c == "+" || c == "."
        }

        while i < chars.count {
            skipSeparators()
            guard i < chars.count else { break }
            if chars[i].isLetter {
                cmd = chars[i]
                i += 1
                if cmd == "Z" || cmd == "z" {
                    path.closeSubpath()
                    cx = sx
                    cy = sy
                    lastC = nil
                    lastQ = nil
                    continue
                }
            }
            let relative = cmd.isLowercase
            switch cmd.uppercased() {
            case "M":
                guard let x = parseNumber(), let y = parseNumber() else { i = chars.count; break }
                cx = relative ? cx + x : x
                cy = relative ? cy + y : y
                sx = cx
                sy = cy
                path.move(to: CGPoint(x: cx, y: cy))
                // Further coordinate pairs continue as lines (các cặp toạ độ tiếp theo nối thành line).
                cmd = relative ? "l" : "L"
                lastC = nil
                lastQ = nil
            case "L":
                guard let x = parseNumber(), let y = parseNumber() else { i = chars.count; break }
                cx = relative ? cx + x : x
                cy = relative ? cy + y : y
                path.addLine(to: CGPoint(x: cx, y: cy))
                lastC = nil
                lastQ = nil
            case "H":
                guard let x = parseNumber() else { i = chars.count; break }
                cx = relative ? cx + x : x
                path.addLine(to: CGPoint(x: cx, y: cy))
                lastC = nil
                lastQ = nil
            case "V":
                guard let y = parseNumber() else { i = chars.count; break }
                cy = relative ? cy + y : y
                path.addLine(to: CGPoint(x: cx, y: cy))
                lastC = nil
                lastQ = nil
            case "C":
                guard let x1 = parseNumber(), let y1 = parseNumber(),
                      let x2 = parseNumber(), let y2 = parseNumber(),
                      let x = parseNumber(), let y = parseNumber() else { i = chars.count; break }
                let c1 = CGPoint(x: relative ? cx + x1 : x1, y: relative ? cy + y1 : y1)
                let c2 = CGPoint(x: relative ? cx + x2 : x2, y: relative ? cy + y2 : y2)
                let end = CGPoint(x: relative ? cx + x : x, y: relative ? cy + y : y)
                path.addCurve(to: end, control1: c1, control2: c2)
                cx = end.x
                cy = end.y
                lastC = c2
                lastQ = nil
            case "S":
                guard let x2 = parseNumber(), let y2 = parseNumber(),
                      let x = parseNumber(), let y = parseNumber() else { i = chars.count; break }
                let c1: CGPoint
                if let last = lastC {
                    c1 = CGPoint(x: 2 * cx - last.x, y: 2 * cy - last.y)
                } else {
                    c1 = CGPoint(x: cx, y: cy)
                }
                let c2 = CGPoint(x: relative ? cx + x2 : x2, y: relative ? cy + y2 : y2)
                let end = CGPoint(x: relative ? cx + x : x, y: relative ? cy + y : y)
                path.addCurve(to: end, control1: c1, control2: c2)
                cx = end.x
                cy = end.y
                lastC = c2
                lastQ = nil
            case "Q":
                guard let x1 = parseNumber(), let y1 = parseNumber(),
                      let x = parseNumber(), let y = parseNumber() else { i = chars.count; break }
                let c = CGPoint(x: relative ? cx + x1 : x1, y: relative ? cy + y1 : y1)
                let end = CGPoint(x: relative ? cx + x : x, y: relative ? cy + y : y)
                path.addQuadCurve(to: end, control: c)
                cx = end.x
                cy = end.y
                lastQ = c
                lastC = nil
            case "T":
                guard let x = parseNumber(), let y = parseNumber() else { i = chars.count; break }
                let c: CGPoint
                if let last = lastQ {
                    c = CGPoint(x: 2 * cx - last.x, y: 2 * cy - last.y)
                } else {
                    c = CGPoint(x: cx, y: cy)
                }
                let end = CGPoint(x: relative ? cx + x : x, y: relative ? cy + y : y)
                path.addQuadCurve(to: end, control: c)
                cx = end.x
                cy = end.y
                lastQ = c
                lastC = nil
            case "A":
                guard let rx = parseNumber(), let ry = parseNumber(),
                      let rot = parseNumber(),
                      let largeFlag = parseFlag(), let sweepFlag = parseFlag(),
                      let x = parseNumber(), let y = parseNumber() else { i = chars.count; break }
                let end = CGPoint(x: relative ? cx + x : x, y: relative ? cy + y : y)
                addArc(path, from: CGPoint(x: cx, y: cy), to: end,
                       rx: rx, ry: ry, rotationDegrees: rot,
                       largeArc: largeFlag != 0, sweep: sweepFlag != 0)
                cx = end.x
                cy = end.y
                lastC = nil
                lastQ = nil
            default:
                i = chars.count
            }
            // A letter we cannot honour ends the parse (chữ không hiểu sẽ kết thúc).
            if !hasArgument() && i < chars.count && !chars[i].isLetter {
                i = chars.count
            }
        }
        return path
    }

    /// SVG's endpoint→centre arc parameterisation (spec appendix F.6), split into
    /// ≤90° cubic segments — the same conversion browsers do.
    /// Tham số hoá cung từ đầu-cuối về tâm của SVG (phụ lục F.6), chia thành các
    /// đoạn cubic ≤90° — đúng cách trình duyệt thực hiện.
    private static func addArc(_ path: CGMutablePath, from start: CGPoint, to end: CGPoint,
                               rx rxIn: CGFloat, ry ryIn: CGFloat, rotationDegrees: CGFloat,
                               largeArc: Bool, sweep: Bool) {
        if start.x == end.x && start.y == end.y { return }
        var rx = abs(rxIn)
        var ry = abs(ryIn)
        if rx == 0 || ry == 0 {
            path.addLine(to: end)
            return
        }
        let phi = rotationDegrees * .pi / 180
        let cosPhi = cos(phi)
        let sinPhi = sin(phi)
        let dx2 = (start.x - end.x) / 2
        let dy2 = (start.y - end.y) / 2
        let x1p = cosPhi * dx2 + sinPhi * dy2
        let y1p = -sinPhi * dx2 + cosPhi * dy2
        let lambda = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry)
        if lambda > 1 {
            let s = CGFloat(sqrt(Double(lambda)))
            rx *= s
            ry *= s
        }
        let rxSq = rx * rx
        let rySq = ry * ry
        let x1pSq = x1p * x1p
        let y1pSq = y1p * y1p
        let den = rxSq * y1pSq + rySq * x1pSq
        var num = rxSq * rySq - rxSq * y1pSq - rySq * x1pSq
        num = max(0, num)
        let sign: CGFloat = (largeArc != sweep) ? 1 : -1
        let coef = sign * CGFloat(sqrt(Double(num / den)))
        let cxp = coef * rx * y1p / ry
        let cyp = -coef * ry * x1p / rx
        let center = CGPoint(x: cosPhi * cxp - sinPhi * cyp + (start.x + end.x) / 2,
                             y: sinPhi * cxp + cosPhi * cyp + (start.y + end.y) / 2)

        func vectorAngle(_ ux: CGFloat, _ uy: CGFloat, _ vx: CGFloat, _ vy: CGFloat) -> CGFloat {
            let dot = ux * vx + uy * vy
            let len = (ux * ux + uy * uy).squareRoot() * (vx * vx + vy * vy).squareRoot()
            guard len > 0 else { return 0 }
            let cosA = max(-1 as CGFloat, min(1, dot / len))
            let angle = CGFloat(acos(Double(cosA)))
            return (ux * vy - uy * vx) < 0 ? -angle : angle
        }

        let theta1 = vectorAngle(1, 0, (x1p - cxp) / rx, (y1p - cyp) / ry)
        var delta = vectorAngle((x1p - cxp) / rx, (y1p - cyp) / ry,
                                (-x1p - cxp) / rx, (-y1p - cyp) / ry)
        if !sweep && delta > 0 {
            delta -= 2 * .pi
        } else if sweep && delta < 0 {
            delta += 2 * .pi
        }

        let segments = max(1, Int(ceil(Double(abs(delta) / (.pi / 2)))))
        let deltaSeg = delta / CGFloat(segments)
        let t = 8 / 3 * sin(deltaSeg / 4) * sin(deltaSeg / 4) / sin(deltaSeg / 2)

        func toUser(_ ex: CGFloat, _ ey: CGFloat) -> CGPoint {
            CGPoint(x: cosPhi * ex - sinPhi * ey + center.x,
                    y: sinPhi * ex + cosPhi * ey + center.y)
        }

        var th = theta1
        for _ in 0..<segments {
            let th2 = th + deltaSeg
            let cosTh = cos(th)
            let sinTh = sin(th)
            let cosTh2 = cos(th2)
            let sinTh2 = sin(th2)
            let e1 = CGPoint(x: rx * cosTh, y: ry * sinTh)
            let e2 = CGPoint(x: rx * cosTh2, y: ry * sinTh2)
            let ep1 = CGPoint(x: -rx * sinTh, y: ry * cosTh)
            let ep2 = CGPoint(x: -rx * sinTh2, y: ry * cosTh2)
            let c1 = toUser(e1.x + t * ep1.x, e1.y + t * ep1.y)
            let c2 = toUser(e2.x - t * ep2.x, e2.y - t * ep2.y)
            path.addCurve(to: toUser(e2.x, e2.y), control1: c1, control2: c2)
            th = th2
        }
    }
}

/// The on-map control handles of Valetudo's map-annotation editor: the red ✕ that
/// deletes, the grey ✛ that moves a line's endpoint, the ↻ that rotates a quad and
/// the ↖↘ that scales it. Ported 1:1 from `hypfer/valetudo`
/// `frontend/src/map/structures/icons/*_zone.svg` — same 32×32 viewBox, same
/// circle gradients, same glyph path data.
/// Các tay nắm điều khiển trên bản đồ của trình sửa chú thích Valetudo: ✕ đỏ để
/// xoá, ✛ xám để kéo đầu mút của đường, ↻ để xoay khối và ↖↘ để co giãn. Chuyển
/// thể 1:1 từ `hypfer/valetudo` `frontend/src/map/structures/icons/*_zone.svg` —
/// cùng khung nhìn 32×32, cùng gradient của vòng tròn, cùng dữ liệu đường vẽ.
enum ValetudoHandles {

    static let deleteKind = 0
    static let moveKind = 1
    static let rotateKind = 2
    static let scaleKind = 3

    static let size: CGFloat = 28
    static let hitScale: CGFloat = 0.85
    static let bodyToleranceMeters = 15.0
    static let dragSlop: CGFloat = 4

    /*
     * Glyph path data copied verbatim from the SVGs' <path d="..."> (32×32).
     * Dữ liệu đường vẽ chép nguyên từ <path d="..."> của các SVG (32×32).
     */
    private static let deleteGlyphData =
        "m9.4609 7.4492-2.0117 2.0117 6.5508 6.5508-6.5508 6.5488 2.0117 2.0117 6.5508-6.5488"
        + " 6.5488 6.5488 2.0117-2.0117-6.5488-6.5488 6.5488-6.5508-2.0117-2.0117-6.5488"
        + " 6.5508-6.5508-6.5508z"

    private static let moveGlyphData =
        "m16.01 2.8652-1.3555 2.3477-0.066406 0.11523-1.2891 2.2324h1.2891v7.0313h-7.0293"
        + "v-1.2871l-2.2324 1.2871h-0.0019531v0.001953l-0.11328 0.064453-2.3477 1.3555"
        + " 2.4609 1.4199v0.003906h0.0058593l2.2285 1.2871v-1.2871h7.0293v7.0215h-1.2871"
        + "l1.2871 2.2305v0.009766h0.005859l1.416 2.4531 1.3555-2.3457 0.0625-0.10742h0.003907"
        + "v-0.007813l1.2891-2.2324h-1.2891v-7.0215h7.0371v1.2871l4.6953-2.7109-2.3477-1.3555"
        + "-2.3477-1.3535v1.2871h-7.0371v-7.0313h1.2871l-1.2871-2.2305v-0.0019531l-1.4219-2.4629z"

    private static let rotateGlyphData =
        "m9.3027 7.1855-2.4102 1.4258h-0.0019531v0.0019531l-0.12305 0.070313-2.5352 1.5"
        + " 2.6582 1.5723v0.003907h0.0058594l2.4062 1.4238v-1.4238-0.18164a1.4 1.4 0 0 0"
        + " 0.09375 0.003906h2.6523c4.5179 0 8.1445 3.6964 8.1445 8.3828v2.7188a1.4 1.4 0 0 0"
        + " 0 0.035156h-0.13867-1.3906l1.3906 2.4688v0.009766h0.005859l1.5293 2.7148"
        + " 1.4629-2.5957 0.06836-0.11914h0.003906v-0.007813l1.3926-2.4707h-1.3926-0.13086a1.4"
        + " 1.4 0 0 0 0-0.035156v-2.7188c0-6.1527-4.8873-11.184-10.945-11.184h-2.6523a1.4 1.4"
        + " 0 0 0-0.09375 0.0039062v-0.17383-1.4258z"

    private static let scaleGlyphData =
        "m6.6523 6.6523 0.70536 2.6328 0.036447 0.13507-0.00214 0.0022 0.00429 0.0043"
        + " 0.66677 2.4934 0.91333-0.91333 12.028 12.03-0.91118 0.91118 2.4956 0.67106"
        + " 0.0022 0.0022 0.0022-0.0022 0.13293 0.03644 2.6328 0.70536-0.74181-2.7657"
        + " 0.0043-0.0043-0.0065-0.0065-0.66677-2.4913-0.91118 0.91118-12.03-12.028"
        + " 0.91118-0.91333-2.4913-0.66677-0.0043-0.00429-0.0022 0.00214z"

    private static let glyphs: [CGPath] = [
        SvgPathReader.makePath(deleteGlyphData),
        SvgPathReader.makePath(moveGlyphData),
        SvgPathReader.makePath(rotateGlyphData),
        SvgPathReader.makePath(scaleGlyphData),
    ]

    /// Screen positions of the selected annotation's handles, laid out exactly
    /// like Valetudo: the ✕ sits on p0, then the ✛ on p1 for a line, or the ↻ on p1
    /// and ↖↘ on p2 for a quad.
    /// Vị trí màn hình của các tay nắm trên chú thích đang chọn, bố trí đúng như
    /// Valetudo: ✕ tại p0, rồi ✛ tại p1 với đường, hoặc ↻ tại p1 và ↖↘ tại p2 với
    /// khối tứ giác.
    static func spots(for annotation: MapAnnotation,
                      project: (MapAnnotation.Point) -> CGPoint) -> [(kind: Int, point: CGPoint)] {
        guard annotation.points.count >= 2 else { return [] }
        var result: [(kind: Int, point: CGPoint)] = []
        result.append((deleteKind, project(annotation.points[0])))
        let p1 = project(annotation.points[1])
        if annotation.type.isArea && annotation.points.count >= 4 {
            result.append((rotateKind, p1))
            result.append((scaleKind, project(annotation.points[2])))
        } else {
            result.append((moveKind, p1))
        }
        return result
    }

    /// Draw one handle centred on `point`, `size` points across, rotated by
    /// `angleDegrees` exactly like Valetudo's `drawRotatedIcon`.
    /// Vẽ một tay nắm tại tâm `point`, bề ngang `size` điểm, xoay `angleDegrees`
    /// đúng như `drawRotatedIcon` của Valetudo.
    static func draw(_ context: CGContext, kind: Int, at point: CGPoint,
                     size: CGFloat, angleDegrees: CGFloat) {
        let scale = size / 32
        context.saveGState()
        context.translateBy(x: point.x, y: point.y)
        context.rotate(by: angleDegrees * .pi / 180)
        context.scaleBy(x: scale, y: scale)
        context.translateBy(x: -16, y: -16)

        let circle = CGPath(ellipseIn: CGRect(x: 16 - 14.946, y: 16 - 14.946,
                                              width: 29.892, height: 29.892),
                            transform: nil)

        // Circle gradients straight from the SVGs' <linearGradient> defs.
        // Gradient của vòng tròn lấy thẳng từ phần <linearGradient> của các SVG.
        let fillColors: [CGColor] = kind == deleteKind
            ? [UIColor(red: 0xF1 / 255, green: 0x5B / 255, blue: 0x5B / 255, alpha: 1).cgColor,
               UIColor(red: 0xEF / 255, green: 0x44 / 255, blue: 0x44 / 255, alpha: 1).cgColor]
            : [UIColor(red: 0x8C / 255, green: 0x8C / 255, blue: 0x8C / 255, alpha: 1).cgColor,
               UIColor(red: 0x80 / 255, green: 0x80 / 255, blue: 0x80 / 255, alpha: 1).cgColor]
        let fillGradient = CGGradient(colorsSpace: CGColorSpaceCreateDeviceRGB(),
                                      colors: fillColors as CFArray, locations: [0, 1])!
        let inkColors: [CGColor] = [
            UIColor(red: 0x40 / 255, green: 0x40 / 255, blue: 0x40 / 255, alpha: 1).cgColor,
            UIColor(red: 0x33 / 255, green: 0x33 / 255, blue: 0x33 / 255, alpha: 1).cgColor,
        ]
        let inkGradient = CGGradient(colorsSpace: CGColorSpaceCreateDeviceRGB(),
                                     colors: inkColors as CFArray, locations: [0, 1])!
        let circleStart = CGPoint(x: 27.32, y: 4.692)
        let circleEnd = CGPoint(x: 4.692, y: 27.32)

        context.saveGState()
        context.addPath(circle)
        context.clip()
        context.drawLinearGradient(fillGradient, start: circleStart, end: circleEnd, options: [])
        context.restoreGState()

        // The border is a gradient stroke: clip to the stroked annulus.
        // Viền là nét gradient: cắt theo vòng nhẫn đã nở nét.
        context.saveGState()
        context.addPath(circle)
        context.setLineWidth(2.1087)
        context.replacePathWithStrokedPath()
        context.clip()
        context.drawLinearGradient(inkGradient, start: circleStart, end: circleEnd, options: [])
        context.restoreGState()

        if glyphs.indices.contains(kind) {
            context.saveGState()
            context.addPath(glyphs[kind])
            context.clip()
            context.drawLinearGradient(inkGradient,
                                       start: CGPoint(x: 22.065, y: 9.9568),
                                       end: CGPoint(x: 9.9568, y: 22.065),
                                       options: [])
            context.restoreGState()
        }
        context.restoreGState()
    }
}

/// Valetudo's grouped-Add type icons (`CustomIcons.tsx`): a no-go circle, a
/// threshold step and a curtain drape, each already carrying its ⊕ "add" badge.
/// Biểu tượng kiểu của nút Thêm gộp nhóm Valetudo (`CustomIcons.tsx`): vòng tròn
/// cấm, bậc cổng và màn buông, mỗi cái đã mang sẵn huy hiệu ⊕ "thêm".
enum MapAnnotationAddIcons {

    private static let keepoutData =
        "M 12 2 C 6.4771635 2 2 6.4771635 2 12 C 2 17.522835 6.4771635 22 12 22"
        + " C 12.592362 22 13.17226 21.946433 13.736328 21.847656 A 6 6 0 0 1 13 19"
        + " A 6 6 0 0 1 16.623047 13.5 L 3.3457031 13.5 L 3.3457031 10.5 L 20.650391 10.503906"
        + " L 20.650391 13.232422 A 6 6 0 0 1 21.849609 13.720703"
        + " C 21.946622 13.161509 22 12.586934 22 12 C 22 6.4771635 17.522835 2 12 2 z"
        + " M 18 15 L 18 18 L 15 18 L 15 20 L 18 20 L 18 23 L 20 23 L 20 20 L 23 20"
        + " L 23 18 L 20 18 L 20 15 L 18 15 z"

    private static let corridorData =
        "m6.4453 1.5 2.2227 2.2227-7.668 7.6699 1.1152 1.1074 7.6699-7.6602 2.2148 2.2148"
        + "v-5.5547h-5.5547zm5.0547 7v6.5h-10.5v7.5h10.5 2.627a6 6 0 0 1-1.127-3.5"
        + " 6 6 0 0 1 6-6 6 6 0 0 1 4 1.5273v-6.0273h-11.5zm6.5 6.5v3h-3v2h3v3h2v-3h3v-2h-3v-3h-2z"

    private static let windData =
        "m1 1.5v2h22v-2h-22zm1 3v18h4c0-3-2-5-2-5 6-4 7-13 7-13h-9zm11 0s0.55061 4.9571"
        + " 3.2871 9.1484a6 6 0 0 1 2.7129-0.64844 6 6 0 0 1 3 0.80469v-9.3047h-9z"
        + "m5 10.5v3h-3v2h3v3h2v-3h3v-2h-3v-3h-2z"

    private static var cache: [Int: UIImage] = [:]

    /// 24×24 like Android's `ic_annotation_*.xml`, ink #424242.
    /// 24×24 như `ic_annotation_*.xml` trên Android, mực #424242.
    static func image(for type: MapAnnotation.AnnotationType) -> UIImage {
        if let hit = cache[type.rawValue] {
            return hit
        }
        let data: String
        switch type {
        case .keepout: data = keepoutData
        case .corridor: data = corridorData
        case .wind: data = windData
        }
        let path = SvgPathReader.makePath(data)
        let side: CGFloat = 24
        let image = UIGraphicsImageRenderer(size: CGSize(width: side, height: side)).image { ctx in
            let c = ctx.cgContext
            c.setFillColor(UIColor(red: 0x42 / 255, green: 0x42 / 255, blue: 0x42 / 255, alpha: 1).cgColor)
            c.addPath(path)
            c.fillPath()
        }
        cache[type.rawValue] = image
        return image
    }
}
