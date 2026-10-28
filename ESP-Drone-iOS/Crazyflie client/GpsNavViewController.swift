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
    private let navigator = Navigator()
    private var tapTarget: Waypoint?

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

        // Flight commands: icon above text so four fit across a phone.
        // Lệnh bay: biểu tượng nằm trên chữ để bốn nút vừa ngang một chiếc điện thoại.
        let rowFont = UIFont.systemFont(ofSize: 13, weight: .semibold)
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
            if !navigator.startMission() {
                flash("No waypoints to fly.")
            } else {
                // MISSION → drone FOLLOW (0x03).
                client.setFlightMode(.follow)
                flash("Mission started — FOLLOW.")
            }
        case 1:
            if let t = tapTarget {
                navigator.startGuided(latitude: t.latitude, longitude: t.longitude)
                client.setFlightMode(.follow)
                flash("GUIDED → FOLLOW.")
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
        if navigator.isPaused {
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
                               font: .systemFont(ofSize: 13, weight: .semibold), imageOnTop: true)
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
        tapTarget = nil
        refreshOverlays()
    }

    @objc private func mapTapped(_ gesture: UITapGestureRecognizer) {
        let point = gesture.location(in: mapView)
        let coordinate = mapView.convert(point, toCoordinateFrom: mapView)
        tapTarget = Waypoint(latitude: coordinate.latitude, longitude: coordinate.longitude, name: "Guided")
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
        refreshOverlays()
    }

    private func sendTargetIfNeeded() {
        guard navigator.isNavigating, !navigator.isPaused else { return }
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
            flash(String(format: "Loaded %d waypoints (%.0f m)",
                        mission.count, mission.totalDistanceMeters()))
            refreshOverlays()
        } catch {
            flash("Could not load file: \(error)")
        }
    }
}
