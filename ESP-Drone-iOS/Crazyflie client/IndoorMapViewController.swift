//
//  IndoorMapViewController.swift
//  Crazyflie client
//
//  Indoor Map page (PROTOCOL.md §10): the home of the Valetudo map-annotation
//  editor, laid out from `piotrmachowski/lovelace-xiaomi-vacuum-map-card`'s
//  render() — map with a selection overlay, then the modes bar (icon + name,
//  one active), the icons row (actions) and the tiles row (the current
//  selections as labelled tiles).
//  Trang Indoor Map (PROTOCOL.md §10): nơi ở của trình sửa chú thích bản đồ
//  Valetudo, bố cục theo render() của `piotrmachowski/lovelace-xiaomi-vacuum-map-card`
//  — bản đồ với lớp phủ lựa chọn, rồi thanh chế độ (biểu tượng + tên, một cái
//  đang bật), hàng biểu tượng (hành động) và hàng thẻ (các lựa chọn hiện tại
//  dưới dạng thẻ có nhãn).
//
//  The card's SelectionType mapping is kept: MANUAL_RECTANGLE draws a
//  drag-drawn rectangle (Keep-out / Wind zones), MANUAL_PATH is two taps
//  (fly-through Corridor), MANUAL_POINT is one tap (Go to) and Select edits
//  the shapes already on the map with Valetudo's own handles.
//  Ánh xạ SelectionType của thẻ được giữ nguyên: MANUAL_RECTANGLE vẽ hình chữ
//  nhật kéo thả (vùng Cấm bay / Gió), MANUAL_PATH là hai lần chạm (Hành lang
//  bay qua), MANUAL_POINT là một lần chạm (Đi tới) và Chọn sửa các hình đã có
//  trên bản đồ bằng tay nắm của chính Valetudo.
//
//  The plan (waypoints + annotations + guided target) is shared with the
//  Mission page through NavStore, so what gets edited here is what flies there.
//  Kế hoạch (điểm hành trình + chú thích + mục tiêu dẫn đường) dùng chung với
//  trang Mission qua NavStore, nên cái sửa ở đây chính là cái bay ở đó.
//

import UIKit
import MapKit
import CoreLocation

final class IndoorMapViewController: UIViewController {

    private let client: CrtpClient
    private let telemetry: TelemetryStore

    /// The shared plan (NavStore): edits land in the mission the runner flies.
    /// Kế hoạch dùng chung (NavStore): các chỉnh sửa rơi vào sứ mệnh mà bộ chạy bay.
    private let navigator = NavStore.shared.navigator

    /// Set by the Fly screen: opens another section after this screen closes.
    /// Do màn hình Fly đặt: mở một khu vực khác sau khi màn hình này đóng.
    var onSwitchSection: ((AppSection) -> Void)?
    private var sectionHeader: SectionHeader?

    private let mapView = MKMapView()
    private let modeScroll = UIScrollView()
    private let modeStack = UIStackView()
    private let iconsRow = UIStackView()
    private let tilesScroll = UIScrollView()
    private let tilesStack = UIStackView()
    private let statusLabel = UILabel()

    private let deleteButton = UIButton(type: .system)
    private let clearButton = UIButton(type: .system)
    private let centerButton = UIButton(type: .system)

    /// The card's `map_modes` bar: which interaction the map is in right now.
    /// Thanh `map_modes` của thẻ: thao tác nào đang áp cho bản đồ lúc này.
    private enum Mode: Int {
        case select, keepout, wind, corridor, goto
    }

    private var mode: Mode = .select
    private var modeButtons: [Mode: UIButton] = [:]

    /// The card's selection overlay state: which shape is active (gets handles).
    /// Trạng thái lớp phủ lựa chọn của thẻ: hình nào đang active (có tay nắm).
    private var selectedAnnotationIndex: Int?

    // MANUAL_RECTANGLE: a one-finger drag draws a rubber-band rectangle
    // (Valetudo drops zones at the viewport centre; a drag is the mobile
    // equivalent). Pinch still zooms: the gesture refuses a second finger.
    // MANUAL_RECTANGLE: kéo một ngón vẽ hình chữ nhật bằng dải cao su (Valetudo
    // thả vùng tại tâm khung nhìn; kéo thả là bản tương đương di động). Véo vẫn
    // phóng to: thao tác từ chối ngón thứ hai.
    private let rectGesture = UIPanGestureRecognizer()
    private let rubberBand = UIView()
    private var rectStartCoordinate: CLLocationCoordinate2D?
    private var rectStartPoint: CGPoint = .zero

    // MANUAL_PATH (Corridor): the first tap picks an endpoint, the second
    // completes the line.
    // MANUAL_PATH (Hành lang): lần chạm đầu chọn một đầu mút, lần hai hoàn tất.
    private var placeLat: Double?
    private var placeLon: Double?

    /// Handle/body editing drag in Select mode — Valetudo's translate branches,
    /// snapping on release (postProcess).
    /// Kéo sửa tay nắm/thân hình ở chế độ Chọn — các nhánh translate của Valetudo,
    /// hút lại khi nhả tay (postProcess).
    private let dragGesture = UIPanGestureRecognizer()

    /// What the current drag edits (nhánh thao tác kéo hiện tại).
    private enum DragMode {
        case none, deleteHandle, endpoint, body, rotate, scale
    }

    private var dragMode: DragMode = .none
    private var dragLast: CLLocationCoordinate2D?
    private var dragStart: CGPoint = .zero
    private var dragMoved = false

    private let locationManager = CLLocationManager()

    // MARK: - Init

    init(client: CrtpClient, telemetry: TelemetryStore) {
        self.client = client
        self.telemetry = telemetry
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not supported")
    }

    // MARK: - View

    /// Like the Mission screen: the map reads best upright.
    /// Như màn hình Mission: bản đồ đọc tốt nhất khi thẳng đứng.
    override var supportedInterfaceOrientations: UIInterfaceOrientationMask { .portrait }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = "Indoor Map"
        navigationItem.leftBarButtonItem = UIBarButtonItem(
            barButtonSystemItem: .done, target: self, action: #selector(close))

        // Shared header: status + section chips + CUT MOTORS, one tap on this
        // screen like Mission and Tune.
        // Header dùng chung: trạng thái + chip khu + CẮT ĐỘNG CƠ, chạm một lần
        // trên màn hình này như Mission và Tune.
        let header = SectionHeader(host: self, selected: .indoor, telemetry: telemetry,
                                   twoTapCut: false) { [weak self] in
            self?.cutMotors()
        }
        header.prepareToLeave = { [weak self] in
            self?.locationManager.stopUpdatingLocation()
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

        buildModeBar()
        buildIconsRow()
        buildTilesRow()

        statusLabel.numberOfLines = 0
        statusLabel.font = .monospacedSystemFont(ofSize: 12, weight: .regular)

        modeScroll.heightAnchor.constraint(equalToConstant: 44).isActive = true
        iconsRow.heightAnchor.constraint(equalToConstant: 52).isActive = true
        tilesScroll.heightAnchor.constraint(equalToConstant: 64).isActive = true

        stack.addArrangedSubview(modeScroll)
        stack.addArrangedSubview(iconsRow)
        stack.addArrangedSubview(tilesScroll)
        stack.addArrangedSubview(statusLabel)

        // The rubber band floats over the map while a MANUAL_RECTANGLE draws.
        // Dải cao su lơ lửng trên bản đồ khi đang vẽ MANUAL_RECTANGLE.
        rubberBand.backgroundColor = UIColor(red: 0x10 / 255, green: 0xB9 / 255,
                                             blue: 0x81 / 255, alpha: 0.15)
        rubberBand.layer.borderColor = UIColor(red: 0x10 / 255, green: 0xB9 / 255,
                                               blue: 0x81 / 255, alpha: 0.9).cgColor
        rubberBand.layer.borderWidth = 2
        rubberBand.isHidden = true
        rubberBand.isUserInteractionEnabled = false
        mapView.addSubview(rubberBand)

        // Tap: mode-dependent placement or selection.
        // Chạm: đặt chỗ theo chế độ hoặc chọn.
        let tap = UITapGestureRecognizer(target: self, action: #selector(mapTapped(_:)))
        mapView.addGestureRecognizer(tap)

        // Two pan gestures: the rectangle draw (placement modes) and the
        // handle/body edit (Select). The delegate only lets one begin when its
        // mode owns the touch, so the map still pans everywhere else.
        // Hai thao tác kéo: vẽ hình chữ nhật (các chế độ đặt) và sửa tay nắm/thân
        // hình (Chọn). Delegate chỉ cho một cái bắt đầu khi chế độ của nó sở hữu
        // cú chạm, nên bản đồ vẫn cuộn được ở mọi nơi khác.
        rectGesture.addTarget(self, action: #selector(rectDragged(_:)))
        rectGesture.delegate = self
        rectGesture.maximumNumberOfTouches = 1
        mapView.addGestureRecognizer(rectGesture)

        dragGesture.addTarget(self, action: #selector(mapPanned(_:)))
        dragGesture.delegate = self
        dragGesture.maximumNumberOfTouches = 1
        mapView.addGestureRecognizer(dragGesture)

        // Location: only for the blue dot and Center.
        // Vị trí: chỉ cho chấm xanh và nút Căn giữa.
        locationManager.delegate = self
        locationManager.requestWhenInUseAuthorization()
        locationManager.startUpdatingLocation()

        flash("Select: tap a zone to edit it. Keep-out / Wind: drag to draw.")
        refreshOverlays()
    }

    // MARK: - Mode bar (the card's map_modes: icon + name, one active)

    private func buildModeBar() {
        modeScroll.showsHorizontalScrollIndicator = false
        modeStack.axis = .horizontal
        modeStack.spacing = 8
        modeStack.translatesAutoresizingMaskIntoConstraints = false
        modeScroll.addSubview(modeStack)
        NSLayoutConstraint.activate([
            modeStack.topAnchor.constraint(equalTo: modeScroll.contentLayoutGuide.topAnchor),
            modeStack.bottomAnchor.constraint(equalTo: modeScroll.contentLayoutGuide.bottomAnchor),
            modeStack.leadingAnchor.constraint(equalTo: modeScroll.contentLayoutGuide.leadingAnchor),
            modeStack.trailingAnchor.constraint(equalTo: modeScroll.contentLayoutGuide.trailingAnchor),
            modeStack.heightAnchor.constraint(equalTo: modeScroll.frameLayoutGuide.heightAnchor),
        ])

        // Valetudo's own type glyphs (CustomIcons.tsx) for the drawing modes.
        // Biểu tượng kiểu của chính Valetudo (CustomIcons.tsx) cho các chế độ vẽ.
        let chips: [(Mode, String, UIImage?)] = [
            (.select, "Select", UIImage(systemName: "hand.tap")),
            (.keepout, "Keep-out", MapAnnotationAddIcons.image(for: .keepout)),
            (.wind, "Wind", MapAnnotationAddIcons.image(for: .wind)),
            (.corridor, "Corridor", MapAnnotationAddIcons.image(for: .corridor)),
            (.goto, "Go to", UIImage(systemName: "location.fill")),
        ]
        for (mode, title, image) in chips {
            let button = makeChip(title, image: image?.withRenderingMode(.alwaysTemplate))
            button.tag = mode.rawValue
            button.addTarget(self, action: #selector(modeTapped(_:)), for: .touchUpInside)
            modeStack.addArrangedSubview(button)
            modeButtons[mode] = button
        }
        updateModeChips()
    }

    private func makeChip(_ title: String, image: UIImage?) -> UIButton {
        let button = UIButton(type: .system)
        var config = UIButton.Configuration.filled()
        config.image = image
        config.imagePadding = 6
        config.contentInsets = NSDirectionalEdgeInsets(top: 0, leading: 12, bottom: 0, trailing: 12)
        var attributes = AttributeContainer()
        attributes.font = .systemFont(ofSize: 14, weight: .semibold)
        config.attributedTitle = AttributedString(title, attributes: attributes)
        button.configuration = config
        button.accessibilityLabel = title
        return button
    }

    @objc private func modeTapped(_ sender: UIButton) {
        guard let next = Mode(rawValue: sender.tag) else { return }
        mode = next
        // Placement never carries over into another mode.
        // Việc đặt chỗ không bao giờ kéo sang chế độ khác.
        cancelPlacement()
        updateModeChips()
        switch mode {
        case .select:
            flash("Select: tap a zone to edit it — drag its handles or body.")
        case .keepout:
            flash("Keep-out: drag on the map to draw the zone.")
        case .wind:
            flash("Wind: drag on the map to draw the zone.")
        case .corridor:
            flash("Corridor: tap the two endpoints.")
        case .goto:
            flash("Go to: tap the target point.")
        }
    }

    private func updateModeChips() {
        for (mode, button) in modeButtons {
            let active = mode == self.mode
            button.configuration?.baseBackgroundColor = active ? .systemBlue : .secondarySystemBackground
            button.configuration?.baseForegroundColor = active ? .white : .label
            button.accessibilityTraits = active ? [.button, .selected] : .button
        }
    }

    // MARK: - Icons row (the card's icons-wrapper: actions on the selection)

    private func buildIconsRow() {
        iconsRow.axis = .horizontal
        iconsRow.spacing = 8

        // Icon-only, words in the VoiceOver label — same as Mission's tools row.
        // Chỉ có biểu tượng, chữ nằm trong nhãn VoiceOver — như hàng công cụ của Mission.
        deleteButton.applyLabel(nil, symbol: "trash", accessibilityText: "Delete selected annotation")
        deleteButton.addTarget(self, action: #selector(deleteSelected), for: .touchUpInside)

        clearButton.applyLabel(nil, symbol: "eraser", accessibilityText: "Clear all annotations")
        clearButton.addTarget(self, action: #selector(clearAll), for: .touchUpInside)

        centerButton.applyLabel(nil, symbol: "scope", accessibilityText: "Center map")
        centerButton.addTarget(self, action: #selector(centerMap), for: .touchUpInside)

        for b in [deleteButton, clearButton, centerButton] {
            b.widthAnchor.constraint(equalToConstant: 52).isActive = true
            iconsRow.addArrangedSubview(b)
        }
        iconsRow.addArrangedSubview(UIView())
    }

    @objc private func deleteSelected() {
        guard let index = selectedAnnotationIndex else { return }
        let label = navigator.mission.annotation(at: index)?.type.label ?? "Annotation"
        navigator.mission.removeAnnotation(at: index)
        selectedAnnotationIndex = nil
        flash("\(label) deleted.")
        refreshOverlays()
    }

    @objc private func clearAll() {
        navigator.mission.clearAnnotations()
        selectedAnnotationIndex = nil
        flash("All annotations cleared.")
        refreshOverlays()
    }

    /// Center on the drone when one is shown, else the first object on the map.
    /// Căn giữa theo drone khi có, hoặc theo đối tượng đầu tiên trên bản đồ.
    @objc private func centerMap() {
        if navigator.launchLatitude != 0 || navigator.launchLongitude != 0,
           let fix = mapView.userLocation.location {
            let ne = LaunchFrame.toNorthEast(x: telemetry.value("nav.pos_x") ?? 0,
                                             y: telemetry.value("nav.pos_y") ?? 0,
                                             alphaDegrees: navigator.alphaDegrees)
            let north = GeoMath.destinationPoint(fix.coordinate.latitude, fix.coordinate.longitude,
                                                 0, ne.north)
            let east = GeoMath.destinationPoint(north.latitude, north.longitude, 90, ne.east)
            mapView.setCenter(CLLocationCoordinate2D(latitude: east.latitude, longitude: east.longitude),
                              animated: true)
            return
        }
        if let first = navigator.mission.annotations.first {
            let c = first.center()
            mapView.setCenter(CLLocationCoordinate2D(latitude: c.latitude, longitude: c.longitude),
                              animated: true)
        } else if let first = navigator.mission.waypoints.first {
            mapView.setCenter(CLLocationCoordinate2D(latitude: first.latitude, longitude: first.longitude),
                              animated: true)
        } else if let fix = mapView.userLocation.location {
            mapView.setCenter(fix.coordinate, animated: true)
        }
    }

    // MARK: - Tiles row (the card's tiles-wrapper: predefined selections)

    private func buildTilesRow() {
        tilesScroll.showsHorizontalScrollIndicator = false
        tilesStack.axis = .horizontal
        tilesStack.spacing = 8
        tilesStack.alignment = .center
        tilesStack.translatesAutoresizingMaskIntoConstraints = false
        tilesScroll.addSubview(tilesStack)
        NSLayoutConstraint.activate([
            tilesStack.topAnchor.constraint(equalTo: tilesScroll.contentLayoutGuide.topAnchor),
            tilesStack.bottomAnchor.constraint(equalTo: tilesScroll.contentLayoutGuide.bottomAnchor),
            tilesStack.leadingAnchor.constraint(equalTo: tilesScroll.contentLayoutGuide.leadingAnchor),
            tilesStack.trailingAnchor.constraint(equalTo: tilesScroll.contentLayoutGuide.trailingAnchor),
            tilesStack.heightAnchor.constraint(equalTo: tilesScroll.frameLayoutGuide.heightAnchor),
        ])
        refreshTiles()
    }

    /// One tile per annotation ("Keep-out 1" per type ordinal, with its type
    /// icon) and per waypoint ("WP 1"): tap selects and centres, exactly what
    /// the card's predefined-selection tiles do.
    /// Mỗi chú thích một thẻ ("Keep-out 1" theo số thứ tự trong kiểu, kèm biểu
    /// tượng kiểu) và mỗi điểm hành trình một thẻ ("WP 1"): chạm để chọn và căn
    /// giữa, đúng như các thẻ lựa chọn có sẵn của thẻ bản đồ.
    private func refreshTiles() {
        for view in tilesStack.arrangedSubviews {
            tilesStack.removeArrangedSubview(view)
            view.removeFromSuperview()
        }

        let mission = navigator.mission
        for type: MapAnnotation.AnnotationType in [.keepout, .corridor, .wind] {
            var ordinal = 0
            for (index, annotation) in mission.annotations.enumerated() where annotation.type == type {
                ordinal += 1
                let tile = makeTile(tileTitle(type, ordinal: ordinal),
                                    image: MapAnnotationAddIcons.image(for: type)) { [weak self] in
                    self?.selectAnnotation(index, center: true)
                }
                tilesStack.addArrangedSubview(tile)
            }
        }
        for (index, wp) in mission.waypoints.enumerated() {
            let name = wp.name.isEmpty ? "WP \(index + 1)" : wp.name
            let tile = makeTile(name, image: UIImage(systemName: "mappin")) { [weak self] in
                self?.mapView.setCenter(
                    CLLocationCoordinate2D(latitude: wp.latitude, longitude: wp.longitude),
                    animated: true)
            }
            tilesStack.addArrangedSubview(tile)
        }

        if tilesStack.arrangedSubviews.isEmpty {
            // The card shows the mode hint when nothing is selected yet.
            // Thẻ hiển thị gợi ý chế độ khi chưa có gì được chọn.
            let hint = UILabel()
            hint.text = "Draw a zone — pick a mode above"
            hint.font = .systemFont(ofSize: 13)
            hint.textColor = .secondaryLabel
            tilesStack.addArrangedSubview(hint)
        }
    }

    private func tileTitle(_ type: MapAnnotation.AnnotationType, ordinal: Int) -> String {
        let short: String
        switch type {
        case .keepout: short = "Keep-out"
        case .corridor: short = "Corridor"
        case .wind: short = "Wind"
        }
        return "\(short) \(ordinal)"
    }

    private func makeTile(_ title: String, image: UIImage?, action: @escaping () -> Void) -> UIButton {
        let button = UIButton(type: .system)
        var config = UIButton.Configuration.tinted()
        config.image = image
        config.imagePadding = 6
        config.contentInsets = NSDirectionalEdgeInsets(top: 0, leading: 10, bottom: 0, trailing: 10)
        var attributes = AttributeContainer()
        attributes.font = .systemFont(ofSize: 13, weight: .medium)
        config.attributedTitle = AttributedString(title, attributes: attributes)
        button.configuration = config
        button.accessibilityLabel = title
        button.addAction(UIAction { _ in action() }, for: .touchUpInside)
        return button
    }

    // MARK: - Map interaction

    @objc private func mapTapped(_ gesture: UITapGestureRecognizer) {
        let point = gesture.location(in: mapView)
        let coordinate = mapView.convert(point, toCoordinateFrom: mapView)

        switch mode {
        case .select:
            selectTapped(point: point, coordinate: coordinate)
        case .keepout, .wind:
            flash("Drag on the map to draw the zone.")
        case .corridor:
            corridorTapped(coordinate.latitude, coordinate.longitude)
        case .goto:
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
            flash("Go to target set — start GUIDED from the Mission page.")
        }
    }

    /// Select mode: Valetudo's handle hitboxes come first — a tap on the ✕
    /// deletes and the ✛ / ↻ / ↖↘ swallow the tap (drag-only affordances).
    /// Then a tap on a shape selects it and a tap elsewhere deselects.
    /// Chế độ Chọn: hitbox tay nắm của Valetudo được xét trước — chạm ✕ thì xoá
    /// còn ✛ / ↻ / ↖↘ nuốt cú chạm (chỉ là chỗ để kéo). Sau đó chạm vào hình để
    /// chọn nó và chạm chỗ khác để bỏ chọn.
    private func selectTapped(point: CGPoint, coordinate: CLLocationCoordinate2D) {
        if let index = selectedAnnotationIndex,
           let selected = navigator.mission.annotation(at: index) {
            let hitRadius = ValetudoHandles.size * ValetudoHandles.hitScale
            for spot in ValetudoHandles.spots(for: selected, project: projectView) {
                let dx = point.x - spot.point.x
                let dy = point.y - spot.point.y
                if dx * dx + dy * dy <= hitRadius * hitRadius {
                    if spot.kind == ValetudoHandles.deleteKind {
                        deleteSelected()
                    }
                    return
                }
            }
        }

        let toleranceMeters = 10.0
        for (index, annotation) in navigator.mission.annotations.enumerated()
        where annotation.nearPoint(coordinate.latitude, coordinate.longitude,
                                   toleranceMeters: toleranceMeters) {
            selectAnnotation(index, center: false)
            flash("\(annotation.type.label) selected — drag its handles to edit.")
            return
        }
        selectAnnotation(nil, center: false)
    }

    /// First tap picks a corridor endpoint, the second completes the line
    /// (MANUAL_PATH). The new shape is selected at once — Valetudo's add→active —
    /// so its handles appear immediately.
    /// Lần chạm đầu chọn một đầu mút của hành lang, lần hai hoàn tất đường
    /// (MANUAL_PATH). Hình mới được chọn ngay — add→active của Valetudo — để tay
    /// nắm hiện ra ngay.
    private func corridorTapped(_ latitude: Double, _ longitude: Double) {
        guard let firstLat = placeLat, let firstLon = placeLon else {
            placeLat = latitude
            placeLon = longitude
            flash("Corridor: tap the second endpoint.")
            return
        }
        let annotation = MapAnnotation.line(type: .corridor,
                                            firstLat, firstLon, latitude, longitude).snapped()
        navigator.mission.addAnnotation(annotation)
        placeLat = nil
        placeLon = nil
        selectedAnnotationIndex = navigator.mission.annotations.count - 1
        flash("Corridor placed — switch to Select to adjust it.")
        refreshOverlays()
    }

    private func selectAnnotation(_ index: Int?, center: Bool) {
        selectedAnnotationIndex = index
        if center, let index = index, let annotation = navigator.mission.annotation(at: index) {
            let c = annotation.center()
            mapView.setCenter(CLLocationCoordinate2D(latitude: c.latitude, longitude: c.longitude),
                              animated: true)
        }
        refreshAnnotationOverlays()
        refreshTiles()
        updateIconButtons()
    }

    private func cancelPlacement() {
        placeLat = nil
        placeLon = nil
        rectStartCoordinate = nil
        rubberBand.isHidden = true
    }

    // MARK: MANUAL_RECTANGLE drag (Keep-out / Wind zones)

    /// The drag draws a rubber-band rectangle; on release it becomes a
    /// 4-corner zone snapped like Valetudo's postProcess (5° and whole metres).
    /// Thao tác kéo vẽ hình chữ nhật bằng dải cao su; khi nhả tay nó thành vùng
    /// 4 góc được hút như postProcess của Valetudo (5° và mét nguyên).
    @objc private func rectDragged(_ gesture: UIPanGestureRecognizer) {
        let point = gesture.location(in: mapView)
        switch gesture.state {
        case .began:
            mapView.isScrollEnabled = false
            rectStartCoordinate = mapView.convert(point, toCoordinateFrom: mapView)
            rectStartPoint = point
            rubberBand.isHidden = false
            updateRubberBand(to: point)
        case .changed:
            guard rectStartCoordinate != nil else { return }
            updateRubberBand(to: point)
        case .ended:
            endRectPlacement(at: point)
        case .cancelled, .failed:
            // A second finger (pinch) cancels the draw and lets the map zoom.
            // Ngón thứ hai (véo) hủy nét vẽ và để bản đồ phóng to.
            cancelRectPlacement()
        default:
            break
        }
    }

    private func updateRubberBand(to point: CGPoint) {
        rubberBand.frame = CGRect(x: min(rectStartPoint.x, point.x),
                                  y: min(rectStartPoint.y, point.y),
                                  width: abs(point.x - rectStartPoint.x),
                                  height: abs(point.y - rectStartPoint.y))
    }

    private func endRectPlacement(at point: CGPoint) {
        defer { cancelRectPlacement() }
        guard let start = rectStartCoordinate else { return }
        // A stationary touch is a tap, not a zone.
        // Chạm đứng yên là một cú chạm, không phải vùng.
        guard hypot(point.x - rectStartPoint.x, point.y - rectStartPoint.y)
                > ValetudoHandles.dragSlop else { return }
        let end = mapView.convert(point, toCoordinateFrom: mapView)
        let type: MapAnnotation.AnnotationType = mode == .wind ? .wind : .keepout
        let annotation = MapAnnotation.rectFromCorners(type: type,
                                                       start.latitude, start.longitude,
                                                       end.latitude, end.longitude).snapped()
        navigator.mission.addAnnotation(annotation)
        selectedAnnotationIndex = navigator.mission.annotations.count - 1
        flash("\(type.label) placed — switch to Select to adjust it.")
        refreshOverlays()
    }

    private func cancelRectPlacement() {
        rectStartCoordinate = nil
        rubberBand.isHidden = true
        mapView.isScrollEnabled = true
    }

    // MARK: Handle editing drag (Valetudo's translate / postProcess branches)

    /// Screen position of an annotation vertex — the shared projection for the
    /// renderer and for hit-testing.
    /// Vị trí màn hình của một đỉnh chú thích — phép chiếu dùng chung cho phần
    /// vẽ và cho dò va chạm.
    private func projectView(_ point: MapAnnotation.Point) -> CGPoint {
        mapView.convert(CLLocationCoordinate2D(latitude: point.latitude,
                                               longitude: point.longitude),
                        toPointTo: mapView)
    }

    /// Starts a handle/body drag when the touch lands on one of Valetudo's
    /// control elements — ✕ delete at p0, ✛ move at p1 for lines, ↻ rotate at p1
    /// and ↖↘ scale at p2 for areas — or on the shape's body.
    /// Bắt đầu kéo tay nắm/thân hình khi chạm trúng một trong các phần điều khiển
    /// của Valetudo — ✕ xoá tại p0, ✛ di chuyển tại p1 với đường, ↻ xoay tại p1 và
    /// ↖↘ co giãn tại p2 với vùng — hoặc trên thân hình.
    private func beginAnnotationDrag(_ annotation: MapAnnotation, at point: CGPoint) -> Bool {
        let hitRadius = ValetudoHandles.size * ValetudoHandles.hitScale
        for spot in ValetudoHandles.spots(for: annotation, project: projectView) {
            let dx = point.x - spot.point.x
            let dy = point.y - spot.point.y
            if dx * dx + dy * dy <= hitRadius * hitRadius {
                dragMode = spot.kind == ValetudoHandles.deleteKind ? .deleteHandle
                    : spot.kind == ValetudoHandles.moveKind ? .endpoint
                    : spot.kind == ValetudoHandles.rotateKind ? .rotate
                    : .scale
                dragLast = mapView.convert(point, toCoordinateFrom: mapView)
                dragStart = point
                dragMoved = false
                return true
            }
        }
        let coordinate = mapView.convert(point, toCoordinateFrom: mapView)
        if annotation.nearPoint(coordinate.latitude, coordinate.longitude,
                                toleranceMeters: ValetudoHandles.bodyToleranceMeters) {
            dragMode = .body
            dragLast = coordinate
            dragStart = point
            dragMoved = false
            return true
        }
        return false
    }

    /// Applies one drag step exactly like Valetudo's translate branches, then
    /// snaps when the finger lifts (postProcess). A tap on the ✕ handle without
    /// dragging deletes.
    /// Áp dụng một bước kéo đúng như các nhánh translate của Valetudo, rồi hút
    /// lại khi nhả tay (postProcess). Chạm tay nắm ✕ mà không kéo thì xoá.
    @objc private func mapPanned(_ gesture: UIPanGestureRecognizer) {
        let point = gesture.location(in: mapView)
        switch gesture.state {
        case .began:
            mapView.isScrollEnabled = false
        case .changed:
            guard dragMode != .none else { return }
            if hypot(point.x - dragStart.x, point.y - dragStart.y) > ValetudoHandles.dragSlop {
                dragMoved = true
            }
            handleAnnotationDrag(coordinate: mapView.convert(point, toCoordinateFrom: mapView))
        case .ended, .cancelled, .failed:
            endAnnotationDrag(finished: gesture.state == .ended)
        default:
            break
        }
    }

    private func handleAnnotationDrag(coordinate: CLLocationCoordinate2D) {
        guard let index = selectedAnnotationIndex,
              let annotation = navigator.mission.annotation(at: index) else { return }
        let current = MapAnnotation.Point(latitude: coordinate.latitude,
                                          longitude: coordinate.longitude)
        var updated = annotation
        switch dragMode {
        case .endpoint:
            updated = annotation.movingPoint(index: 1, to: current)
        case .body:
            if let last = dragLast {
                updated = annotation.translated(dLat: coordinate.latitude - last.latitude,
                                               dLon: coordinate.longitude - last.longitude)
            }
        case .rotate:
            if let last = dragLast {
                updated = annotation.rotated(
                    last: MapAnnotation.Point(latitude: last.latitude, longitude: last.longitude),
                    current: current)
            }
        case .scale:
            updated = annotation.resized(pointer: current)
        case .none, .deleteHandle:
            break
        }
        dragLast = coordinate
        navigator.mission.updateAnnotation(updated, at: index)
        // Light refresh: the pins and the mission polyline stay put so a drag
        // does not flicker.
        // Làm mới nhẹ: các ghim và đường sứ mệnh đứng yên để thao tác kéo không
        // bị nhấp nháy.
        refreshAnnotationOverlays()
    }

    private func endAnnotationDrag(finished: Bool) {
        defer {
            dragMode = .none
            dragLast = nil
            dragMoved = false
            mapView.isScrollEnabled = true
        }
        if dragMode == .deleteHandle && !dragMoved && finished {
            deleteSelected()
            return
        }
        // postProcess: whole metres and a 5° angle (PROTOCOL.md §10).
        // postProcess: mét nguyên và góc bội của 5° (PROTOCOL.md §10).
        if dragMoved && dragMode != .deleteHandle,
           let index = selectedAnnotationIndex,
           let annotation = navigator.mission.annotation(at: index) {
            navigator.mission.updateAnnotation(annotation.snapped(), at: index)
            flash("\(annotation.type.label) adjusted — snapped to 5° and whole metres.")
        }
        refreshOverlays()
    }

    // MARK: - Actions

    @objc private func close() {
        dismiss(animated: true)
    }

    @objc private func cutMotors() {
        navigator.stop()
        client.emergency()
        UINotificationFeedbackGenerator().notificationOccurred(.error)
        flash("CUT MOTORS — motors off.")
    }

    private func flash(_ text: String) {
        statusLabel.text = text
    }

    // MARK: - Overlays

    private func refreshOverlays() {
        mapView.removeOverlays(mapView.overlays)

        // Mission legs — the plan this editor annotates.
        // Các chặng của sứ mệnh — kế hoạch mà trình sửa này chú thích.
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

        refreshAnnotationOverlays()

        // Waypoint annotations plus the pending Go-to pin.
        // Các annotation điểm hành trình cộng ghim Đi tới đang chờ.
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
        if let target = NavStore.shared.tapTarget {
            let ann = MKPointAnnotation()
            ann.title = "Guided"
            ann.coordinate = CLLocationCoordinate2D(latitude: target.latitude,
                                                    longitude: target.longitude)
            mapView.addAnnotation(ann)
        }

        // Drone estimated marker (same launch-frame maths as the Mission page).
        // Marker ước tính của drone (cùng phép tính khung cất cánh như trang Mission).
        if navigator.launchLatitude != 0 || navigator.launchLongitude != 0,
           let fix = mapView.userLocation.location {
            let ne = LaunchFrame.toNorthEast(x: telemetry.value("nav.pos_x") ?? 0,
                                             y: telemetry.value("nav.pos_y") ?? 0,
                                             alphaDegrees: navigator.alphaDegrees)
            let north = GeoMath.destinationPoint(fix.coordinate.latitude, fix.coordinate.longitude,
                                                 0, ne.north)
            let east = GeoMath.destinationPoint(north.latitude, north.longitude, 90, ne.east)
            let droneAnn = MKPointAnnotation()
            droneAnn.title = "Drone (estimated)"
            droneAnn.coordinate = CLLocationCoordinate2D(latitude: east.latitude,
                                                         longitude: east.longitude)
            mapView.addAnnotation(droneAnn)
        }

        refreshTiles()
        updateIconButtons()
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
        for (index, annotation) in navigator.mission.annotations.enumerated() {
            mapView.addOverlay(MapAnnotationShape(annotation: annotation,
                                                  selected: selectedAnnotationIndex == index))
        }
    }

    private func updateIconButtons() {
        deleteButton.isEnabled = selectedAnnotationIndex != nil
        clearButton.isEnabled = !navigator.mission.annotations.isEmpty
    }
}

// MARK: - UIGestureRecognizerDelegate

extension IndoorMapViewController: UIGestureRecognizerDelegate {

    /// The rectangle draw only begins in a placement mode and the editing pan
    /// only on Valetudo's handles or the selected shape's body; everywhere else
    /// the map pans normally.
    /// Việc vẽ hình chữ nhật chỉ bắt đầu ở chế độ đặt và việc kéo sửa chỉ trên
    /// tay nắm của Valetudo hoặc thân hình đang chọn; mọi nơi khác bản đồ cuộn
    /// bình thường.
    func gestureRecognizerShouldBegin(_ gestureRecognizer: UIGestureRecognizer) -> Bool {
        if gestureRecognizer == rectGesture {
            return mode == .keepout || mode == .wind
        }
        if gestureRecognizer == dragGesture {
            guard mode == .select, let index = selectedAnnotationIndex,
                  let annotation = navigator.mission.annotation(at: index) else { return false }
            return beginAnnotationDrag(annotation, at: dragGesture.location(in: mapView))
        }
        return true
    }
}

// MARK: - MKMapViewDelegate

extension IndoorMapViewController: MKMapViewDelegate {
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

extension IndoorMapViewController: CLLocationManagerDelegate {
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        // The blue dot is all this page needs; nothing to hold or filter here.
        // Trang này chỉ cần chấm xanh; không có gì phải giữ hay lọc ở đây.
    }

    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        if manager.authorizationStatus == .authorizedWhenInUse
            || manager.authorizationStatus == .authorizedAlways {
            manager.startUpdatingLocation()
        }
    }
}
