//
//  PlannerViewController.swift
//  Crazyflie client
//
//  The Plan screen — the Mission Planner Flight Plan port (PROTOCOL.md §12).
//  MapKit shows the plan; the grid is its Command/P1..P4/Lat/Lon/Alt table;
//  Read/Write are the MISSION_* handshake over CRTP port 7 channel 2 and Start
//  flies the stored mission onboard. The `.waypoints` files are Mission
//  Planner's own QGC WPL 110 format, so plans move between the two.
//  Màn Plan — bản chuyển thể Flight Plan của Mission Planner (PROTOCOL.md mục
//  12). MapKit hiển thị kế hoạch; lưới là bảng Lệnh/P1..P4/Vĩ/Kinh/Độ cao của
//  nó; Read/Write là bắt tay MISSION_* trên CRTP cổng 7 kênh 2 và Start bay
//  nhiệm vụ đã lưu trên bo. Tệp `.waypoints` đúng định dạng QGC WPL 110 của
//  Mission Planner, nên kế hoạch đi lại được giữa hai bên.
//

import UIKit
import MapKit
import CoreLocation
import UniformTypeIdentifiers

final class PlannerViewController: UIViewController, MissionTransferListener {

    private let client: CrtpClient
    private let telemetry: TelemetryStore

    /// The plan this screen edits. It is its own mission — the Mission screen's
    /// NavStore plan is a different, phone-flown one.
    /// Kế hoạch màn hình này sửa. Nó là một nhiệm vụ riêng — kế hoạch NavStore
    /// của màn Mission là nhiệm vụ khác, bay bởi điện thoại.
    private let mission = PlannerMission()
    /// The grid row being edited, or -1.
    /// Dòng lưới đang sửa, hoặc -1.
    private var selected = -1
    /// Mission Planner's "Alt" box: the altitude new waypoints get.
    /// Hộp "Alt" của Mission Planner: độ cao cho các waypoint mới.
    private var defaultAlt: Float = 3.0
    /// Guards the edit strip while it is being filled from the row (a fill must
    /// not read as a user commit).
    /// Giữ dải sửa trong khi đang điền từ dòng (một lần điền không được coi là
    /// ghi nhận của người dùng).
    private var filling = false
    /// Set while a document picker is saving, so the delegate knows which flow.
    /// Đặt khi trình chọn tệp đang lưu, để delegate biết đang ở luồng nào.
    private var saving = false

    /// Map pin per grid row that carries a position (rows with 0/0 stay off
    /// the map, like the Android screen).
    /// Ghim bản đồ cho mỗi dòng lưới có vị trí (dòng 0/0 ở ngoài bản đồ, như
    /// màn Android).
    private var mapIndices: [Int] = []

    private static let defaultWaypointRadius: Float = 2.5
    /// The commands the firmware flies, the dropdown's first entries.
    /// Các lệnh firmware bay được, các mục đầu của dropdown.
    private static let flownNames = ["WAYPOINT", "TAKEOFF", "LAND", "RETURN_TO_LAUNCH", "DO_JUMP"]

    private let mapView = MKMapView()
    private let statusLabel = UILabel()
    private let editStrip = UIStackView()
    private let commandButton = UIButton(type: .system)
    private let editP1 = UITextField()
    private let editP2 = UITextField()
    private let editP3 = UITextField()
    private let editP4 = UITextField()
    private let editLat = UITextField()
    private let editLon = UITextField()
    private let altField = UITextField()
    private let grid = UITableView(frame: .zero, style: .plain)

    /// Set by the Fly screen: opens another section after this screen closes.
    /// Do màn hình Fly đặt: mở một khu vực khác sau khi màn hình này đóng.
    var onSwitchSection: ((AppSection) -> Void)?
    private var sectionHeader: SectionHeader?

    private lazy var transfer = MissionTransfer(listener: self)
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

    deinit {
        transfer.detach()
    }

    // MARK: - View

    /// Plan is a portrait screen (the map reads best upright, like Android's).
    /// Plan là màn hình dọc (bản đồ đọc tốt nhất khi thẳng đứng, như Android).
    override var supportedInterfaceOrientations: UIInterfaceOrientationMask { .portrait }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = "Plan"
        navigationItem.leftBarButtonItem = UIBarButtonItem(
            barButtonSystemItem: .done, target: self, action: #selector(close))

        // Shared header; one tap to cut, like Android's Mission/Plan screens.
        // Header dùng chung; chạm một lần để cắt, như màn Mission/Plan trên Android.
        let header = SectionHeader(host: self, selected: .plan, telemetry: telemetry,
                                   twoTapCut: false) { [weak self] in
            self?.cutMotors()
        }
        header.leaveGuard = { [weak self] in
            guard let self = self, self.transfer.isBusy else { return nil }
            return "A mission transfer is running. Wait for it to finish before switching."
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

        statusLabel.numberOfLines = 1
        statusLabel.font = .monospacedSystemFont(ofSize: 12, weight: .regular)
        statusLabel.text = "Tap the map to add a waypoint."

        buildEditStrip()
        let tools = buildToolsRow()
        buildGrid()
        let addRow = buildAddRow()
        let transferRow = buildTransferRow()

        editStrip.isHidden = true
        grid.heightAnchor.constraint(equalToConstant: 150).isActive = true

        stack.addArrangedSubview(statusLabel)
        stack.addArrangedSubview(editStrip)
        stack.addArrangedSubview(tools)
        stack.addArrangedSubview(grid)
        stack.addArrangedSubview(addRow)
        stack.addArrangedSubview(transferRow)

        // Tapping the map adds a waypoint there — Mission Planner's map click.
        // Chạm bản đồ thêm một waypoint tại đó — map click của Mission Planner.
        let tap = UITapGestureRecognizer(target: self, action: #selector(mapTapped(_:)))
        mapView.addGestureRecognizer(tap)

        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.requestWhenInUseAuthorization()
        locationManager.startUpdatingLocation()

        transfer.attach(client)
        refreshUi()
    }

    // MARK: - Actions

    @objc private func close() {
        transfer.detach()
        dismiss(animated: true)
    }

    private func cutMotors() {
        client.emergency()
        flash("CUT MOTORS - motors off")
    }

    @objc private func mapTapped(_ gesture: UITapGestureRecognizer) {
        commitRow()
        let point = gesture.location(in: mapView)
        let coordinate = mapView.convert(point, toCoordinateFrom: mapView)
        // Keep-out zones veto the placement outright (PROTOCOL.md §10).
        // Vùng cấm bay phủ quyết việc đặt điểm ngay (PROTOCOL.md mục 10).
        if let refusal = NavStore.shared.navigator.checkWaypoint(coordinate.latitude,
                                                                coordinate.longitude) {
            flash("Cannot place waypoint: \(refusal)")
            return
        }
        var item = PlannerItem.waypoint(coordinate.latitude, coordinate.longitude, alt: defaultAlt)
        item.p2 = PlannerViewController.defaultWaypointRadius
        mission.add(item)
        selected = mission.size - 1
        refreshUi()
    }

    // MARK: - Mission editing

    private func selectRow(_ position: Int) {
        selected = position
        refreshUi()
    }

    @objc private func onAddWaypoint() {
        commitRow()
        var item = PlannerItem.waypoint(0, 0, alt: defaultAlt)
        item.p2 = PlannerViewController.defaultWaypointRadius
        // Without a map position yet, place it on the last one so the marker
        // stack stays visible; the Lat/Lng fields move it where it belongs.
        // Chưa có vị trí trên bản đồ thì đặt lên vị trí cuối để các marker còn
        // nhìn thấy; ô Vĩ độ/Kinh độ sẽ đưa nó về đúng chỗ.
        if mission.size > 0 {
            let last = mission.items[mission.size - 1]
            item.lat = last.lat
            item.lon = last.lon
        }
        mission.add(item)
        selected = mission.size - 1
        refreshUi()
    }

    @objc private func onMoveUp() { moveSelected(-1) }
    @objc private func onMoveDown() { moveSelected(1) }

    private func moveSelected(_ delta: Int) {
        commitRow()
        guard selected >= 0 else {
            flash("Select a mission item first")
            return
        }
        let to = selected + delta
        guard mission.items.indices.contains(to) else { return }
        let moved = mission.items.remove(at: selected)
        mission.items.insert(moved, at: to)
        selected = to
        refreshUi()
    }

    @objc private func onRemove() {
        commitRow()
        guard selected >= 0 else {
            flash("Select a mission item first")
            return
        }
        mission.items.remove(at: selected)
        if selected >= mission.size {
            selected = mission.size - 1
        }
        refreshUi()
    }

    /// Apply the altitude box to the selected row; else it is the new-WP alt.
    /// Ghi ô độ cao vào dòng đang chọn; nếu không thì đó là độ cao cho WP mới.
    @objc private func onSetAlt() {
        let alt = Float(altField.text ?? "") ?? defaultAlt
        defaultAlt = alt
        if selected >= 0 {
            commitRow()
            mission.items[selected].alt = alt
            refreshUi()
            flash("Altitude set")
        } else {
            flash("New waypoints will use alt \(alt)")
        }
    }

    /// Write the edit-strip fields into the selected row.
    /// Ghi các trường trong dải sửa vào dòng đang chọn.
    private func commitRow() {
        guard !filling, mission.items.indices.contains(selected) else { return }
        mission.items[selected].p1 = Float(editP1.text ?? "") ?? mission.items[selected].p1
        mission.items[selected].p2 = Float(editP2.text ?? "") ?? mission.items[selected].p2
        mission.items[selected].p3 = Float(editP3.text ?? "") ?? mission.items[selected].p3
        mission.items[selected].p4 = Float(editP4.text ?? "") ?? mission.items[selected].p4
        mission.items[selected].lat = Double(editLat.text ?? "") ?? mission.items[selected].lat
        mission.items[selected].lon = Double(editLon.text ?? "") ?? mission.items[selected].lon
    }

    /// Mission Planner's command column: a dropdown of the MAV_CMD names.
    /// Cột lệnh của Mission Planner: một dropdown các tên MAV_CMD.
    @objc private func commandTapped() {
        guard mission.items.indices.contains(selected) else { return }
        commitRow()
        let item = mission.items[selected]
        // The dropdown lists the commands this firmware flies plus the row's
        // own (or "UNKNOWN", as Mission Planner labels the rest).
        // Dropdown liệt kê các lệnh firmware bay được cộng với lệnh của dòng đó
        // (hoặc "UNKNOWN", như Mission Planner đặt tên cho phần còn lại).
        var names = PlannerViewController.flownNames
        let current = item.commandName
        if !names.contains(current) {
            names.append(current)
        }
        if !names.contains("UNKNOWN") {
            names.append("UNKNOWN")
        }
        let actions = names.map { name in
            UIAction(title: name, state: name == current ? .on : .off) { [weak self] _ in
                self?.pickCommand(name)
            }
        }
        commandButton.menu = UIMenu(children: actions)
        commandButton.showsMenuAsPrimaryAction = true
        // Present the menu now — the tap that built it must open it too.
        // Hiện menu ngay — cái chạm dựng nó cũng phải mở nó ra.
        commandButton.sendActions(for: .menuActionTriggered)
    }

    private func pickCommand(_ name: String) {
        guard mission.items.indices.contains(selected) else { return }
        let command = PlannerItem.parseCommand(name)
        if command >= 0 {
            // "UNKNOWN" parses to -1 and keeps the row's id, like Mission
            // Planner's Tag round-trip.
            // "UNKNOWN" phân tích ra -1 và giữ id của dòng, như cách Mission
            // Planner giữ Tag.
            mission.items[selected].command = command
            refreshUi()
        }
    }

    // MARK: - Files

    @objc private func onLoadMission() {
        // `asCopy: true` is the old `.import` mode: take a copy of the file
        // rather than hold security-scoped access to the original.
        // `asCopy: true` là chế độ `.import` cũ: lấy một bản sao của file
        // thay vì giữ quyền truy cập phạm vi bảo mật vào file gốc.
        saving = false
        let picker = UIDocumentPickerViewController(forOpeningContentTypes: [.plainText],
                                                    asCopy: true)
        picker.delegate = self
        present(picker, animated: true)
    }

    @objc private func onSaveMission() {
        commitRow()
        let text = mission.format()
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("mission.waypoints")
        do {
            try text.write(to: url, atomically: true, encoding: .utf8)
        } catch {
            flash("Save failed: \(error.localizedDescription)")
            return
        }
        saving = true
        let picker = UIDocumentPickerViewController(forExporting: [url], asCopy: true)
        picker.delegate = self
        present(picker, animated: true)
    }

    /// Swap in a freshly loaded/downloaded mission (its home comes with it).
    /// Thay bằng nhiệm vụ vừa nạp/tải về (home của nó đi kèm).
    private func replaceMission(_ loaded: PlannerMission) {
        mission.clear()
        mission.home = loaded.home
        for item in loaded.items {
            mission.add(item)
        }
        selected = mission.size == 0 ? -1 : 0
        refreshUi()
    }

    // MARK: - Transfer (PROTOCOL.md §12)

    @objc private func onUpload() {
        commitRow()
        ensureOrigin()
        transfer.attach(client)
        transfer.upload(mission.items)
    }

    @objc private func onDownload() {
        commitRow()
        transfer.attach(client)
        transfer.download()
    }

    @objc private func onStartMission() {
        commitRow()
        ensureOrigin()
        transfer.attach(client)
        transfer.startMission()
    }

    /// The mission items are WGS-84 degrees and the drone flies a dead-reckoned
    /// local frame — the origin is the only bridge (§12). Send the mission's
    /// home, or fall back to the phone's fix when the plan has no home yet.
    /// Các mục nhiệm vụ là độ WGS-84 còn drone bay theo khung cục bộ tự dẫn
    /// đường — điểm gốc là cây cầu duy nhất (mục 12). Gửi home của nhiệm vụ, hoặc
    /// dùng vị trí GPS của điện thoại khi kế hoạch chưa có home.
    private func ensureOrigin() {
        if mission.home.lat == 0 && mission.home.lon == 0 {
            if let fix = locationManager.location {
                mission.home.lat = fix.coordinate.latitude
                mission.home.lon = fix.coordinate.longitude
            }
        }
        if mission.home.lat != 0 || mission.home.lon != 0 {
            client.sendNavOrigin(latitude: mission.home.lat, longitude: mission.home.lon)
        } else {
            flash("No home set - the drone will use its stored origin")
        }
    }

    private func missionResultName(_ result: Int) -> String {
        switch result {
        case MissionPacket.missionAccepted: return "accepted"
        case MissionPacket.missionNoSpace: return "no space on the drone"
        case MissionPacket.missionInvalid: return "invalid mission"
        case MissionPacket.missionDenied: return "denied"
        default: return "error \(result)"
        }
    }

    // MARK: - MissionTransferListener

    func transferStatus(_ message: String) {
        statusLabel.text = message
    }

    func uploadFinished(_ result: Int) {
        flash(result == MissionPacket.missionAccepted
                ? "Mission uploaded" : "Upload failed: \(missionResultName(result))")
        refreshUi()
    }

    func downloadFinished(_ mission: PlannerMission?, _ result: Int) {
        guard let mission = mission, result == MissionPacket.missionAccepted else {
            flash("Download failed: \(missionResultName(result))")
            return
        }
        replaceMission(mission)
        flash("Mission downloaded (\(mission.size) items)")
    }

    func commandFinished(_ cmd: UInt8, _ ok: Bool) {
        if cmd == MissionPacket.cmdStart {
            flash(ok ? "Mission started" : "Start refused")
        } else if cmd == MissionPacket.cmdSetCurrent {
            flash(ok ? "Current item set" : "Bad item number")
        }
    }

    func currentItem(_ seq: Int) {
        statusLabel.text = String(format: "flying item %d of %d", seq + 1, mission.size)
    }

    // MARK: - UI

    private func flash(_ text: String) {
        statusLabel.text = text
    }

    private func refreshUi() {
        grid.reloadData()

        // Map markers: rows that carry a position, in mission order.
        // Marker bản đồ: các dòng có vị trí, theo thứ tự nhiệm vụ.
        mapView.removeAnnotations(mapView.annotations)
        mapView.removeOverlays(mapView.overlays)
        mapIndices = []
        for (i, item) in mission.items.enumerated()
        where item.lat != 0 || item.lon != 0 {
            mapIndices.append(i)
            let ann = MKPointAnnotation()
            ann.title = "#\(i + 1)"
            ann.coordinate = CLLocationCoordinate2D(latitude: item.lat, longitude: item.lon)
            mapView.addAnnotation(ann)
        }
        if mission.home.lat != 0 || mission.home.lon != 0 {
            let home = MKPointAnnotation()
            home.title = "Home"
            home.coordinate = CLLocationCoordinate2D(latitude: mission.home.lat,
                                                    longitude: mission.home.lon)
            mapView.addAnnotation(home)
        }
        var coords: [CLLocationCoordinate2D] = []
        if mission.home.lat != 0 || mission.home.lon != 0 {
            coords.append(CLLocationCoordinate2D(latitude: mission.home.lat,
                                                longitude: mission.home.lon))
        }
        for i in mapIndices {
            coords.append(CLLocationCoordinate2D(latitude: mission.items[i].lat,
                                                longitude: mission.items[i].lon))
        }
        if coords.count > 1 {
            mapView.addOverlay(MKPolyline(coordinates: &coords, count: coords.count))
        }

        statusLabel.text = String(format: "%d items · home %.6f, %.6f",
                                  mission.size, mission.home.lat, mission.home.lon)
        fillEditStrip()
    }

    private func fillEditStrip() {
        filling = true
        defer { filling = false }
        guard mission.items.indices.contains(selected) else {
            editStrip.isHidden = true
            return
        }
        editStrip.isHidden = false
        let item = mission.items[selected]
        editP1.text = String(item.p1)
        editP2.text = String(item.p2)
        editP3.text = String(item.p3)
        editP4.text = String(item.p4)
        editLat.text = String(format: "%.7f", item.lat)
        editLon.text = String(format: "%.7f", item.lon)
        commandButton.applyLabel(item.commandName, symbol: "chevron.down",
                                 font: .systemFont(ofSize: 13, weight: .semibold))
    }

    // MARK: - View building

    private func buildEditStrip() {
        editStrip.axis = .vertical
        editStrip.spacing = 4

        let row1 = UIStackView(arrangedSubviews: [commandButton, labeled("P1", editP1)])
        let row2 = UIStackView(arrangedSubviews: [
            labeled("P2", editP2), labeled("P3", editP3), labeled("P4", editP4)])
        let row3 = UIStackView(arrangedSubviews: [labeled("Lat", editLat), labeled("Lng", editLon)])
        for row in [row1, row2, row3] {
            row.axis = .horizontal
            row.spacing = 6
            row.distribution = .fillEqually
            editStrip.addArrangedSubview(row)
        }
        for field in [editP1, editP2, editP3, editP4, editLat, editLon] {
            field.borderStyle = .roundedRect
            field.font = .monospacedDigitSystemFont(ofSize: 12, weight: .regular)
            field.keyboardType = .numbersAndPunctuation
            field.addTarget(self, action: #selector(fieldEnded), for: .editingDidEnd)
        }
        commandButton.applyLabel("WAYPOINT", symbol: "chevron.down",
                                 font: .systemFont(ofSize: 13, weight: .semibold))
        commandButton.addTarget(self, action: #selector(commandTapped), for: .touchUpInside)
    }

    /// A label + field pair sharing one grid cell of the edit strip.
    /// Cặp nhãn + trường dùng chung một ô lưới của dải sửa.
    private func labeled(_ title: String, _ field: UITextField) -> UIStackView {
        let label = UILabel()
        label.text = title
        label.font = .systemFont(ofSize: 12)
        label.setContentHuggingPriority(.required, for: .horizontal)
        let pair = UIStackView(arrangedSubviews: [label, field])
        pair.axis = .horizontal
        pair.spacing = 2
        pair.alignment = .center
        return pair
    }

    @objc private func fieldEnded() {
        commitRow()
        // The row's other cells show the committed values.
        // Các ô khác của dòng hiện giá trị đã ghi nhận.
        grid.reloadData()
    }

    private func buildToolsRow() -> UIStackView {
        // Routine tools: icon-only, words in the VoiceOver label.
        // Công cụ thường ngày: chỉ có biểu tượng, chữ nằm trong nhãn VoiceOver.
        let up = toolButton("arrow.up", "Move item up", #selector(onMoveUp))
        let down = toolButton("arrow.down", "Move item down", #selector(onMoveDown))
        let remove = toolButton("trash", "Delete selected item", #selector(onRemove))
        let load = toolButton("folder", "Load mission file", #selector(onLoadMission))
        let save = toolButton("square.and.arrow.down", "Save mission file", #selector(onSaveMission))
        let label = UILabel()
        label.text = "Mission"
        label.font = .systemFont(ofSize: 13, weight: .bold)
        label.setContentHuggingPriority(.required, for: .horizontal)

        let row = UIStackView(arrangedSubviews: [label, up, down, remove, load, save])
        row.axis = .horizontal
        row.spacing = 8
        row.distribution = .fillEqually
        row.heightAnchor.constraint(equalToConstant: 52).isActive = true
        return row
    }

    private func toolButton(_ symbol: String, _ spoken: String, _ action: Selector) -> UIButton {
        let button = UIButton(type: .system)
        button.applyLabel(nil, symbol: symbol, accessibilityText: spoken)
        button.addTarget(self, action: action, for: .touchUpInside)
        return button
    }

    private func buildGrid() {
        grid.dataSource = self
        grid.delegate = self
        grid.rowHeight = UITableView.automaticDimension
        grid.estimatedRowHeight = 64
        grid.register(PlannerItemCell.self, forCellReuseIdentifier: PlannerItemCell.reuseId)
    }

    private func buildAddRow() -> UIStackView {
        let addButton = UIButton(type: .system)
        addButton.applyLabel("Add WP", symbol: "mappin.and.ellipse",
                             font: .systemFont(ofSize: 13, weight: .semibold))
        addButton.addTarget(self, action: #selector(onAddWaypoint), for: .touchUpInside)

        let altLabel = UILabel()
        altLabel.text = "Alt"
        altLabel.font = .systemFont(ofSize: 12)
        altField.borderStyle = .roundedRect
        altField.text = "3.0"
        altField.font = .monospacedDigitSystemFont(ofSize: 12, weight: .regular)
        altField.keyboardType = .decimalPad
        altField.widthAnchor.constraint(equalToConstant: 60).isActive = true

        let setButton = UIButton(type: .system)
        setButton.applyLabel("Set", symbol: nil,
                             font: .systemFont(ofSize: 13, weight: .semibold))
        setButton.addTarget(self, action: #selector(onSetAlt), for: .touchUpInside)

        let row = UIStackView(arrangedSubviews: [addButton, altLabel, altField, setButton])
        row.axis = .horizontal
        row.spacing = 8
        row.alignment = .center
        return row
    }

    private func buildTransferRow() -> UIStackView {
        // Read/Write in Mission Planner's words — the MISSION_* handshake of
        // §12 over CRTP port 7 channel 2. Start flies the stored mission (it
        // arms first), so it carries an icon AND a label.
        // Read/Write theo cách gọi của Mission Planner — bắt tay MISSION_* của
        // mục 12 trên CRTP cổng 7 kênh 2. Start bay nhiệm vụ đã lưu (nó arm
        // trước), nên mang cả biểu tượng lẫn nhãn.
        let rowFont = UIFont.systemFont(ofSize: 12, weight: .semibold)
        let upload = UIButton(type: .system)
        upload.applyLabel("Upload", symbol: "arrow.up", font: rowFont, imageOnTop: true)
        upload.addTarget(self, action: #selector(onUpload), for: .touchUpInside)

        let download = UIButton(type: .system)
        download.applyLabel("Download", symbol: "arrow.down", font: rowFont, imageOnTop: true)
        download.addTarget(self, action: #selector(onDownload), for: .touchUpInside)

        let start = UIButton(type: .system)
        start.applyLabel("Start", symbol: "play.fill", font: rowFont, imageOnTop: true)
        start.addTarget(self, action: #selector(onStartMission), for: .touchUpInside)

        let row = UIStackView(arrangedSubviews: [upload, download, start])
        row.axis = .horizontal
        row.distribution = .fillEqually
        return row
    }
}

// MARK: - UITableViewDataSource / Delegate

extension PlannerViewController: UITableViewDataSource, UITableViewDelegate {

    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        mission.size
    }

    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(withIdentifier: PlannerItemCell.reuseId,
                                                 for: indexPath) as! PlannerItemCell
        cell.fill(seq: indexPath.row, item: mission.items[indexPath.row])
        return cell
    }

    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        commitRow()
        selectRow(indexPath.row)
    }
}

/// One mission-item grid row: the Mission Planner Flight Plan grid columns
/// (# / Command / P1-P4 / Lat / Lon / Alt) stacked for a phone screen.
/// Một dòng lưới mục nhiệm vụ: các cột lưới Flight Plan của Mission Planner
/// (# / Lệnh / P1-P4 / Vĩ độ / Kinh độ / Độ cao) xếp chồng cho màn hình điện thoại.
final class PlannerItemCell: UITableViewCell {

    static let reuseId = "PlannerItemCell"

    private let seqLabel = UILabel()
    private let cmdLabel = UILabel()
    private let altLabel = UILabel()
    private let paramsLabel = UILabel()
    private let posLabel = UILabel()

    override init(style: UITableViewCell.CellStyle, reuseIdentifier: String?) {
        super.init(style: style, reuseIdentifier: reuseIdentifier)
        seqLabel.font = .systemFont(ofSize: 13, weight: .bold)
        seqLabel.widthAnchor.constraint(equalToConstant: 36).isActive = true
        cmdLabel.font = .systemFont(ofSize: 13, weight: .bold)
        altLabel.font = .monospacedDigitSystemFont(ofSize: 12, weight: .regular)

        let top = UIStackView(arrangedSubviews: [seqLabel, cmdLabel, altLabel])
        top.axis = .horizontal
        top.spacing = 6

        for label in [paramsLabel, posLabel] {
            label.font = .monospacedSystemFont(ofSize: 12, weight: .regular)
            label.textColor = .secondaryLabel
        }

        let stack = UIStackView(arrangedSubviews: [top, paramsLabel, posLabel])
        stack.axis = .vertical
        stack.spacing = 2
        stack.translatesAutoresizingMaskIntoConstraints = false
        contentView.addSubview(stack)
        NSLayoutConstraint.activate([
            stack.topAnchor.constraint(equalTo: contentView.topAnchor, constant: 4),
            stack.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 8),
            stack.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -8),
            stack.bottomAnchor.constraint(equalTo: contentView.bottomAnchor, constant: -4),
        ])
        selectionStyle = .default
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not supported")
    }

    /// The row: # / Command / Alt, then P1-P4, then the position.
    /// Dòng lưới: # / Lệnh / Độ cao, rồi P1-P4, rồi vị trí.
    func fill(seq: Int, item: PlannerItem) {
        seqLabel.text = "#\(seq)"
        cmdLabel.text = item.commandName
        altLabel.text = String(format: "alt %g", item.alt)
        paramsLabel.text = String(format: "P1 %g  P2 %g  P3 %g  P4 %g",
                                  item.p1, item.p2, item.p3, item.p4)
        posLabel.text = String(format: "%.8f, %.8f", item.lat, item.lon)
    }
}

// MARK: - MapKit

extension PlannerViewController: MKMapViewDelegate {

    func mapView(_ mapView: MKMapView, rendererFor overlay: MKOverlay) -> MKOverlayRenderer {
        if let polyline = overlay as? MKPolyline {
            let r = MKPolylineRenderer(polyline: polyline)
            r.strokeColor = .systemBlue
            r.lineWidth = 2
            return r
        }
        return MKOverlayRenderer(overlay: overlay)
    }

    func mapView(_ mapView: MKMapView, viewFor annotation: MKAnnotation) -> MKAnnotationView? {
        if annotation is MKUserLocation { return nil }
        let id = "plannerPin"
        let view = mapView.dequeueReusableAnnotationView(withIdentifier: id)
            as? MKMarkerAnnotationView ?? MKMarkerAnnotationView(annotation: annotation,
                                                                 reuseIdentifier: id)
        view.annotation = annotation
        view.canShowCallout = true
        // The selected row's marker reads differently — Mission Planner's grid
        // selection and map highlight go together.
        // Marker của dòng được chọn hiển thị khác — lựa chọn trong lưới của
        // Mission Planner và phần tô sáng trên bản đồ đi cùng nhau.
        if let title = annotation.title ?? nil, title.hasPrefix("#"),
           let row = Int(title.dropFirst()), row == selected {
            view.markerTintColor = .systemOrange
        } else if annotation.title == "Home" {
            view.markerTintColor = .systemGreen
        } else {
            view.markerTintColor = .systemBlue
        }
        return view
    }
}

// MARK: - Document picker

extension PlannerViewController: UIDocumentPickerDelegate {

    func documentPicker(_ controller: UIDocumentPickerViewController,
                        didPickDocumentsAt urls: [URL]) {
        guard let url = urls.first else { return }
        if saving {
            flash("Mission saved")
            return
        }
        do {
            let text = try String(contentsOf: url, encoding: .utf8)
            let loaded = try PlannerMission.parse(text)
            replaceMission(loaded)
            if !loaded.invalidLines.isEmpty {
                flash("Mission loaded - \(loaded.invalidLines.count) invalid line(s) skipped")
            } else {
                flash("Mission loaded")
            }
        } catch {
            flash("Load failed: \(error.localizedDescription)")
        }
    }
}

// MARK: - Location

extension PlannerViewController: CLLocationManagerDelegate {

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        // The fix only feeds ensureOrigin; no per-fix UI to refresh.
        // Fix chỉ dùng cho ensureOrigin; không có giao diện cần làm mới theo fix.
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
    }
}
