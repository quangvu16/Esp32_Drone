//
//  ConnectionViewController.swift
//  Crazyflie client
//
//  Connection-method picker. Replaces the hard-coded UDP endpoints with a
//  user-configurable transport + address (PROTOCOL.md §8). The choice is
//  persisted in LinkSettings and the ViewModel rebuilds the driver from it.
//  The picker also names the BLE peripheral and can ask the drone to switch
//  link profile (FC_CMD_LINK 0x10) — explicitly, never on Apply.
//  Trình chọn phương thức kết nối. Thay cho các điểm cuối UDP cố định bằng tầng
//  truyền + địa chỉ do người dùng cấu hình (PROTOCOL.md §8). Lựa chọn được lưu
//  trong LinkSettings và ViewModel dựng lại trình điều khiển từ đó. Trình chọn
//  còn đặt tên thiết bị BLE và có thể yêu cầu drone đổi hồ sơ liên kết
//  (FC_CMD_LINK 0x10) — một cách tường minh, không bao giờ khi bấm Apply.
//

import CoreBluetooth
import UIKit

final class ConnectionViewController: UIViewController, UITableViewDataSource, UITableViewDelegate {

    /// Called when the user saves: the app should tear down and rebuild its
    /// link with the new settings.
    var onApply: (() -> Void)?

    /// Called when the user confirms "Switch drone to this profile": the app
    /// sends FC_CMD_LINK with the selected profile number. The firmware
    /// refuses while armed and reboots otherwise (~1.5 s).
    /// Gọi khi người dùng xác nhận "Đổi hồ sơ drone sang hồ sơ này": app gửi
    /// FC_CMD_LINK với số hồ sơ đã chọn. Firmware từ chối khi đang bay còn
    /// không thì khởi động lại (~1,5 giây).
    var onSwitchLinkProfile: ((UInt8) -> Void)?

    private let typeControl = UISegmentedControl(items: LinkType.allCases.map { $0.title })
    private let detailLabel = UILabel()
    private let hostField = UITextField()
    private let devicePortField = UITextField()
    private let appPortField = UITextField()
    private let bleNameField = UITextField()
    private let deviceTable = UITableView()
    private let connectDeviceButton = UIButton(type: .system)
    private let bleNoteLabel = UILabel()
    private let switchProfileButton = UIButton(type: .system)
    private let scrollView = UIScrollView()
    private let stack = UIStackView()

    /// The live device list — (name, identifier, rssi) as advertisements arrive.
    /// Danh sách thiết bị sống — (name, identifier, rssi) khi quảng bá đến.
    private var devices: [(name: String, id: String, rssi: Int)] = []
    private var selectedId: String?
    private var scanner: PickerScanner?

    /// Landscape-only, matching the connect screen that presents it.
    /// Chỉ nằm ngang, theo màn hình kết nối mở nó ra.
    override var supportedInterfaceOrientations: UIInterfaceOrientationMask { .landscapeRight }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = "Connection"

        navigationItem.leftBarButtonItem = UIBarButtonItem(
            barButtonSystemItem: .cancel, target: self, action: #selector(cancel))
        navigationItem.rightBarButtonItem = UIBarButtonItem(
            title: "Apply", style: .done, target: self, action: #selector(apply))

        scrollView.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(scrollView)
        stack.axis = .vertical
        stack.spacing = 12
        stack.translatesAutoresizingMaskIntoConstraints = false
        scrollView.addSubview(stack)

        NSLayoutConstraint.activate([
            scrollView.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            scrollView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            scrollView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            scrollView.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            stack.topAnchor.constraint(equalTo: scrollView.contentLayoutGuide.topAnchor, constant: 16),
            stack.leadingAnchor.constraint(equalTo: scrollView.frameLayoutGuide.leadingAnchor, constant: 16),
            stack.trailingAnchor.constraint(equalTo: scrollView.frameLayoutGuide.trailingAnchor, constant: -16),
            stack.bottomAnchor.constraint(equalTo: scrollView.contentLayoutGuide.bottomAnchor, constant: -16),
        ])

        typeControl.selectedSegmentIndex = LinkSettings.linkType.rawValue
        typeControl.addTarget(self, action: #selector(typeChanged), for: .valueChanged)
        // SPP is greyed out, not hidden: the title still documents that the
        // firmware has the profile (LINK 2) even though iOS cannot open it.
        // SPP bị làm mờ, không ẩn: tiêu đề vẫn ghi nhận firmware có hồ sơ đó
        // (LINK 2) dù iOS không mở được nó.
        if let sppIndex = LinkType.allCases.firstIndex(of: .bluetoothSPP) {
            typeControl.setEnabled(false, forSegmentAt: sppIndex)
        }
        stack.addArrangedSubview(typeControl)

        detailLabel.numberOfLines = 0
        detailLabel.font = .systemFont(ofSize: 13)
        detailLabel.textColor = .secondaryLabel
        stack.addArrangedSubview(detailLabel)

        configureField(hostField, placeholder: "Host (192.168.43.42 or esp-drone.local)")
        configureField(devicePortField, placeholder: "Drone UDP port (2390)")
        devicePortField.keyboardType = .numberPad
        configureField(appPortField, placeholder: "Phone UDP port (2399)")
        appPortField.keyboardType = .numberPad
        stack.addArrangedSubview(hostField)
        stack.addArrangedSubview(devicePortField)
        stack.addArrangedSubview(appPortField)

        configureField(bleNameField, placeholder: "BLE drone name (ESP32_Done; blank = first match)")
        stack.addArrangedSubview(bleNameField)
        bleNameField.addTarget(self, action: #selector(nameEdited), for: .editingChanged)

        // The live device list: tap a row to choose that exact drone — the
        // identifier it carries is what the link connects to, no name scan.
        // Danh sách thiết bị sống: chạm một hàng để chọn đúng drone đó — định
        // danh nó mang theo là thứ link kết nối tới, không quét theo tên.
        deviceTable.dataSource = self
        deviceTable.delegate = self
        deviceTable.translatesAutoresizingMaskIntoConstraints = false
        deviceTable.heightAnchor.constraint(equalToConstant: 150).isActive = true
        stack.addArrangedSubview(deviceTable)

        connectDeviceButton.applyLabel("Connect to selected device", symbol: "link",
                                       font: .systemFont(ofSize: 15, weight: .semibold))
        connectDeviceButton.addTarget(self, action: #selector(connectPicked), for: .touchUpInside)
        stack.addArrangedSubview(connectDeviceButton)

        bleNoteLabel.numberOfLines = 0
        bleNoteLabel.font = .monospacedSystemFont(ofSize: 12, weight: .regular)
        bleNoteLabel.textColor = .secondaryLabel
        stack.addArrangedSubview(bleNoteLabel)

        switchProfileButton.applyLabel("Switch drone to this profile",
                                       symbol: "arrow.triangle.2.circlepath",
                                       font: .systemFont(ofSize: 15, weight: .semibold))
        switchProfileButton.addTarget(self, action: #selector(switchProfileTapped), for: .touchUpInside)
        stack.addArrangedSubview(switchProfileButton)

        refreshForType()
    }

    private func configureField(_ field: UITextField, placeholder: String) {
        field.borderStyle = .roundedRect
        field.placeholder = placeholder
        field.autocapitalizationType = .none
        field.autocorrectionType = .no
        field.font = .monospacedSystemFont(ofSize: 15, weight: .regular)
    }

    @objc private func typeChanged() {
        refreshForType()
    }

    private func refreshForType() {
        guard let type = LinkType(rawValue: typeControl.selectedSegmentIndex) else { return }
        detailLabel.text = type.detail
        detailLabel.textColor = .secondaryLabel

        // Coexistence profiles 4/5 use this app over their WiFi side, so the
        // UDP fields apply there too.
        // Các hồ sơ chung sống 4/5 dùng app này qua phía WiFi, nên ô UDP cũng
        // áp dụng cho chúng.
        let isUDP = (type == .wifiSoftAP || type == .wifiStation
                     || type == .wifiApBle || type == .wifiStaBle)
        hostField.isHidden = !isUDP
        devicePortField.isHidden = !isUDP
        appPortField.isHidden = !isUDP
        hostField.text = LinkSettings.host(for: type)
        devicePortField.text = String(LinkSettings.devicePort)
        appPortField.text = String(LinkSettings.appPort)

        bleNameField.isHidden = !type.servesBle
        bleNameField.text = LinkSettings.bleName
        bleNoteLabel.isHidden = !type.servesBle
        deviceTable.isHidden = !type.servesBle
        connectDeviceButton.isHidden = !type.servesBle
        if type.servesBle {
            bleNoteLabel.text = "Tap a device in the list to choose it, then Connect.\n"
                + "Or type a name: blank = first match, the field narrows the scan."
            startScanning()
        } else {
            scanner?.stop()
        }
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        scanner?.stop()
    }

    private func startScanning() {
        if scanner == nil {
            scanner = PickerScanner()
            scanner?.onDevices = { [weak self] rows in
                guard let self = self else { return }
                self.devices = rows
                // Keep the checkmark honest while the list reorders underneath.
                // Giữ dấu tick trung thực khi danh sách sắp xếp lại bên dưới.
                self.deviceTable.reloadData()
            }
        }
        scanner?.start()
    }

    // MARK: - Device list

    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return devices.count
    }

    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = UITableViewCell(style: .subtitle, reuseIdentifier: nil)
        let device = devices[indexPath.row]
        cell.textLabel?.text = device.name
        cell.detailTextLabel?.text = "\(device.rssi) dBm · \(device.id)"
        cell.detailTextLabel?.font = .monospacedSystemFont(ofSize: 11, weight: .regular)
        cell.accessoryType = device.id == selectedId ? .checkmark : .none
        return cell
    }

    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        let device = devices[indexPath.row]
        selectedId = device.id
        bleNameField.text = device.name
        tableView.deselectRow(at: indexPath, animated: true)
        tableView.reloadData()
    }

    @objc private func nameEdited() {
        // A hand-typed name must not inherit a picked identifier — only a tap
        // on a row pins the exact device.
        // Tên gõ tay không được thừa hưởng định danh đã chọn — chỉ cú chạm vào
        // hàng mới chốt đúng thiết bị.
        selectedId = nil
        deviceTable.reloadData()
    }

    @objc private func connectPicked() {
        guard selectedId != nil else { return }
        apply()
    }

    @objc private func cancel() {
        dismiss(animated: true)
    }

    @objc private func apply() {
        guard let type = LinkType(rawValue: typeControl.selectedSegmentIndex) else { return }
        LinkSettings.linkType = type
        if type == .wifiSoftAP || type == .wifiStation || type == .wifiApBle || type == .wifiStaBle {
            if let host = hostField.text, !host.isEmpty {
                LinkSettings.setHost(host, for: type)
            }
            if let text = devicePortField.text, let port = UInt16(text), port > 0 {
                LinkSettings.devicePort = port
            }
            if let text = appPortField.text, let port = UInt16(text), port > 0 {
                LinkSettings.appPort = port
            }
        }
        if type.servesBle {
            LinkSettings.bleName = bleNameField.text ?? ""
            // The picked identifier only counts while it still names the row
            // the field shows — a hand-typed name must not inherit it.
            // Định danh đã chọn chỉ có hiệu lực khi nó vẫn là hàng mà ô tên
            // hiển thị — tên gõ tay không được thừa hưởng nó.
            if let picked = selectedId,
               let row = devices.first(where: { $0.id == picked }),
               row.name == (bleNameField.text ?? "") {
                LinkSettings.bleId = picked
            } else {
                LinkSettings.bleId = ""
            }
        }
        onApply?()
        dismiss(animated: true)
    }

    /// Sending LINK reboots the board, so this always confirms first — a blind
    /// send could kill a pilot's live link on a coexistence profile.
    /// Gửi LINK sẽ khởi động lại bo mạch, nên bước này luôn xác nhận trước —
    /// gửi mù có thể cắt link đang dùng của người lái trên hồ sơ chung sống.
    @objc private func switchProfileTapped() {
        guard let type = LinkType(rawValue: typeControl.selectedSegmentIndex) else { return }
        let mode = type.firmwareLinkMode
        let alert = UIAlertController(
            title: "Switch drone link profile?",
            message: "The drone will switch to \"\(type.title)\" (LINK \(mode)) "
                + "and reboot in about 1.5 seconds. Any open connection will drop. "
                + "This is refused while the drone is armed.",
            preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        alert.addAction(UIAlertAction(title: "Switch", style: .destructive) { [weak self] _ in
            self?.onSwitchLinkProfile?(mode)
        })
        present(alert, animated: true)
    }
}

/// Live BLE list for the connection picker. Owns its own CBCentralManager so it
/// can scan while no link exists yet, and reports whole lists — drone-like
/// names first, then strongest signal — every time something new turns up.
/// Danh sách BLE sống cho trình chọn kết nối. Sở hữu CBCentralManager riêng để
/// quét được khi chưa có link nào, và báo cả danh sách — tên giống drone lên
/// trước, sau đó là tín hiệu mạnh nhất — mỗi khi có thiết bị mới.
final class PickerScanner: NSObject, CBCentralManagerDelegate {

    /// (name, identifier, rssi) rows, already sorted.
    /// Các hàng (name, identifier, rssi), đã sắp xếp.
    var onDevices: (([(name: String, id: String, rssi: Int)]) -> Void)?

    private var central: CBCentralManager!
    private var found: [String: (name: String, id: String, rssi: Int)] = [:]

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: .main)
    }

    func start() {
        guard central.state == .poweredOn else {
            // centralManagerDidUpdateState starts the scan once the radio is up.
            // centralManagerDidUpdateState sẽ bắt đầu quét khi radio sẵn sàng.
            return
        }
        found.removeAll()
        central.scanForPeripherals(withServices: nil, options: nil)
    }

    func stop() {
        central.stopScan()
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            start()
        }
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        let id = peripheral.identifier.uuidString
        found[id] = (peripheral.name ?? "(no name)", id, RSSI.intValue)
        let rows = found.values.sorted { a, b in
            let da = BleLink.isDroneName(a.name)
            let db = BleLink.isDroneName(b.name)
            if da != db {
                return da
            }
            return a.rssi > b.rssi
        }
        onDevices?(rows)
    }
}
