//
//  ConnectionViewController.swift
//  Crazyflie client
//
//  Connection-method picker. Replaces the hard-coded UDP endpoints with a
//  user-configurable transport + address (PROTOCOL.md §8). The choice is
//  persisted in LinkSettings and the ViewModel rebuilds the driver from it.
//

import UIKit

final class ConnectionViewController: UIViewController {

    /// Called when the user saves: the app should tear down and rebuild its
    /// link with the new settings.
    var onApply: (() -> Void)?

    private let typeControl = UISegmentedControl(items: LinkType.allCases.map { $0.title })
    private let detailLabel = UILabel()
    private let hostField = UITextField()
    private let devicePortField = UITextField()
    private let appPortField = UITextField()
    private let bleListLabel = UILabel()
    private let scrollView = UIScrollView()
    private let stack = UIStackView()

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

        bleListLabel.numberOfLines = 0
        bleListLabel.font = .monospacedSystemFont(ofSize: 12, weight: .regular)
        bleListLabel.textColor = .secondaryLabel
        stack.addArrangedSubview(bleListLabel)

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

        let isUDP = (type == .wifiSoftAP || type == .wifiStation)
        hostField.isHidden = !isUDP
        devicePortField.isHidden = !isUDP
        appPortField.isHidden = !isUDP
        hostField.text = LinkSettings.host(for: type)
        devicePortField.text = String(LinkSettings.devicePort)
        appPortField.text = String(LinkSettings.appPort)

        bleListLabel.isHidden = type != .bleGatt
        if type == .bleGatt {
            bleListLabel.text = "Scanning for drones advertising the Bitcraze service…\n" +
                "(first match is used; check Settings to name one)"
        }
        if type == .bluetoothSPP {
            detailLabel.text = type.detail + "\n\n" + SppLink.unavailableReason
            detailLabel.textColor = .systemRed
        } else {
            detailLabel.textColor = .secondaryLabel
        }
    }

    @objc private func cancel() {
        dismiss(animated: true)
    }

    @objc private func apply() {
        guard let type = LinkType(rawValue: typeControl.selectedSegmentIndex) else { return }
        if type == .bluetoothSPP {
            let alert = UIAlertController(
                title: "Not available on iOS",
                message: SppLink.unavailableReason,
                preferredStyle: .alert)
            alert.addAction(UIAlertAction(title: "OK", style: .default))
            present(alert, animated: true)
            return
        }
        LinkSettings.linkType = type
        if type == .wifiSoftAP || type == .wifiStation {
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
        onApply?()
        dismiss(animated: true)
    }
}
