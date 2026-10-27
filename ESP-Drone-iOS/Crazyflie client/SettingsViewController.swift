//
//  SettingsViewController.swift
//  Crazyflie client
//
//  Created by Martin Eberl on 24.01.17.
//  Copyright © 2017 Bitcraze. All rights reserved.
//

import UIKit

final class SettingsViewController: UIViewController {
    var viewModel: SettingsViewModel?
    
    @IBOutlet weak var pitchrollSensitivity: UITextField!
    @IBOutlet weak var thrustSensitivity: UITextField!
    @IBOutlet weak var yawSensitivity: UITextField!
    @IBOutlet weak var sensitivitySelector: UISegmentedControl!
    @IBOutlet weak var controlModeSelector: UISegmentedControl!
    
    @IBOutlet weak var leftXLabel: UILabel!
    @IBOutlet weak var leftYLabel: UILabel!
    @IBOutlet weak var rightXLabel: UILabel!
    @IBOutlet weak var rightYLabel: UILabel!
    @IBOutlet weak var closeButton: UIButton!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        setupUI()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        viewModel?.delegate = self
        updateUI()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        viewModel?.delegate = nil
    }
    
    // MARK: - Private Methods
    
    private func setupUI() {
        closeButton.layer.borderColor = closeButton.tintColor.cgColor
        
        if MotionLink().canAccessMotion {
            controlModeSelector.insertSegment(withTitle: "Tilt Mode", at: 4, animated: true)
        }
    }
    
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        print("Operation input box: \(textField)")
        textField.becomeFirstResponder()
        return true
    }
    
    fileprivate func updateUI() {
        guard let viewModel = viewModel else {
            return
        }
        
        sensitivitySelector.selectedSegmentIndex = viewModel.sensitivity.index
        controlModeSelector.selectedSegmentIndex = viewModel.controlMode.index
        
        leftXLabel.text = viewModel.leftXTitle
        leftYLabel.text = viewModel.leftYTitle
        rightXLabel.text = viewModel.rightXTitle
        rightYLabel.text = viewModel.rightYTitle
        
        if let pitch = viewModel.pitch {
            pitchrollSensitivity.text = String(describing: pitch)
        }
        if let thrust = viewModel.thrust {
            thrustSensitivity.text = String(describing: thrust)
        }
        if let yaw = viewModel.yaw {
            yawSensitivity.text = String(describing: yaw)
        }
    }
    
    fileprivate func changeData() {
        var sensitivity = Dictionary<String, Any>()
        print("1223", thrustSensitivity.text as Any, "", pitchrollSensitivity.text as Any, "", yawSensitivity.text as Any)
        var thrustNumber: Float?
        var pitchNumber: Float?
        var yawNumber: Float?

        if let thrustDouble = Float(thrustSensitivity.text!) {
            thrustNumber = thrustDouble
        }
        if let pitchDouble = Float(pitchrollSensitivity.text!) {
            pitchNumber = pitchDouble
        }
        if let yawDouble = Float(yawSensitivity.text!) {
            yawNumber = yawDouble
        }
        sensitivity.updateValue(thrustNumber!, forKey: "maxThrust")
        sensitivity.updateValue(pitchNumber!, forKey: "pitchRate")
        sensitivity.updateValue(yawNumber!, forKey: "yawRate")
        let settings = viewModel?.sensitivity.dataDic(dic: sensitivity)
        print(sensitivity, settings as Any)
        viewModel?.sensitivity.save(settings: settings!)
    }
    
    @IBAction func sensitivityModeChanged(_ sender: Any) {
        changeData()
        viewModel?.didSetSensitivityMode(at: sensitivitySelector.selectedSegmentIndex)
    }
    
    @IBAction func controlModeChanged(_ sender: Any) {
        changeData()
        viewModel?.didSetControlMode(at: controlModeSelector.selectedSegmentIndex)
    }
    
    @IBAction func closeClicked(_ sender: Any) {
        changeData()
        dismiss(animated: true, completion: nil)
    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        print("Click on the page touchesBegan")
        changeData()
        view.endEditing(true)
    }
}

extension SettingsViewController: SettingsViewModelDelegate {
    func didUpdate() {
        updateUI()
    }
}
