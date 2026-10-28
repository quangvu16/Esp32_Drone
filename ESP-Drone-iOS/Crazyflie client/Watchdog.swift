//
//  Watchdog.swift
//  Crazyflie client
//
//  Created by Arnaud Taffanel on 31/07/15.
//  Copyright (c) 2015 Bitcraze. All rights reserved.
//

import Foundation
import UIKit

/**
    Simple watchdog class based on NSTimer

    After starting it, reset should be called periodically befor the
    period (in seconds) is expired. Otherwise the onTimeout function
    will be called.
*/
/**
    Lớp watchdog đơn giản dựa trên NSTimer

    Sau khi khởi động, cần gọi reset định kỳ trước khi chu kỳ (tính bằng
    giây) hết hạn. Nếu không, hàm onTimeout sẽ được gọi.
*/
class Watchdog : NSObject
{
    fileprivate var started = false
    fileprivate var timer: Timer! = nil
    var onTimeout: (()->())?
    var period: Double
    
    init(period: Double, onTimeout: @escaping ()->()) {
        self.period = period
        self.onTimeout = onTimeout
    }
    
    /// Start the watchdog
    /// Bắt đầu watchdog
    func start() {
        if self.started {
            return
        }
        
        self.timer = Timer.scheduledTimer(timeInterval: self.period, target:self, selector: #selector(timeout), userInfo: nil, repeats: false)
        self.started = true
    }
    
    @objc
    private func timeout(timer: Timer) {
        self.onTimeout?()
        self.stop()
    }
    
    /// Stop the watchdog
    /// Dừng watchdog
    func stop() {
        if !self.started {
            return
        }
        
        self.timer.invalidate()
        self.timer = nil
        self.started = false
    }
    
    /**
        Reset the timer to the preset period. Optionally reset it to an higher period
        (for example just before running an operation which is known to take longer
        time)
    
        - parameter period: Nil to use the default period (configured when creating the 
                watchdog and contained in the period property). Otherwise should be
                the period in seconds
    */
    /**
        Đặt lại timer về chu kỳ đã cài. Có thể tùy chọn đặt lại với chu kỳ dài hơn
        (ví dụ ngay trước khi chạy một thao tác được biết là mất nhiều thời gian hơn)

        - parameter period: Truyền nil để dùng chu kỳ mặc định (được cấu hình khi
                tạo watchdog và lưu trong thuộc tính period). Nếu không thì phải là
                chu kỳ tính bằng giây
    */
    func reset(_ period: Double? = nil) {
        guard self.started == true else { return }
        
        let candidatePeriod = period ?? self.period
        
        self.timer.invalidate()
        self.timer = Timer.scheduledTimer(timeInterval: candidatePeriod, target:self, selector: #selector(timeout), userInfo: nil, repeats: false)
    }
    
}
