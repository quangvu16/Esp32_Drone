//
//  AppDelegate.swift
//  golfclub-liebenau
//
//  Created by Martin Eberl on 01.08.16.
//  Copyright © 2016 Martin Eberl. All rights reserved.
//

import UIKit

@UIApplicationMain
class AppDelegate: UIResponder, UIApplicationDelegate {

    var window: UIWindow?


    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        let defaults = UserDefaults.standard
        defaults.set(true, forKey: "isYawOnOff")
        defaults.synchronize()
        return true
    }

    func applicationWillResignActive(_ application: UIApplication) {
        // Sent when the application is about to move from active to inactive state. This can occur for certain types of temporary interruptions (such as an incoming phone call or SMS message) or when the user quits the application and it begins the transition to the background state.
        // Use this method to pause ongoing tasks, disable timers, and throttle down OpenGL ES frame rates. Games should use this method to pause the game.
        // Được gọi khi ứng dụng sắp chuyển từ active sang inactive. Việc này có thể xảy ra
        // do một số kiểu gián đoạn tạm thời (như cuộc gọi hay tin nhắn SMS đến) hoặc khi
        // người dùng thoát ứng dụng và ứng dụng bắt đầu chuyển sang nền.
        // Dùng phương thức này để tạm dừng các tác vụ đang chạy, tắt timer và giảm tốc độ
        // khung hình OpenGL ES. Trò chơi nên dùng phương thức này để tạm dừng game.
    }

    func applicationDidEnterBackground(_ application: UIApplication) {
        // Use this method to release shared resources, save user data, invalidate timers, and store enough application state information to restore your application to its current state in case it is terminated later.
        // If your application supports background execution, this method is called instead of applicationWillTerminate: when the user quits.
        // Dùng phương thức này để giải phóng tài nguyên dùng chung, lưu dữ liệu người dùng,
        // vô hiệu hóa timer và lưu đủ thông tin trạng thái để khôi phục ứng dụng về trạng
        // thái hiện tại nếu ứng dụng bị kết thúc sau đó.
        // Nếu ứng dụng hỗ trợ chạy nền, phương thức này được gọi thay cho
        // applicationWillTerminate: khi người dùng thoát.
    }

    func applicationWillEnterForeground(_ application: UIApplication) {
        // Called as part of the transition from the background to the inactive state; here you can undo many of the changes made on entering the background.
        // Được gọi trong quá trình chuyển từ nền sang inactive; tại đây bạn có thể hoàn tác
        // nhiều thay đổi đã thực hiện khi vào nền.
    }

    func applicationDidBecomeActive(_ application: UIApplication) {
        // Restart any tasks that were paused (or not yet started) while the application was inactive. If the application was previously in the background, optionally refresh the user interface.
        // Khởi động lại các tác vụ đã bị tạm dừng (hoặc chưa kịp chạy) trong khi ứng dụng ở
        // trạng thái inactive. Nếu ứng dụng trước đó ở nền, có thể làm mới giao diện người dùng.
    }

    func applicationWillTerminate(_ application: UIApplication) {
        // Called when the application is about to terminate. Save data if appropriate. See also applicationDidEnterBackground:.
        // Được gọi khi ứng dụng sắp bị kết thúc. Lưu dữ liệu nếu cần. Xem thêm
        // applicationDidEnterBackground:.
    }
}

