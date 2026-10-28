//
//  SceneDelegate.swift
//  Crazyflie client
//
//  The app used to get its window from UIMainStoryboardFile via the app
//  delegate's `window`. UIKit still does that for pre-scene apps, but apps
//  linked against the current SDK must adopt the scene lifecycle or fail the
//  launch check outright: UIKit traps in
//  _UIApplicationEvaluateRuntimeIssueForNoSceneLifecycleAdoption and the
//  process dies before any of our code runs. That is why the test host could
//  never connect.
//
//  Ứng dụng trước đây lấy window từ UIMainStoryboardFile qua thuộc tính `window`
//  của app delegate. UIKit vẫn làm vậy với các ứng dụng chưa dùng scene, nhưng
//  ứng dụng liên kết với SDK hiện tại phải áp dụng vòng đời scene, nếu không sẽ
//  hỏng ngay bước kiểm tra khởi động: UIKit bắt lỗi tại
//  _UIApplicationEvaluateRuntimeIssueForNoSceneLifecycleAdoption và tiến trình
//  chết trước khi bất kỳ mã nào của chúng ta chạy. Đó là lý do test host không
//  bao giờ kết nối được.
//
//  This is the whole migration. UIKit hands us a scene, we build the same
//  window from the same storyboard. Nothing else about the app changes.
//
//  Toàn bộ quá trình chuyển đổi chỉ có vậy. UIKit giao cho ta một scene, ta
//  dựng window y hệt từ cùng một storyboard. Không có gì khác trong ứng dụng
//  thay đổi.
//

import UIKit

class SceneDelegate: UIResponder, UIWindowSceneDelegate {

    var window: UIWindow?

    func scene(_ scene: UIScene,
               willConnectTo session: UISceneSession,
               options connectionOptions: UIScene.ConnectionOptions) {
        guard let windowScene = scene as? UIWindowScene else { return }

        let window = UIWindow(windowScene: windowScene)
        window.rootViewController = UIStoryboard(name: "Main", bundle: nil)
            .instantiateInitialViewController()
        self.window = window
        window.makeKeyAndVisible()
    }
}
