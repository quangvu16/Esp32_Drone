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
//  This is the whole migration. UIKit hands us a scene, we build the same
//  window from the same storyboard. Nothing else about the app changes.
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
