package se.bitcraze.crazyfliecontrol2.gpsnav;

/**
 * Process-wide navigation store shared by the Mission screen (GpsNavActivity) and the Indoor Map
 * screen (IndoorMapActivity). One {@link Navigator} holds the plan and the flight state, so zones
 * drawn on the Indoor Map are enforced by the Mission runner, and one GUIDED "go to" target set on
 * either screen is seen by both.
 *
 * <p>Kho lưu điều hướng dùng chung toàn process cho màn Mission (GpsNavActivity) và màn Indoor Map
 * (IndoorMapActivity). Một {@link Navigator} giữ kế hoạch và trạng thái bay, nên các vùng vẽ trên
 * Indoor Map được trình chạy sứ mệnh của Mission thực thi, và một mục tiêu GUIDED "go to" đặt ở màn
 * nào cũng được cả hai màn nhìn thấy.
 */
public final class NavStore {

    /** The one plan + flight navigator both screens edit and fly. */
    /** Kế hoạch + trình điều hướng bay duy nhất mà cả hai màn hình cùng sửa và bay. */
    private static final Navigator NAVIGATOR = new Navigator();

    /** The pending one-shot GUIDED target, or null. Immutable, so plain reads are safe. */
    /** Mục tiêu GUIDED dùng một lần đang chờ, hoặc null. Bất biến nên đọc trực tiếp là an toàn. */
    private static volatile Waypoint sGuidedTarget;

    private NavStore() {
    }

    public static Navigator navigator() {
        return NAVIGATOR;
    }

    /** The pending GUIDED "go to" target, or null when none has been tapped yet. */
    /** Mục tiêu GUIDED "go to" đang chờ, hoặc null khi chưa chạm chọn cái nào. */
    public static Waypoint getGuidedTarget() {
        return sGuidedTarget;
    }

    public static void setGuidedTarget(Waypoint target) {
        sGuidedTarget = target;
    }
}
