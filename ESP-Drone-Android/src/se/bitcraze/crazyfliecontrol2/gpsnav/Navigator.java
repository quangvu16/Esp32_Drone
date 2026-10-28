package se.bitcraze.crazyfliecontrol2.gpsnav;

import java.io.IOException;
import java.util.Locale;

/**
 * GPS mission navigation state machine, ported from the OpenBot {@code gpsnav} navigator — with the
 * differential-drive steering law dropped: the drone has its own tilt controller and only needs to
 * be told where to go (PROTOCOL.md section 10).
 *
 * <p>Progress is measured against the DRONE's dead-reckoned launch-frame position ({@code
 * nav.pos_x}/{@code nav.pos_y}), never the phone's GPS: the pilot may walk away, and a
 * phone-position arrival check would fire while the drone is nowhere near the waypoint. The phone's
 * GPS only defines the plan and the launch frame. One asymmetry (PROTOCOL.md section 10): in FOLLOW
 * the target itself may be the phone, so the phone is the target source while the drone's estimate
 * remains the progress source.
 *
 * <p>Máy trạng thái điều hướng sứ mệnh GPS, chuyển thể từ trình điều hướng {@code gpsnav} của
 * OpenBot — lược bỏ luật lái dẫn động sai phân: drone có bộ điều khiển nghiêng riêng và chỉ cần
 * được cho biết đi đâu (PROTOCOL.md mục 10).
 *
 * <p>Tiến độ được đo theo vị trí trong khung cất cánh do drone tự dẫn đường quán
 * tính ({@code nav.pos_x}/{@code nav.pos_y}), không bao giờ theo GPS của điện thoại: người điều
 * khiển có thể đi chỗ khác, khi đó phép kiểm tra đến nơi theo vị trí điện thoại sẽ kích hoạt dù
 * drone còn ở rất xa điểm hành trình. GPS của điện thoại chỉ định nghĩa kế hoạch và khung quy
 * chiếu lúc cất cánh. Một điểm bất đối xứng (PROTOCOL.md mục 10): trong FOLLOW, bản thân mục tiêu
 * có thể là điện thoại, nên điện thoại là nguồn cung cấp mục tiêu trong khi ước tính của drone vẫn
 * là nguồn đo tiến độ.
 */
public class Navigator {

    /** Mission planner flight modes, mirroring the OpenBot enum. */
    /** Các chế độ bay của mission planner, tương ứng với enum của OpenBot. */
    public enum NavMode {
        IDLE,
        MISSION,
        GUIDED,
        RTL
    }

    /**
     * Hold rather than navigate on a phone fix worse than this (PROTOCOL.md section 10). Navigating
     * on a bad fix puts the target in the wrong place; waiting costs nothing.
     *
     * <p>Giữ nguyên thay vì điều hướng khi vị trí GPS của điện thoại kém hơn mức này (PROTOCOL.md
     * mục 10). Điều hướng trên một vị trí sai sẽ đặt mục tiêu nhầm chỗ; chờ đợi thì không mất gì.
     */
    public static final float MAX_POSITION_ACCURACY_M = 15.0f;

    /** Outcome of one navigation tick. Distances are metres, bearings are compass degrees. */
    /** Kết quả của một nhịp điều hướng. Khoảng cách tính bằng mét, phương hướng tính bằng độ la bàn. */
    public static final class NavStatus {

        private final NavMode mMode;
        private final float mDistanceMeters;
        private final float mBearingDegrees;
        private final float mTurnErrorDegrees;
        private final int mActiveWaypointIndex;
        private final boolean mArrived;
        private final String mMessage;

        NavStatus(NavMode mode, float distanceMeters, float bearingDegrees, float turnErrorDegrees,
                int activeWaypointIndex, boolean arrived, String message) {
            this.mMode = mode;
            this.mDistanceMeters = distanceMeters;
            this.mBearingDegrees = bearingDegrees;
            this.mTurnErrorDegrees = turnErrorDegrees;
            this.mActiveWaypointIndex = activeWaypointIndex;
            this.mArrived = arrived;
            this.mMessage = message == null ? "" : message;
        }

        public NavMode getMode() {
            return mMode;
        }

        /** Distance to the current target in metres, NaN when there is no target. */
        /** Khoảng cách tới mục tiêu hiện tại tính bằng mét, NaN khi không có mục tiêu. */
        public float getDistanceMeters() {
            return mDistanceMeters;
        }

        /** Compass bearing to the current target in degrees, NaN when there is no target. */
        /** Phương hướng la bàn tới mục tiêu hiện tại tính bằng độ, NaN khi không có mục tiêu. */
        public float getBearingDegrees() {
            return mBearingDegrees;
        }

        /** Turn from the drone's heading onto the target bearing; positive = turn right. */
        /** Góc quay từ hướng mũi drone sang phương hướng mục tiêu; dương = rẽ phải. */
        public float getTurnErrorDegrees() {
            return mTurnErrorDegrees;
        }

        /** Index of the waypoint being sought (before any advance), -1 when not in MISSION. */
        /** Chỉ số của điểm hành trình đang tìm (trước khi chuyển tiếp), -1 khi không ở MISSION. */
        public int getActiveWaypointIndex() {
            return mActiveWaypointIndex;
        }

        public boolean isArrived() {
            return mArrived;
        }

        public String getMessage() {
            return mMessage;
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "NavStatus(%s dist=%.1fm bearing=%.0f %s)",
                    mMode, mDistanceMeters, mBearingDegrees, mMessage);
        }
    }

    private Mission mMission = new Mission();
    private NavMode mMode = NavMode.IDLE;
    private boolean mPaused;
    private int mActiveIndex;
    private Waypoint mGuidedTarget;
    private NavStatus mLastStatus;

    /** Compass bearing of the nose at ALIGN, NaN until the launch frame exists. */
    /** Phương hướng la bàn của mũi drone lúc ALIGN, NaN cho tới khi khung cất cánh tồn tại. */
    private double mLaunchHeadingDeg = Double.NaN;

    /** The drone's compass heading from {@code flight.heading}, NaN until the first log block. */
    /** Hướng mũi la bàn của drone lấy từ {@code flight.heading}, NaN cho tới khi có khối log đầu tiên. */
    private float mDroneHeadingDeg = Float.NaN;

    // region mission bookkeeping

    public Navigator() {
    }

    public Navigator(Mission mission) {
        loadMission(mission);
    }

    public Mission getMission() {
        return mMission;
    }

    public boolean isNavigating() {
        return mMode != NavMode.IDLE;
    }

    public boolean isPaused() {
        return mPaused;
    }

    public NavMode getMode() {
        return mMode;
    }

    public int getActiveIndex() {
        return mActiveIndex;
    }

    public NavStatus getLastStatus() {
        return mLastStatus;
    }

    /** Records the launch position RTL returns to and every target offset is measured from. */
    /** Ghi lại vị trí cất cánh mà RTL quay về, cũng là gốc để đo mọi độ lệch mục tiêu. */
    public void markHome(double latitude, double longitude) {
        mMission.setHome(latitude, longitude);
    }

    /**
     * Fixes the launch frame (PROTOCOL.md section 6): {@code alphaDeg} is the compass bearing of the
     * nose at ALIGN. Needed to compare the drone's launch-frame position against waypoints.
     *
     * <p>Cố định khung cất cánh (PROTOCOL.md mục 6): {@code alphaDeg} là phương hướng
     * la bàn của mũi drone lúc ALIGN. Cần để so sánh vị trí của drone trong khung cất cánh
     * với các điểm hành trình.
     */
    public void setLaunchHeading(double alphaDeg) {
        mLaunchHeadingDeg = alphaDeg;
    }

    /** Feeds {@code flight.heading} in so {@link NavStatus#getTurnErrorDegrees()} can be computed. */
    /** Nạp {@code flight.heading} vào để tính được {@link NavStatus#getTurnErrorDegrees()}. */
    public void setDroneHeading(float headingDeg) {
        mDroneHeadingDeg = headingDeg;
    }

    public void loadMission(Mission mission) {
        mMission = mission != null ? mission : new Mission();
        mActiveIndex = 0;
    }

    /** Loads a Mission Planner {@code .waypoints} document. Returns the mission now held. */
    /** Nạp tài liệu {@code .waypoints} của Mission Planner. Trả về sứ mệnh đang được giữ. */
    public Mission loadMissionFile(String contents) throws IOException {
        loadMission(Mission.parseWaypointsFile(contents));
        return mMission;
    }

    /** Serialises the current mission to the {@code .waypoints} format. */
    /** Tuần tự hoá sứ mệnh hiện tại sang định dạng {@code .waypoints}. */
    public String saveMissionFile() {
        return mMission.toWaypointsFile();
    }

    /** The waypoint being sought right now, or null when idle. */
    /** Điểm hành trình đang tìm ngay lúc này, hoặc null khi đang nhàn rỗi. */
    public Waypoint currentTarget() {
        if (mMode == NavMode.MISSION) {
            return mMission.getWaypoint(mActiveIndex);
        }
        return mGuidedTarget;
    }

    // endregion

    // region mode control

    /** MISSION: fly every waypoint in order. Returns false if there are none. */
    /** MISSION: bay hết các điểm hành trình theo thứ tự. Trả về false nếu không có điểm nào. */
    public boolean startMission() {
        if (mMission.isEmpty()) {
            return false;
        }
        mActiveIndex = 0;
        mGuidedTarget = null;
        mMode = NavMode.MISSION;
        mPaused = false;
        return true;
    }

    /** GUIDED: fly to a single target ("go to here"). */
    /** GUIDED: bay tới một mục tiêu duy nhất ("đi tới đây"). */
    public boolean startGuided(double latitude, double longitude) {
        mGuidedTarget = new Waypoint(
                latitude, longitude, "", Waypoint.DEFAULT_ACCEPTANCE_RADIUS_M);
        mMode = NavMode.GUIDED;
        mPaused = false;
        return true;
    }

    /** RTL: return to the recorded launch position. */
    /** RTL: quay về vị trí cất cánh đã ghi lại. */
    public boolean startRtl() {
        if (!mMission.hasHome()) {
            return false;
        }
        mGuidedTarget = new Waypoint(
                mMission.getHomeLatitude(),
                mMission.getHomeLongitude(),
                "Home",
                Waypoint.DEFAULT_ACCEPTANCE_RADIUS_M);
        mMode = NavMode.RTL;
        mPaused = false;
        return true;
    }

    /**
     * Holds the current state. The caller stops sending TARGET packets; the firmware stops chasing
     * after its 3 s target timeout (PROTOCOL.md section 6).
     *
     * <p>Giữ nguyên trạng thái hiện tại. Bên gọi ngừng gửi các gói tin TARGET; firmware ngừng đuổi
     * theo sau khi hết thời gian chờ mục tiêu 3 giây (PROTOCOL.md mục 6).
     */
    public void pause() {
        if (isNavigating()) {
            mPaused = true;
        }
    }

    public void resume() {
        mPaused = false;
    }

    /** Disarms navigation and returns the drone to idle/hold. */
    /** Ngắt điều hướng và đưa drone về idle/hold. */
    public void stop() {
        mMode = NavMode.IDLE;
        mPaused = false;
        mGuidedTarget = null;
        mActiveIndex = 0;
    }

    // endregion

    // region stepping

    /**
     * Computes one navigation tick from the drone's launch-frame position.
     *
     * <p>Tính một nhịp điều hướng từ vị trí của drone trong khung cất cánh.
     *
     * @param posX               drone {@code nav.pos_x} in metres
     * @param posX               {@code nav.pos_x} của drone, tính bằng mét
     * @param posY               drone {@code nav.pos_y} in metres
     * @param posY               {@code nav.pos_y} của drone, tính bằng mét
     * @param phoneAccuracyMeters horizontal accuracy of the phone fix; NaN when unknown
     * @param phoneAccuracyMeters độ chính xác ngang của vị trí GPS điện thoại; NaN khi không rõ
     */
    public NavStatus step(float posX, float posY, float phoneAccuracyMeters) {
        if (!isNavigating() || mPaused) {
            return record(status(mMode, Float.NaN, Float.NaN, Float.NaN, -1, false,
                    mPaused ? "paused" : "idle"));
        }

        if (!Float.isNaN(phoneAccuracyMeters) && phoneAccuracyMeters > MAX_POSITION_ACCURACY_M) {
            // Targeting from a bad fix is worse than waiting: hold and say so out loud.
            // Chọn mục tiêu từ một vị trí sai còn tệ hơn là chờ: giữ nguyên và nói rõ điều đó.
            return record(status(mMode, Float.NaN, Float.NaN, Float.NaN, mActiveIndex, false,
                    String.format(Locale.US, "GPS accuracy %.1f m, holding", phoneAccuracyMeters)));
        }

        if (Double.isNaN(mLaunchHeadingDeg) || !mMission.hasHome()) {
            // Without ALIGN there is no launch frame to compare in; never navigate on NaN.
            // Không có ALIGN thì không có khung cất cánh để so sánh; không bao giờ điều hướng trên giá trị NaN.
            return record(status(mMode, Float.NaN, Float.NaN, Float.NaN, mActiveIndex, false,
                    "launch frame not set (ALIGN first), holding"));
        }

        Waypoint target = currentTarget();
        if (target == null) {
            stop();
            return record(status(NavMode.IDLE, Float.NaN, Float.NaN, Float.NaN, -1, false,
                    "no target"));
        }

        float[] targetXY = target.toLaunchFrame(
                mMission.getHomeLatitude(), mMission.getHomeLongitude(), mLaunchHeadingDeg);
        float dx = targetXY[0] - posX;
        float dy = targetXY[1] - posY;
        float distance = (float) Math.hypot(dx, dy);

        // +y is LEFT of the nose, so atan2(dy, dx) counts the wrong way round for compass
        // (compass increases clockwise, toward the right): the clockwise bearing from the nose
        // is -atan2(dy, dx), and converting that to compass adds alpha.
        // +y nằm BÊN TRÁI mũi drone, nên atan2(dy, dx) đếm ngược chiều so với la bàn
        // (la bàn tăng theo chiều kim đồng hồ, sang bên phải): phương hướng theo chiều kim đồng
        // hồ tính từ mũi là -atan2(dy, dx), và đổi sang la bàn thì cộng thêm alpha.
        float bearing = (float) GeoMath.wrapDegrees(
                mLaunchHeadingDeg - Math.toDegrees(Math.atan2(dy, dx)));
        float turnError = Float.isNaN(mDroneHeadingDeg)
                ? Float.NaN
                : (float) GeoMath.turnErrorDegrees(mDroneHeadingDeg, bearing);

        // Snapshot before advancing, so the status names the waypoint just hit.
        // Chụp trạng thái trước khi chuyển tiếp, để status gọi tên điểm hành trình vừa chạm tới.
        int reachedIndex = mActiveIndex;

        if (distance <= target.getAcceptanceRadiusMeters()) {
            onArrived();
            return record(status(mMode, distance, bearing, turnError, reachedIndex, true,
                    "waypoint reached"));
        }

        return record(status(mMode, distance, bearing, turnError, reachedIndex, false, ""));
    }

    private void onArrived() {
        if (mMode == NavMode.MISSION) {
            mActiveIndex++;
            if (mActiveIndex >= mMission.size()) {
                // Last waypoint: mission complete, back to idle.
                // Điểm hành trình cuối: sứ mệnh hoàn tất, quay về idle.
                stop();
            }
        } else {
            // GUIDED and RTL are one-shot: arriving disarms them.
            // GUIDED và RTL là các lệnh dùng một lần (one-shot): đến nơi là tự ngắt.
            stop();
        }
    }

    private static NavStatus status(NavMode mode, float distance, float bearing, float turnError,
            int activeIndex, boolean arrived, String message) {
        return new NavStatus(mode, distance, bearing, turnError, activeIndex, arrived, message);
    }

    private NavStatus record(NavStatus status) {
        mLastStatus = status;
        return status;
    }

    // endregion
}
