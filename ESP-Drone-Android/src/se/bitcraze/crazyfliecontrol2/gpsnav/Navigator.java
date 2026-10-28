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

    /** The drone's last known latitude/longitude (launch-frame back-projection), NaN until step(). */
    /** Vĩ độ/kinh độ đã biết gần nhất của drone (dựng ngược từ khung cất cánh), NaN cho tới khi step(). */
    private double mDroneLat = Double.NaN;
    private double mDroneLon = Double.NaN;

    /**
     * Pending fly-through corridor crossing, or null: when a leg crosses a corridor the drone aims
     * at the gate first and continues to the real target after crossing it (PROTOCOL.md section 10).
     *
     * <p>Điểm băng qua hành lang đang chờ, hoặc null: khi một chặng cắt qua hành lang, drone nhắm
     * vào cổng trước rồi tiếp tục tới mục tiêu thật sau khi băng qua (PROTOCOL.md mục 10).
     */
    private Waypoint mVia;

    /** Zone-hold state: the drone drifted into a keep-out zone and targeting is suppressed. */
    /** Trạng thái giữ trong vùng: drone lạc vào vùng cấm bay và việc nhắm mục tiêu bị ngưng. */
    private boolean mZoneHold;

    /** Pilot pressed Resume while held: fly on anyway. Cleared when the drone leaves the zone. */
    /** Người lái bấm Tiếp tục khi đang giữ: cứ bay tiếp. Xoá khi drone rời khỏi vùng. */
    private boolean mZoneOverride;

    /** Consecutive in-radius ticks a wind-zone arrival still needs before advancing. */
    /** Các nhịp liên tiếp trong bán kính mà việc tới vùng gió còn cần trước khi chuyển tiếp. */
    private int mArrivalConfirmTicks;

    /**
     * Ticks a wind-zone arrival must hold inside the acceptance radius before it counts. The fix
     * bounces in unsteady air; two consecutive ticks is the cheapest filter that notices.
     *
     * <p>Số nhịp việc tới vùng gió phải nằm trong bán kính chấp nhận trước khi được tính. Vị trí
     * GPS nhảy trong khí không ổn định; hai nhịp liên tiếp là bộ lọc rẻ nhất phát hiện điều đó.
     */
    private static final int ARRIVAL_CONFIRM_TICKS = 2;

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

    /**
     * True while the runner is holding the drone because it is inside a keep-out zone. The caller
     * stops sending TARGET packets — same hold mechanism as {@link #isPaused()}, but positional:
     * it clears itself when the drone leaves the zone, or via {@link #resume()} (pilot authority).
     *
     * <p>Đúng khi bộ chạy đang giữ drone vì nó ở trong vùng cấm bay. Bên gọi ngừng gửi gói TARGET —
     * cùng cơ chế giữ như {@link #isPaused()}, nhưng theo vị trí: tự xoá khi drone rời vùng, hoặc
     * qua {@link #resume()} (quyền của người lái).
     */
    public boolean isZoneHold() {
        return mZoneHold;
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

    /** The waypoint being sought right now, or null when idle. A pending corridor gate wins. */
    /** Điểm hành trình đang tìm ngay lúc này, hoặc null khi đang nhàn rỗi. Cổng hành lang đang chờ được ưu tiên. */
    public Waypoint currentTarget() {
        if (mVia != null) {
            return mVia;
        }
        if (mMode == NavMode.MISSION) {
            return mMission.getWaypoint(mActiveIndex);
        }
        return mGuidedTarget;
    }

    // region map annotations (PROTOCOL.md section 10)

    /**
     * Non-null refusal when a waypoint may not be placed here: it sits inside a keep-out zone.
     * Checked on every map tap and on every file load.
     *
     * <p>Lời từ chối khác null khi một điểm hành trình không được đặt ở đây: nó nằm trong vùng
     * cấm bay. Được kiểm tra mỗi lần chạm bản đồ và mỗi lần nạp tệp.
     */
    public String checkWaypoint(double latitude, double longitude) {
        for (MapAnnotation annotation : mMission.getAnnotations()) {
            if (annotation.getType() == MapAnnotation.Type.KEEPOUT
                    && annotation.containsPoint(latitude, longitude)) {
                return "inside keep-out zone - move the waypoint";
            }
        }
        return null;
    }

    /**
     * Non-null refusal when a planned leg may not be flown as drawn: it crosses a keep-out zone
     * and no fly-through corridor sanctions the crossing. A leg through a corridor is fine — the
     * runner re-routes it through the gate (see {@link #corridorVia}).
     *
     * <p>Lời từ chối khác null khi một chặng được lên kế hoạch không được bay theo nét vẽ: nó cắt
     * qua vùng cấm bay và không có hành lang cho phép bay qua nào chấp thuận lối băng. Chặng đi qua
     * hành lang thì được — bộ chạy định tuyến lại qua cổng (xem {@link #corridorVia}).
     */
    public String checkLeg(double fromLat, double fromLon, double toLat, double toLon) {
        for (MapAnnotation annotation : mMission.getAnnotations()) {
            if (annotation.getType() == MapAnnotation.Type.KEEPOUT
                    && annotation.crossesSegment(fromLat, fromLon, toLat, toLon)) {
                return corridorVia(fromLat, fromLon, toLat, toLon) != null
                        ? null
                        : "leg crosses a keep-out zone with no fly-through corridor";
            }
        }
        return null;
    }

    /**
     * The point where the leg crosses a fly-through corridor, or null when it crosses none. That
     * point is the via the runner targets before the real destination.
     *
     * <p>Điểm mà chặng cắt qua hành lang cho phép bay qua, hoặc null khi không cắt qua cái nào.
     * Điểm đó là trung gian mà bộ chạy nhắm tới trước đích thật.
     */
    public double[] corridorVia(double fromLat, double fromLon, double toLat, double toLon) {
        for (MapAnnotation annotation : mMission.getAnnotations()) {
            if (annotation.getType() == MapAnnotation.Type.CORRIDOR) {
                double[] hit = annotation.segmentCrossing(fromLat, fromLon, toLat, toLon);
                if (hit != null) {
                    return hit;
                }
            }
        }
        return null;
    }

    /**
     * Validates the whole plan the way {@link #startMission()} will fly it: no waypoint inside a
     * keep-out zone, no leg crossing one without a corridor. Returns the first refusal or null.
     *
     * <p>Kiểm tra toàn bộ kế hoạch đúng như {@link #startMission()} sẽ bay: không điểm hành trình
     * nào nằm trong vùng cấm bay, không chặng nào cắt qua vùng đó mà không có hành lang. Trả về
     * lời từ chối đầu tiên hoặc null.
     */
    public String checkMission() {
        if (mMission.isEmpty()) {
            return "no waypoints to fly";
        }
        double prevLat = mMission.getHomeLatitude();
        double prevLon = mMission.getHomeLongitude();
        boolean hasPrev = mMission.hasHome();
        for (int i = 0; i < mMission.size(); i++) {
            Waypoint waypoint = mMission.getWaypoint(i);
            String refusal = checkWaypoint(waypoint.getLatitude(), waypoint.getLongitude());
            if (refusal != null) {
                return String.format(Locale.US, "waypoint %d: %s", i + 1, refusal);
            }
            if (hasPrev) {
                refusal = checkLeg(prevLat, prevLon, waypoint.getLatitude(), waypoint.getLongitude());
                if (refusal != null) {
                    return String.format(Locale.US, "leg to waypoint %d: %s", i + 1, refusal);
                }
            }
            prevLat = waypoint.getLatitude();
            prevLon = waypoint.getLongitude();
            hasPrev = true;
        }
        return null;
    }

    /** The annotation of this type containing the point, or null. */
    /** Chú thích kiểu này chứa điểm đó, hoặc null. */
    private MapAnnotation annotationAt(MapAnnotation.Type type, double lat, double lon) {
        for (MapAnnotation annotation : mMission.getAnnotations()) {
            if (annotation.getType() == type && annotation.containsPoint(lat, lon)) {
                return annotation;
            }
        }
        return null;
    }

    /**
     * Arms corridor via-routing for a leg: when the straight leg crosses a fly-through corridor the
     * drone aims at the gate first (PROTOCOL.md section 10).
     *
     * <p>Gắn định tuyến qua hành lang cho một chặng: khi chặng thẳng cắt qua hành lang, drone nhắm
     * vào cổng trước (PROTOCOL.md mục 10).
     */
    private void armLeg(double fromLat, double fromLon, Waypoint to) {
        mVia = null;
        mArrivalConfirmTicks = 0;
        if (to == null || Double.isNaN(fromLat) || Double.isNaN(fromLon)) {
            return;
        }
        double[] via = corridorVia(fromLat, fromLon, to.getLatitude(), to.getLongitude());
        if (via != null) {
            // A small acceptance radius: the gate is narrow and the point is a crossing, not a stop.
            // Bán kính chấp nhận nhỏ: cổng hẹp và điểm này là nơi băng qua, không phải điểm dừng.
            mVia = new Waypoint(via[0], via[1], "corridor", 1.5f);
        }
    }

    // endregion

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
        // The plan's first leg starts at home — the drone launches from there.
        // Chặng đầu của kế hoạch bắt đầu từ home — drone cất cánh từ đó.
        armLeg(mMission.getHomeLatitude(), mMission.getHomeLongitude(), mMission.getWaypoint(0));
        return true;
    }

    /** GUIDED: fly to a single target ("go to here"). */
    /** GUIDED: bay tới một mục tiêu duy nhất ("đi tới đây"). */
    public boolean startGuided(double latitude, double longitude) {
        mGuidedTarget = new Waypoint(
                latitude, longitude, "", Waypoint.DEFAULT_ACCEPTANCE_RADIUS_M);
        mMode = NavMode.GUIDED;
        mPaused = false;
        armLeg(legOriginLat(), legOriginLon(), mGuidedTarget);
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
        armLeg(legOriginLat(), legOriginLon(), mGuidedTarget);
        return true;
    }

    /** Where a new leg starts: the drone's last known position, else the launch position. */
    /** Nơi một chặng mới bắt đầu: vị trí đã biết gần nhất của drone, nếu không thì vị trí cất cánh. */
    private double legOriginLat() {
        return Double.isNaN(mDroneLat) ? mMission.getHomeLatitude() : mDroneLat;
    }

    private double legOriginLon() {
        return Double.isNaN(mDroneLon) ? mMission.getHomeLongitude() : mDroneLon;
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
        if (mZoneHold) {
            // Pilot authority: they saw the keep-out hold and chose to fly on. Cleared when the
            // drone leaves the zone, so the next entry holds again.
            // Quyền của người lái: họ thấy cảnh báo giữ trong vùng cấm bay và chọn bay tiếp. Được
            // xoá khi drone rời vùng, nên lần vào sau sẽ giữ lại.
            mZoneOverride = true;
            mZoneHold = false;
        }
    }

    /** Disarms navigation and returns the drone to idle/hold. */
    /** Ngắt điều hướng và đưa drone về idle/hold. */
    public void stop() {
        mMode = NavMode.IDLE;
        mPaused = false;
        mGuidedTarget = null;
        mActiveIndex = 0;
        mVia = null;
        mZoneHold = false;
        mZoneOverride = false;
        mArrivalConfirmTicks = 0;
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

        // Where the drone is on the map, so the annotations can judge it (PROTOCOL.md section 10).
        // Drone đang ở đâu trên bản đồ, để các chú thích có thể đánh giá nó (PROTOCOL.md mục 10).
        double[] droneLatLon = GeoMath.launchFrameToLatLon(
                posX, posY, mLaunchHeadingDeg,
                mMission.getHomeLatitude(), mMission.getHomeLongitude());
        mDroneLat = droneLatLon[0];
        mDroneLon = droneLatLon[1];

        // Keep-out zones make the runner behave differently where they are: a drone that drifts
        // into one is held there, not chased onward through it. RTL is exempt — safety outranks
        // the hint — and Resume is the pilot's override.
        // Vùng cấm bay khiến bộ chạy cư xử khác đi tại chỗ chúng: drone lạc vào trong sẽ bị giữ
        // ở đó, không bị đuổi tiếp xuyên qua. RTL được miễn — an toàn quan trọng hơn gợi ý — và
        // nút Tiếp tục là quyền ghi đè của người lái.
        MapAnnotation keepOut = annotationAt(MapAnnotation.Type.KEEPOUT, mDroneLat, mDroneLon);
        if (keepOut == null) {
            // Left the zone: the next entry holds again.
            // Đã rời vùng: lần vào sau sẽ giữ lại.
            mZoneOverride = false;
            mZoneHold = false;
        } else if (mMode != NavMode.RTL && !mZoneOverride) {
            mZoneHold = true;
            return record(status(mMode, Float.NaN, Float.NaN, Float.NaN, mActiveIndex, false,
                    "keep-out zone entered - holding"));
        } else {
            mZoneHold = false;
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
            // Wind zones demand a settled fix before the runner advances (PROTOCOL.md section 10):
            // the fix bounces in unsteady air, so an arrival there must hold for consecutive ticks.
            // Vùng gió đòi hỏi vị trí ổn định trước khi bộ chạy chuyển tiếp (PROTOCOL.md mục 10):
            // vị trí GPS nhảy trong khí không ổn định, nên việc tới nơi ở đó phải giữ qua các nhịp
            // liên tiếp.
            if (annotationAt(MapAnnotation.Type.WIND, target.getLatitude(), target.getLongitude()) != null
                    || annotationAt(MapAnnotation.Type.WIND, mDroneLat, mDroneLon) != null) {
                mArrivalConfirmTicks++;
                if (mArrivalConfirmTicks < ARRIVAL_CONFIRM_TICKS) {
                    return record(status(mMode, distance, bearing, turnError, reachedIndex, false,
                            "wind zone - confirm arrival"));
                }
            }
            mArrivalConfirmTicks = 0;

            if (target == mVia) {
                // The corridor is a gate, not a stop: cross it and keep going.
                // Hành lang là cổng, không phải điểm dừng: bay xuyên qua rồi tiếp tục.
                mVia = null;
                return record(status(mMode, distance, bearing, turnError, reachedIndex, false,
                        "corridor crossed - continuing"));
            }

            onArrived(target);
            return record(status(mMode, distance, bearing, turnError, reachedIndex, true,
                    "waypoint reached"));
        }
        mArrivalConfirmTicks = 0;

        return record(status(mMode, distance, bearing, turnError, reachedIndex, false, ""));
    }

    private void onArrived(Waypoint reached) {
        if (mMode == NavMode.MISSION) {
            mActiveIndex++;
            if (mActiveIndex >= mMission.size()) {
                // Last waypoint: mission complete, back to idle.
                // Điểm hành trình cuối: sứ mệnh hoàn tất, quay về idle.
                stop();
            } else {
                // The next leg starts here — arm it so a corridor gate gets picked up.
                // Chặng tiếp theo bắt đầu ở đây — gắn nó để cổng hành lang được nhận ra.
                armLeg(reached.getLatitude(), reached.getLongitude(),
                        mMission.getWaypoint(mActiveIndex));
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
