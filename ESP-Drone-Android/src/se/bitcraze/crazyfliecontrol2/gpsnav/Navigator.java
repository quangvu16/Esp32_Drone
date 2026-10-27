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
 */
public class Navigator {

    /** Mission planner flight modes, mirroring the OpenBot enum. */
    public enum NavMode {
        IDLE,
        MISSION,
        GUIDED,
        RTL
    }

    /**
     * Hold rather than navigate on a phone fix worse than this (PROTOCOL.md section 10). Navigating
     * on a bad fix puts the target in the wrong place; waiting costs nothing.
     */
    public static final float MAX_POSITION_ACCURACY_M = 15.0f;

    /** Outcome of one navigation tick. Distances are metres, bearings are compass degrees. */
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
        public float getDistanceMeters() {
            return mDistanceMeters;
        }

        /** Compass bearing to the current target in degrees, NaN when there is no target. */
        public float getBearingDegrees() {
            return mBearingDegrees;
        }

        /** Turn from the drone's heading onto the target bearing; positive = turn right. */
        public float getTurnErrorDegrees() {
            return mTurnErrorDegrees;
        }

        /** Index of the waypoint being sought (before any advance), -1 when not in MISSION. */
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
    private double mLaunchHeadingDeg = Double.NaN;

    /** The drone's compass heading from {@code flight.heading}, NaN until the first log block. */
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
    public void markHome(double latitude, double longitude) {
        mMission.setHome(latitude, longitude);
    }

    /**
     * Fixes the launch frame (PROTOCOL.md section 6): {@code alphaDeg} is the compass bearing of the
     * nose at ALIGN. Needed to compare the drone's launch-frame position against waypoints.
     */
    public void setLaunchHeading(double alphaDeg) {
        mLaunchHeadingDeg = alphaDeg;
    }

    /** Feeds {@code flight.heading} in so {@link NavStatus#getTurnErrorDegrees()} can be computed. */
    public void setDroneHeading(float headingDeg) {
        mDroneHeadingDeg = headingDeg;
    }

    public void loadMission(Mission mission) {
        mMission = mission != null ? mission : new Mission();
        mActiveIndex = 0;
    }

    /** Loads a Mission Planner {@code .waypoints} document. Returns the mission now held. */
    public Mission loadMissionFile(String contents) throws IOException {
        loadMission(Mission.parseWaypointsFile(contents));
        return mMission;
    }

    /** Serialises the current mission to the {@code .waypoints} format. */
    public String saveMissionFile() {
        return mMission.toWaypointsFile();
    }

    /** The waypoint being sought right now, or null when idle. */
    public Waypoint currentTarget() {
        if (mMode == NavMode.MISSION) {
            return mMission.getWaypoint(mActiveIndex);
        }
        return mGuidedTarget;
    }

    // endregion

    // region mode control

    /** MISSION: fly every waypoint in order. Returns false if there are none. */
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
    public boolean startGuided(double latitude, double longitude) {
        mGuidedTarget = new Waypoint(
                latitude, longitude, "", Waypoint.DEFAULT_ACCEPTANCE_RADIUS_M);
        mMode = NavMode.GUIDED;
        mPaused = false;
        return true;
    }

    /** RTL: return to the recorded launch position. */
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
     * @param posX               drone {@code nav.pos_x} in metres
     * @param posY               drone {@code nav.pos_y} in metres
     * @param phoneAccuracyMeters horizontal accuracy of the phone fix; NaN when unknown
     */
    public NavStatus step(float posX, float posY, float phoneAccuracyMeters) {
        if (!isNavigating() || mPaused) {
            return record(status(mMode, Float.NaN, Float.NaN, Float.NaN, -1, false,
                    mPaused ? "paused" : "idle"));
        }

        if (!Float.isNaN(phoneAccuracyMeters) && phoneAccuracyMeters > MAX_POSITION_ACCURACY_M) {
            // Targeting from a bad fix is worse than waiting: hold and say so out loud.
            return record(status(mMode, Float.NaN, Float.NaN, Float.NaN, mActiveIndex, false,
                    String.format(Locale.US, "GPS accuracy %.1f m, holding", phoneAccuracyMeters)));
        }

        if (Double.isNaN(mLaunchHeadingDeg) || !mMission.hasHome()) {
            // Without ALIGN there is no launch frame to compare in; never navigate on NaN.
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
        float bearing = (float) GeoMath.wrapDegrees(
                mLaunchHeadingDeg - Math.toDegrees(Math.atan2(dy, dx)));
        float turnError = Float.isNaN(mDroneHeadingDeg)
                ? Float.NaN
                : (float) GeoMath.turnErrorDegrees(mDroneHeadingDeg, bearing);

        // Snapshot before advancing, so the status names the waypoint just hit.
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
                stop();
            }
        } else {
            // GUIDED and RTL are one-shot: arriving disarms them.
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
