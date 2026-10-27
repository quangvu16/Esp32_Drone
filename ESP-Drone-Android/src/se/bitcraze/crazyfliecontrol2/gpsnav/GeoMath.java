package se.bitcraze.crazyfliecontrol2.gpsnav;

/**
 * Geographic helpers for GPS mission navigation.
 *
 * <p>Pure Java so the math can be exercised without a drone attached. All angles are in degrees and
 * all distances in metres. The great-circle formulas are the ones pinned by PROTOCOL.md section 9.
 *
 * <p>The launch-frame helpers implement PROTOCOL.md section 6: at {@code ALIGN} the frame is fixed
 * with {@code +x} where the nose pointed (compass bearing {@code alpha}) and {@code +y} 90 degrees to
 * the LEFT of that. The drone's dead-reckoned {@code nav.pos_x}/{@code nav.pos_y} live in exactly
 * this frame, so waypoints are compared against them without a lat/lon round trip.
 */
public final class GeoMath {

    /** Mean earth radius used by the haversine formula (PROTOCOL.md section 9). */
    public static final double EARTH_RADIUS_M = 6371008.8;

    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;

    private GeoMath() {
    }

    /** Great-circle distance between two WGS-84 points, in metres. */
    public static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
        double dLat = (lat2 - lat1) * DEG_TO_RAD;
        double dLon = (lon2 - lon1) * DEG_TO_RAD;
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                + Math.cos(lat1 * DEG_TO_RAD) * Math.cos(lat2 * DEG_TO_RAD)
                * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        return 2 * EARTH_RADIUS_M * Math.asin(Math.min(1.0, Math.sqrt(a)));
    }

    /**
     * Initial great-circle bearing from the first point to the second.
     *
     * <p>Result is in degrees clockwise from true north, wrapped to {@code [-180, 180)}.
     */
    public static double bearingDegrees(double lat1, double lon1, double lat2, double lon2) {
        double phi1 = lat1 * DEG_TO_RAD;
        double phi2 = lat2 * DEG_TO_RAD;
        double dLon = (lon2 - lon1) * DEG_TO_RAD;
        double y = Math.sin(dLon) * Math.cos(phi2);
        double x = Math.cos(phi1) * Math.sin(phi2) - Math.sin(phi1) * Math.cos(phi2) * Math.cos(dLon);
        return wrapDegrees(Math.atan2(y, x) * RAD_TO_DEG);
    }

    /** Wraps an angle to the range {@code [-180, 180)}. */
    public static double wrapDegrees(double degrees) {
        return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    /**
     * Smallest signed turn from a heading onto a target bearing.
     *
     * <p>Positive means the target is to the right (clockwise), negative to the left.
     */
    public static double turnErrorDegrees(double headingDegrees, double targetBearingDegrees) {
        return wrapDegrees(targetBearingDegrees - headingDegrees);
    }

    /**
     * Moves a point along a great circle by the given distance and initial bearing.
     *
     * <p>Returned as {@code {latitude, longitude}} in degrees.
     */
    public static double[] destinationPoint(
            double latitude, double longitude, double bearingDegrees, double distanceMeters) {
        double angular = distanceMeters / EARTH_RADIUS_M;
        double theta = bearingDegrees * DEG_TO_RAD;
        double phi1 = latitude * DEG_TO_RAD;
        double lambda1 = longitude * DEG_TO_RAD;

        double sinPhi2 = Math.sin(phi1) * Math.cos(angular)
                + Math.cos(phi1) * Math.sin(angular) * Math.cos(theta);
        double phi2 = Math.asin(Math.max(-1.0, Math.min(1.0, sinPhi2)));
        double lambda2 = lambda1 + Math.atan2(
                Math.sin(theta) * Math.sin(angular) * Math.cos(phi1),
                Math.cos(angular) - Math.sin(phi1) * sinPhi2);

        return new double[] {phi2 * RAD_TO_DEG, wrapDegrees(lambda2 * RAD_TO_DEG)};
    }

    /**
     * Rotates a north/east offset into the launch frame (PROTOCOL.md section 6):
     *
     * <pre>
     *   x = n*cos(alpha) + e*sin(alpha)    // +x = where the nose pointed at ALIGN
     *   y = n*sin(alpha) - e*cos(alpha)    // +y = 90 deg LEFT of that
     * </pre>
     *
     * <p>The minus sign on the y line is the whole point: with +y being the drone's left, the frame
     * is left-handed relative to compass bearings, which increase clockwise (to the right).
     *
     * @return {@code {x, y}} in metres
     */
    public static float[] northEastToLaunchFrame(double northM, double eastM, double alphaDeg) {
        double alpha = alphaDeg * DEG_TO_RAD;
        float x = (float) (northM * Math.cos(alpha) + eastM * Math.sin(alpha));
        float y = (float) (northM * Math.sin(alpha) - eastM * Math.cos(alpha));
        return new float[] {x, y};
    }

    /**
     * Inverse of {@link #northEastToLaunchFrame}: launch-frame metres back to a north/east offset.
     *
     * <p>The rotation matrix squares to the identity, so the inverse transform is the same formula
     * (substituting the forward result back cancels every cross term).
     *
     * @return {@code {northM, eastM}}
     */
    public static float[] launchFrameToNorthEast(float x, float y, double alphaDeg) {
        double alpha = alphaDeg * DEG_TO_RAD;
        float north = (float) (x * Math.cos(alpha) + y * Math.sin(alpha));
        float east = (float) (x * Math.sin(alpha) - y * Math.cos(alpha));
        return new float[] {north, east};
    }

    /**
     * Launch-frame metres back to a WGS-84 position, for drawing the drone's dead-reckoned
     * estimate on the map. Rotates the offset back to north/east, then walks the great circle
     * from the home (launch) position.
     *
     * @return {@code {latitude, longitude}} in degrees
     */
    public static double[] launchFrameToLatLon(
            float x, float y, double alphaDeg, double homeLat, double homeLon) {
        float[] northEast = launchFrameToNorthEast(x, y, alphaDeg);
        double distance = Math.hypot(northEast[0], northEast[1]);
        // atan2(east, north) is the compass bearing (clockwise from north) of the offset.
        double bearing = Math.atan2(northEast[1], northEast[0]) * RAD_TO_DEG;
        return destinationPoint(homeLat, homeLon, bearing, distance);
    }
}
