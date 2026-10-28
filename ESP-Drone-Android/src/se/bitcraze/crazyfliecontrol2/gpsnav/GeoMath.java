package se.bitcraze.crazyfliecontrol2.gpsnav;

/**
 * Geographic helpers for GPS mission navigation.
 * Các hàm hỗ trợ tính toán địa lý cho sứ mệnh điều hướng GPS.
 *
 * <p>Pure Java so the math can be exercised without a drone attached. All angles are in degrees and
 * all distances in metres. The great-circle formulas are the ones pinned by PROTOCOL.md section 9.
 * <p>Viết bằng Java thuần để có thể kiểm thử toán học mà không cần gắn drone. Mọi góc đều theo
 * độ và mọi khoảng cách theo mét. Các công thức đường tròn lớn là những công thức được chốt
 * trong PROTOCOL.md mục 9.
 *
 * <p>The launch-frame helpers implement PROTOCOL.md section 6: at {@code ALIGN} the frame is fixed
 * with {@code +x} where the nose pointed (compass bearing {@code alpha}) and {@code +y} 90 degrees to
 * the LEFT of that. The drone's dead-reckoned {@code nav.pos_x}/{@code nav.pos_y} live in exactly
 * this frame, so waypoints are compared against them without a lat/lon round trip.
 * <p>Các hàm khung cất cánh triển khai PROTOCOL.md mục 6: tại {@code ALIGN} khung
 * được cố định với {@code +x} theo hướng mũi drone chỉ khi đó (phương hướng la bàn {@code alpha})
 * và {@code +y} lệch 90 độ sang TRÁI của hướng đó. {@code nav.pos_x}/{@code nav.pos_y} do drone tự
 * tự dẫn đường quán tính nằm đúng trong khung này, nên các điểm hành trình được so với chúng mà không
 * phải lượt đi-về qua vĩ độ/kinh độ.
 */
public final class GeoMath {

    /** Mean earth radius used by the haversine formula (PROTOCOL.md section 9). */
    /** Bán kính trung bình của Trái Đất dùng trong công thức haversine (PROTOCOL.md mục 9). */
    public static final double EARTH_RADIUS_M = 6371008.8;

    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;

    private GeoMath() {
    }

    /** Great-circle distance between two WGS-84 points, in metres. */
    /** Khoảng cách đường tròn lớn giữa hai điểm WGS-84, đơn vị mét. */
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
     * Phương hướng đường tròn lớn ban đầu từ điểm thứ nhất đến điểm thứ hai.
     *
     * <p>Result is in degrees clockwise from true north, wrapped to {@code [-180, 180)}.
     * <p>Kết quả theo độ, tính thuận chiều kim đồng hồ từ hướng bắc địa lý, gói trong
     * {@code [-180, 180)}.
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
    /** Gói một góc về khoảng {@code [-180, 180)}. */
    public static double wrapDegrees(double degrees) {
        return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    /**
     * Smallest signed turn from a heading onto a target bearing.
     * Lượt rẽ có dấu, nhỏ nhất từ hướng mũi hiện tại sang phương hướng mục tiêu.
     *
     * <p>Positive means the target is to the right (clockwise), negative to the left.
     * <p>Dương nghĩa là mục tiêu ở bên phải (thuận chiều kim đồng hồ), âm ở bên trái.
     */
    public static double turnErrorDegrees(double headingDegrees, double targetBearingDegrees) {
        return wrapDegrees(targetBearingDegrees - headingDegrees);
    }

    /**
     * Moves a point along a great circle by the given distance and initial bearing.
     * Dịch chuyển một điểm dọc theo đường tròn lớn theo khoảng cách và phương hướng ban đầu cho trước.
     *
     * <p>Returned as {@code {latitude, longitude}} in degrees.
     * <p>Trả về {@code {vĩ độ, kinh độ}} theo độ.
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
     * Xoay độ lệch bắc/đông về khung cất cánh (PROTOCOL.md mục 6):
     *
     * <pre>
     *   x = n*cos(alpha) + e*sin(alpha)    // +x = where the nose pointed at ALIGN
     *   y = n*sin(alpha) - e*cos(alpha)    // +y = 90 deg LEFT of that
     * </pre>
     *
     * <p>The minus sign on the y line is the whole point: with +y being the drone's left, the frame
     * is left-handed relative to compass bearings, which increase clockwise (to the right).
     * <p>Dấu trừ ở dòng y chính là điểm mấu chốt: vì +y là phía bên trái của drone, khung này
     * thuận tay trái so với phương hướng la bàn, vốn tăng theo chiều kim đồng hồ (sang phải).
     *
     * @return {@code {x, y}} in metres
     * @return {@code {x, y}} theo mét
     */
    public static float[] northEastToLaunchFrame(double northM, double eastM, double alphaDeg) {
        double alpha = alphaDeg * DEG_TO_RAD;
        float x = (float) (northM * Math.cos(alpha) + eastM * Math.sin(alpha));
        float y = (float) (northM * Math.sin(alpha) - eastM * Math.cos(alpha));
        return new float[] {x, y};
    }

    /**
     * Inverse of {@link #northEastToLaunchFrame}: launch-frame metres back to a north/east offset.
     * Nghịch đảo của {@link #northEastToLaunchFrame}: từ mét trong khung cất cánh
     * về lại độ lệch bắc/đông.
     *
     * <p>The rotation matrix squares to the identity, so the inverse transform is the same formula
     * (substituting the forward result back cancels every cross term).
     * <p>Ma trận xoay bình phương ra ma trận đơn vị, nên phép biến đổi nghịch đảo dùng đúng công
     * thức đó (thay kết quả thuận vào lại sẽ triệt tiêu mọi số hạng chéo).
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
     * Từ mét trong khung cất cánh về lại vị trí WGS-84, dùng để vẽ ước tính tự
     * tự dẫn đường quán tính của drone lên bản đồ. Xoay độ lệch về lại bắc/đông, rồi đi dọc đường
     * tròn lớn từ vị trí home (điểm cất cánh).
     *
     * @return {@code {latitude, longitude}} in degrees
     * @return {@code {vĩ độ, kinh độ}} theo độ
     */
    public static double[] launchFrameToLatLon(
            float x, float y, double alphaDeg, double homeLat, double homeLon) {
        float[] northEast = launchFrameToNorthEast(x, y, alphaDeg);
        double distance = Math.hypot(northEast[0], northEast[1]);
        // atan2(east, north) is the compass bearing (clockwise from north) of the offset.
        // atan2(east, north) là phương hướng la bàn (thuận chiều kim đồng hồ từ hướng bắc)
        // của độ lệch.
        double bearing = Math.atan2(northEast[1], northEast[0]) * RAD_TO_DEG;
        return destinationPoint(homeLat, homeLon, bearing, distance);
    }
}
