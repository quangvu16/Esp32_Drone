package se.bitcraze.crazyfliecontrol2.gpsnav;

import java.util.Locale;

/**
 * A single GPS waypoint: the horizontal target plus an acceptance radius. There is no altitude leg
 * — the firmware holds a thrust level, not an altitude (PROTOCOL.md section 5).
 * Một điểm hành trình GPS đơn lẻ: mục tiêu nằm ngang cộng với bán kính chấp nhận. Không có thành
 * phần độ cao — firmware giữ một mức lực đẩy, không phải độ cao (PROTOCOL.md mục 5).
 *
 * <p>Immutable so the navigator can compare one against the drone's live position without
 * defensive copies.
 * <p>Bất biến để bộ điều hướng có thể so điểm này với vị trí trực tiếp của drone mà không cần
 * tạo bản sao phòng ngừa.
 */
public final class Waypoint {

    /**
     * Default acceptance radius. PROTOCOL.md section 10 pins 2.5 m: tight enough to actually
     * "arrive", wide enough to swallow the metres of drift the dead reckoning accumulates per
     * minute — a tighter circle would never be reached before the estimate walks past it.
     * Bán kính chấp nhận mặc định. PROTOCOL.md mục 10 chốt 2,5 m: đủ hẹp để thực sự "đến nơi",
     * đủ rộng để nuốt trọn số mét trôi mà tự dẫn đường quán tính tích tụ mỗi phút — vòng tròn hẹp
     * hơn sẽ không bao giờ được chạm tới trước khi giá trị ước tính đi lệch qua nó.
     */
    public static final float DEFAULT_ACCEPTANCE_RADIUS_M = 2.5f;

    private final double mLatitude;
    private final double mLongitude;
    private final String mName;
    private final float mAcceptanceRadiusMeters;

    public Waypoint(double latitude, double longitude) {
        this(latitude, longitude, "", DEFAULT_ACCEPTANCE_RADIUS_M);
    }

    public Waypoint(double latitude, double longitude, String name, float acceptanceRadiusMeters) {
        this.mLatitude = latitude;
        this.mLongitude = longitude;
        this.mName = name == null ? "" : name;
        this.mAcceptanceRadiusMeters = Math.max(0.5f, acceptanceRadiusMeters);
    }

    public double getLatitude() {
        return mLatitude;
    }

    public double getLongitude() {
        return mLongitude;
    }

    public String getName() {
        return mName;
    }

    public float getAcceptanceRadiusMeters() {
        return mAcceptanceRadiusMeters;
    }

    /** Distance in metres from the given position to this waypoint. */
    /** Khoảng cách từ vị trí cho trước đến điểm hành trình này, đơn vị mét. */
    public double distanceFrom(double latitude, double longitude) {
        return GeoMath.distanceMeters(latitude, longitude, this.mLatitude, this.mLongitude);
    }

    /** Initial bearing in degrees from the given position to this waypoint. */
    /** Phương hướng ban đầu từ vị trí cho trước đến điểm hành trình này, theo độ. */
    public double bearingFrom(double latitude, double longitude) {
        return GeoMath.bearingDegrees(latitude, longitude, this.mLatitude, this.mLongitude);
    }

    /**
     * This waypoint as a north/east offset in metres from the given (launch) position — the exact
     * payload of a {@code TARGET} packet (PROTOCOL.md section 6).
     * Điểm hành trình này dưới dạng độ lệch bắc/đông theo mét từ vị trí (cất cánh) cho trước —
     * đúng phần dữ liệu của một gói tin {@code TARGET} (PROTOCOL.md mục 6).
     *
     * @return {@code {northM, eastM}}
     */
    public float[] toNorthEast(double homeLat, double homeLon) {
        double distance = distanceFrom(homeLat, homeLon);
        double bearing = Math.toRadians(bearingFrom(homeLat, homeLon));
        float north = (float) (distance * Math.cos(bearing));
        float east = (float) (distance * Math.sin(bearing));
        return new float[] {north, east};
    }

    /**
     * This waypoint in the launch frame, i.e. directly comparable against the drone's dead-reckoned
     * {@code nav.pos_x}/{@code nav.pos_y} — same frame, both metres, no lat/lon round trip.
     * Điểm hành trình này trong khung cất cánh, tức là so được trực tiếp với
     * {@code nav.pos_x}/{@code nav.pos_y} do drone tự dẫn đường quán tính — cùng khung, đều theo
     * mét, không có lượt đi-về qua vĩ độ/kinh độ.
     *
     * @return {@code {x, y}} in metres
     * @return {@code {x, y}} theo mét
     */
    public float[] toLaunchFrame(double homeLat, double homeLon, double alphaDeg) {
        float[] northEast = toNorthEast(homeLat, homeLon);
        return GeoMath.northEastToLaunchFrame(northEast[0], northEast[1], alphaDeg);
    }

    @Override
    public String toString() {
        return String.format(
                Locale.US,
                "Waypoint{%s lat=%.7f lon=%.7f r=%.1fm}",
                mName.isEmpty() ? "-" : mName,
                mLatitude,
                mLongitude,
                mAcceptanceRadiusMeters);
    }
}
