package se.bitcraze.crazyfliecontrol2.gpsnav;

import java.util.Locale;

/**
 * A single GPS waypoint: the horizontal target plus an acceptance radius. There is no altitude leg
 * — the firmware holds a thrust level, not an altitude (PROTOCOL.md section 5).
 *
 * <p>Immutable so the navigator can compare one against the drone's live position without
 * defensive copies.
 */
public final class Waypoint {

    /**
     * Default acceptance radius. PROTOCOL.md section 10 pins 2.5 m: tight enough to actually
     * "arrive", wide enough to swallow the metres of drift the dead reckoning accumulates per
     * minute — a tighter circle would never be reached before the estimate walks past it.
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
    public double distanceFrom(double latitude, double longitude) {
        return GeoMath.distanceMeters(latitude, longitude, this.mLatitude, this.mLongitude);
    }

    /** Initial bearing in degrees from the given position to this waypoint. */
    public double bearingFrom(double latitude, double longitude) {
        return GeoMath.bearingDegrees(latitude, longitude, this.mLatitude, this.mLongitude);
    }

    /**
     * This waypoint as a north/east offset in metres from the given (launch) position — the exact
     * payload of a {@code TARGET} packet (PROTOCOL.md section 6).
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
     *
     * @return {@code {x, y}} in metres
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
