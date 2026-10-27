package se.bitcraze.crazyfliecontrol2.gpsnav;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * An ordered GPS mission: a home position plus a list of waypoints to visit.
 *
 * <p>Serialises to the Mission Planner / QGroundControl {@code .waypoints} format ({@code QGC WPL
 * 110}) so a flight can be planned on a desktop and loaded on the phone. Only the horizontal
 * component is meaningful here: the altitude column is written as zero and ignored on read (the
 * firmware holds a thrust level, not an altitude).
 */
public class Mission {

    /** Header line every {@code .waypoints} file starts with. */
    public static final String WPL_HEADER = "QGC WPL 110";

    /** MAVLink {@code MAV_CMD_NAV_WAYPOINT} — the only command read back as a waypoint. */
    public static final int CMD_NAV_WAYPOINT = 16;

    /**
     * Home / launch position. {@code Double.NaN} until set — the navigator must not invent one,
     * because RTL aims at it and every target offset is measured from it.
     */
    private double mHomeLatitude = Double.NaN;

    private double mHomeLongitude = Double.NaN;

    private final List<Waypoint> mWaypoints = new ArrayList<Waypoint>();

    /** Removes every waypoint but keeps the home position. */
    public void clearWaypoints() {
        mWaypoints.clear();
    }

    public void addWaypoint(Waypoint waypoint) {
        if (waypoint != null) {
            mWaypoints.add(waypoint);
        }
    }

    public void addWaypoint(int index, Waypoint waypoint) {
        if (waypoint == null) {
            return;
        }
        mWaypoints.add(Math.max(0, Math.min(index, mWaypoints.size())), waypoint);
    }

    public void removeWaypoint(int index) {
        if (index >= 0 && index < mWaypoints.size()) {
            mWaypoints.remove(index);
        }
    }

    public void moveWaypoint(int fromIndex, int toIndex) {
        if (fromIndex < 0
                || fromIndex >= mWaypoints.size()
                || toIndex < 0
                || toIndex >= mWaypoints.size()) {
            return;
        }
        Waypoint moved = mWaypoints.remove(fromIndex);
        mWaypoints.add(toIndex, moved);
    }

    public void setWaypoints(List<Waypoint> replacement) {
        mWaypoints.clear();
        if (replacement != null) {
            for (Waypoint waypoint : replacement) {
                addWaypoint(waypoint);
            }
        }
    }

    public List<Waypoint> getWaypoints() {
        return Collections.unmodifiableList(mWaypoints);
    }

    public Waypoint getWaypoint(int index) {
        return (index < 0 || index >= mWaypoints.size()) ? null : mWaypoints.get(index);
    }

    public int size() {
        return mWaypoints.size();
    }

    public boolean isEmpty() {
        return mWaypoints.isEmpty();
    }

    public boolean hasHome() {
        return !Double.isNaN(mHomeLatitude) && !Double.isNaN(mHomeLongitude);
    }

    public double getHomeLatitude() {
        return mHomeLatitude;
    }

    public double getHomeLongitude() {
        return mHomeLongitude;
    }

    public void setHome(double latitude, double longitude) {
        this.mHomeLatitude = latitude;
        this.mHomeLongitude = longitude;
    }

    /** Total along-track length in metres, home to last waypoint. */
    public double totalDistanceMeters() {
        if (mWaypoints.isEmpty()) {
            return 0.0;
        }
        double total = 0.0;
        double prevLat = mHomeLatitude;
        double prevLon = mHomeLongitude;
        boolean hasPrev = hasHome();
        for (Waypoint waypoint : mWaypoints) {
            if (hasPrev) {
                total += GeoMath.distanceMeters(
                        prevLat, prevLon, waypoint.getLatitude(), waypoint.getLongitude());
            }
            prevLat = waypoint.getLatitude();
            prevLon = waypoint.getLongitude();
            hasPrev = true;
        }
        return total;
    }

    // region Mission Planner .waypoints codec

    /**
     * Serialises this mission to the Mission Planner / QGroundControl waypoint file format.
     *
     * <p>Index 0 is the home position ({@code current} flag 1), matching what Mission Planner
     * writes. When no home has been recorded yet the first waypoint is used so the file stays
     * valid.
     */
    public String toWaypointsFile() {
        StringBuilder builder = new StringBuilder();
        builder.append(WPL_HEADER).append('\n');

        if (hasHome() || !mWaypoints.isEmpty()) {
            double lat = hasHome() ? mHomeLatitude : mWaypoints.get(0).getLatitude();
            double lon = hasHome() ? mHomeLongitude : mWaypoints.get(0).getLongitude();
            builder.append(formatWplLine(0, 1, lat, lon));
        }
        for (int i = 0; i < mWaypoints.size(); i++) {
            Waypoint waypoint = mWaypoints.get(i);
            builder.append(formatWplLine(i + 1, 0, waypoint.getLatitude(), waypoint.getLongitude()));
        }
        return builder.toString();
    }

    private static String formatWplLine(int index, int current, double lat, double lon) {
        // index, current, frame, command, p1, p2, p3, p4, x, y, z, autocontinue
        return String.format(
                Locale.US,
                "%d\t%d\t3\t%d\t0\t0\t0\t0\t%.8f\t%.8f\t0\t1\n",
                index,
                current,
                CMD_NAV_WAYPOINT,
                lat,
                lon);
    }

    /**
     * Parses a Mission Planner / QGroundControl waypoint file.
     *
     * <p>Row 0 is treated as the home position; every subsequent {@code NAV_WAYPOINT} row becomes a
     * mission waypoint. Non-navigation commands are skipped rather than rejected, so files planned
     * with extra {@code DO_} commands still load their waypoints.
     */
    public static Mission parseWaypointsFile(String contents) throws IOException {
        Mission mission = new Mission();
        if (contents == null) {
            return mission;
        }

        try (BufferedReader reader = new BufferedReader(new StringReader(contents))) {
            String line = reader.readLine();
            if (line == null) {
                throw new IOException("Empty waypoint file");
            }
            if (!line.trim().startsWith("QGC WPL")) {
                throw new IOException("Unsupported waypoint file header: " + line.trim());
            }

            int rowIndex = 0;
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) {
                    continue;
                }
                // Mission Planner writes tabs; tolerate any whitespace so hand-edited files load.
                String[] fields = line.split("\\t");
                if (fields.length < 11) {
                    fields = line.split("\\s+");
                }
                if (fields.length < 11) {
                    throw new IOException("Malformed waypoint row: " + line);
                }

                int command;
                double lat;
                double lon;
                try {
                    command = Integer.parseInt(fields[3].trim());
                    lat = Double.parseDouble(fields[8].trim());
                    lon = Double.parseDouble(fields[9].trim());
                } catch (NumberFormatException e) {
                    throw new IOException("Malformed waypoint row: " + line, e);
                }

                if (rowIndex == 0) {
                    mission.setHome(lat, lon);
                } else if (command == CMD_NAV_WAYPOINT) {
                    mission.addWaypoint(new Waypoint(lat, lon));
                }
                rowIndex++;
            }
        }
        return mission;
    }

    // endregion
}
