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
 * Một sứ mệnh GPS có thứ tự: vị trí home cộng với danh sách điểm hành trình cần ghé.
 *
 * <p>Serialises to the Mission Planner / QGroundControl {@code .waypoints} format ({@code QGC WPL
 * 110}) so a flight can be planned on a desktop and loaded on the phone. Only the horizontal
 * component is meaningful here: the altitude column is written as zero and ignored on read (the
 * firmware holds a thrust level, not an altitude).
 * <p>Tuần tự hoá theo định dạng {@code .waypoints} của Mission Planner / QGroundControl
 * ({@code QGC WPL 110}) để có thể lên kế hoạch bay trên máy tính rồi tải vào điện thoại. Chỉ
 * thành phần nằm ngang là có ý nghĩa ở đây: cột độ cao được ghi bằng 0 và bị bỏ qua khi đọc
 * (firmware giữ một mức lực đẩy, không phải độ cao).
 */
public class Mission {

    /** Header line every {@code .waypoints} file starts with. */
    /** Dòng tiêu đề mọi tệp {@code .waypoints} đều bắt đầu bằng. */
    public static final String WPL_HEADER = "QGC WPL 110";

    /** MAVLink {@code MAV_CMD_NAV_WAYPOINT} — the only command read back as a waypoint. */
    /** MAVLink {@code MAV_CMD_NAV_WAYPOINT} — lệnh duy nhất được đọc lại thành điểm hành trình. */
    public static final int CMD_NAV_WAYPOINT = 16;

    /**
     * Home / launch position. {@code Double.NaN} until set — the navigator must not invent one,
     * because RTL aims at it and every target offset is measured from it.
     * Vị trí home (điểm cất cánh). {@code Double.NaN} cho đến khi được đặt — bộ điều hướng không
     * được bịa ra một vị trí, vì RTL nhắm vào nó và mọi độ lệch của mục tiêu đều đo từ đó.
     */
    private double mHomeLatitude = Double.NaN;

    private double mHomeLongitude = Double.NaN;

    private final List<Waypoint> mWaypoints = new ArrayList<Waypoint>();

    /** Removes every waypoint but keeps the home position. */
    /** Xóa mọi điểm hành trình nhưng giữ nguyên vị trí home. */
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
    /** Tổng chiều dài dọc đường bay, từ home đến điểm hành trình cuối, đơn vị mét. */
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
     * Tuần tự hoá sứ mệnh này theo định dạng tệp điểm hành trình của Mission Planner /
     * QGroundControl.
     *
     * <p>Index 0 is the home position ({@code current} flag 1), matching what Mission Planner
     * writes. When no home has been recorded yet the first waypoint is used so the file stays
     * valid.
     * <p>Chỉ số 0 là vị trí home (cờ {@code current} bằng 1), khớp với những gì Mission Planner
     * ghi. Khi chưa có vị trí home nào được lưu, điểm hành trình đầu tiên sẽ được dùng để tệp
     * vẫn hợp lệ.
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
     * Phân tích cú pháp một tệp điểm hành trình của Mission Planner / QGroundControl.
     *
     * <p>Row 0 is treated as the home position; every subsequent {@code NAV_WAYPOINT} row becomes a
     * mission waypoint. Non-navigation commands are skipped rather than rejected, so files planned
     * with extra {@code DO_} commands still load their waypoints.
     * <p>Dòng 0 được coi là vị trí home; mỗi dòng {@code NAV_WAYPOINT} tiếp theo trở thành một điểm
     * hành trình của sứ mệnh. Các lệnh không phải điều hướng bị bỏ qua thay vì từ chối, nên tệp được
     * lên kế hoạch với các lệnh {@code DO_} thêm vào vẫn tải được các điểm hành trình.
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
                // Mission Planner ghi bằng tab; chấp nhận mọi khoảng trắng để tệp sửa tay vẫn tải được.
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
