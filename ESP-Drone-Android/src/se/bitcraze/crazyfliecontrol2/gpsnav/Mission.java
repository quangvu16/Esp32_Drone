package se.bitcraze.crazyfliecontrol2.gpsnav;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.TreeMap;

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
     * Extension row carrying one map-annotation vertex (PROTOCOL.md section 10). Not a MAVLink
     * command — anything a desktop planner writes under this id is ours by contract, and every
     * other planner ignores it the way it ignores any unknown command. Columns: {@code p1} =
     * annotation type wire value, {@code p2} = ordinal of the annotation within its type,
     * {@code p3} = vertex index, {@code x}/{@code y} = latitude/longitude.
     *
     * <p>Dòng mở rộng mang một đỉnh của chú thích bản đồ (PROTOCOL.md mục 10). Không phải lệnh
     * MAVLink — bất cứ thứ gì trình lập kế hoạch trên máy tính ghi với id này theo hợp đồng là của
     * ta, và mọi trình lập kế hoạch khác bỏ qua nó như bỏ qua bất kỳ lệnh lạ nào. Cột: {@code p1} =
     * giá trị truyền kiểu chú thích, {@code p2} = thứ tự của chú thích trong kiểu đó,
     * {@code p3} = chỉ số đỉnh, {@code x}/{@code y} = vĩ độ/kinh độ.
     */
    public static final int CMD_MAP_ANNOTATION = 31000;

    /**
     * Home / launch position. {@code Double.NaN} until set — the navigator must not invent one,
     * because RTL aims at it and every target offset is measured from it.
     * Vị trí home (điểm cất cánh). {@code Double.NaN} cho đến khi được đặt — bộ điều hướng không
     * được bịa ra một vị trí, vì RTL nhắm vào nó và mọi độ lệch của mục tiêu đều đo từ đó.
     */
    private double mHomeLatitude = Double.NaN;

    private double mHomeLongitude = Double.NaN;

    private final List<Waypoint> mWaypoints = new ArrayList<Waypoint>();

    /**
     * Map annotations (ported from Valetudo's {@code setMapAnnotations} replace-all semantics):
     * everything the runner must respect but the map cannot show. Mission files persist the whole
     * set — there is no partial write.
     *
     * <p>Các chú thích bản đồ (chuyển thể từ ngữ nghĩa thay-toàn-bộ {@code setMapAnnotations} của
     * Valetudo): mọi thứ bộ chạy phải tôn trọng nhưng bản đồ không hiển thị được. Tệp sứ mệnh lưu
     * toàn bộ tập — không có ghi một phần.
     */
    private final List<MapAnnotation> mAnnotations = new ArrayList<MapAnnotation>();

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

    // region map annotations

    public List<MapAnnotation> getAnnotations() {
        return Collections.unmodifiableList(mAnnotations);
    }

    public MapAnnotation getAnnotation(int index) {
        return (index < 0 || index >= mAnnotations.size()) ? null : mAnnotations.get(index);
    }

    public void addAnnotation(MapAnnotation annotation) {
        if (annotation != null) {
            mAnnotations.add(annotation);
        }
    }

    public boolean removeAnnotation(int index) {
        if (index >= 0 && index < mAnnotations.size()) {
            mAnnotations.remove(index);
            return true;
        }
        return false;
    }

    public void clearAnnotations() {
        mAnnotations.clear();
    }

    /** Live count for the grouped Add menu ("Keep-out zone (2)"). */
    /** Số lượng trực tiếp cho menu Thêm gộp nhóm ("Keep-out zone (2)"). */
    public int annotationCount(MapAnnotation.Type type) {
        int count = 0;
        for (MapAnnotation annotation : mAnnotations) {
            if (annotation.getType() == type) {
                count++;
            }
        }
        return count;
    }

    // endregion

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
     * valid. Map annotations ride after the waypoints as {@link #CMD_MAP_ANNOTATION} extension
     * rows — one row per vertex (PROTOCOL.md section 10).
     * <p>Chỉ số 0 là vị trí home (cờ {@code current} bằng 1), khớp với những gì Mission Planner
     * ghi. Khi chưa có vị trí home nào được lưu, điểm hành trình đầu tiên sẽ được dùng để tệp
     * vẫn hợp lệ. Các chú thích bản đồ đi sau các điểm hành trình dưới dạng dòng mở rộng
     * {@link #CMD_MAP_ANNOTATION} — mỗi dòng một đỉnh (PROTOCOL.md mục 10).
     */
    public String toWaypointsFile() {
        StringBuilder builder = new StringBuilder();
        builder.append(WPL_HEADER).append('\n');

        int rowIndex = 0;
        if (hasHome() || !mWaypoints.isEmpty()) {
            double lat = hasHome() ? mHomeLatitude : mWaypoints.get(0).getLatitude();
            double lon = hasHome() ? mHomeLongitude : mWaypoints.get(0).getLongitude();
            builder.append(formatWplLine(rowIndex++, 1, lat, lon));
        }
        for (int i = 0; i < mWaypoints.size(); i++) {
            Waypoint waypoint = mWaypoints.get(i);
            builder.append(formatWplLine(rowIndex++, 0, waypoint.getLatitude(), waypoint.getLongitude()));
        }
        // Annotations come last, in insertion order; p2 renumbers per type 0..n-1 like
        // Valetudo's replace-all setMapAnnotations write.
        // Chú thích đi cuối, theo thứ tự chèn; p2 đánh số lại theo từng kiểu 0..n-1 như cách
        // Valetudo ghi setMapAnnotations thay-toàn-bộ.
        for (MapAnnotation annotation : mAnnotations) {
            int ordinal = annotationOrdinal(mAnnotations, annotation);
            for (int vertex = 0; vertex < annotation.size(); vertex++) {
                double[] point = annotation.getPoint(vertex);
                builder.append(formatAnnotationLine(
                        rowIndex++, annotation.getType().wireValue, ordinal, vertex,
                        point[0], point[1]));
            }
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

    /** One annotation vertex: p1 = type, p2 = ordinal within type, p3 = vertex index. */
    /** Một đỉnh của chú thích: p1 = kiểu, p2 = thứ tự trong kiểu, p3 = chỉ số đỉnh. */
    private static String formatAnnotationLine(
            int index, int typeWire, int ordinal, int vertex, double lat, double lon) {
        return String.format(
                Locale.US,
                "%d\t%d\t3\t%d\t%d\t%d\t%d\t0\t%.8f\t%.8f\t0\t1\n",
                index,
                0,
                CMD_MAP_ANNOTATION,
                typeWire,
                ordinal,
                vertex,
                lat,
                lon);
    }

    /** Ordinal of this annotation within its type (identity-based, insertion order). */
    /** Thứ tự của chú thích này trong kiểu của nó (theo đối tượng, thứ tự chèn). */
    private static int annotationOrdinal(List<MapAnnotation> all, MapAnnotation target) {
        int ordinal = 0;
        for (MapAnnotation annotation : all) {
            if (annotation == target) {
                return ordinal;
            }
            if (annotation.getType() == target.getType()) {
                ordinal++;
            }
        }
        return ordinal;
    }

    /**
     * Parses a Mission Planner / QGroundControl waypoint file.
     * Phân tích cú pháp một tệp điểm hành trình của Mission Planner / QGroundControl.
     *
     * <p>Row 0 is treated as the home position; every subsequent {@code NAV_WAYPOINT} row becomes a
     * mission waypoint. Non-navigation commands are skipped rather than rejected, so files planned
     * with extra {@code DO_} commands still load their waypoints. {@link #CMD_MAP_ANNOTATION} rows
     * assemble back into map annotations and never count as row 0 — an annotations-only file must
     * not invent a home.
     * <p>Dòng 0 được coi là vị trí home; mỗi dòng {@code NAV_WAYPOINT} tiếp theo trở thành một điểm
     * hành trình của sứ mệnh. Các lệnh không phải điều hướng bị bỏ qua thay vì từ chối, nên tệp được
     * lên kế hoạch với các lệnh {@code DO_} thêm vào vẫn tải được các điểm hành trình. Các dòng
     * {@link #CMD_MAP_ANNOTATION} được lắp ráp lại thành chú thích bản đồ và không bao giờ được tính
     * là dòng 0 — tệp chỉ-có-chú-thích không được bịa ra một vị trí home.
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
            List<StagedAnnotation> staged = new ArrayList<StagedAnnotation>();
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

                if (command == CMD_MAP_ANNOTATION) {
                    // p1 = type wire value, p2 = ordinal within type, p3 = vertex index.
                    // p1 = giá trị truyền kiểu, p2 = thứ tự trong kiểu, p3 = chỉ số đỉnh.
                    int typeWire;
                    int ordinal;
                    int vertex;
                    try {
                        typeWire = Integer.parseInt(fields[4].trim());
                        ordinal = Integer.parseInt(fields[5].trim());
                        vertex = Integer.parseInt(fields[6].trim());
                    } catch (NumberFormatException e) {
                        throw new IOException("Malformed map annotation row: " + line, e);
                    }
                    MapAnnotation.Type type = MapAnnotation.Type.fromWire(typeWire);
                    if (type == null) {
                        throw new IOException("Unknown map annotation type: " + line);
                    }
                    stageAnnotationVertex(staged, type, ordinal, vertex, lat, lon);
                    continue;
                }

                if (rowIndex == 0) {
                    mission.setHome(lat, lon);
                } else if (command == CMD_NAV_WAYPOINT) {
                    mission.addWaypoint(new Waypoint(lat, lon));
                }
                rowIndex++;
            }

            for (StagedAnnotation annotation : staged) {
                if (annotation.vertices.size() < 2) {
                    throw new IOException("Map annotation with fewer than 2 points in file");
                }
                mission.addAnnotation(annotation.toAnnotation());
            }
        }
        return mission;
    }

    /** One map annotation while its vertex rows are being collected out of the file. */
    /** Một chú thích bản đồ trong khi các dòng đỉnh của nó được gom từ tệp. */
    private static final class StagedAnnotation {
        final MapAnnotation.Type type;
        final int ordinal;
        final TreeMap<Integer, double[]> vertices = new TreeMap<Integer, double[]>();

        StagedAnnotation(MapAnnotation.Type type, int ordinal) {
            this.type = type;
            this.ordinal = ordinal;
        }

        MapAnnotation toAnnotation() {
            // Vertex rows arrive sorted by the p3 column, so the points rebuild in order.
            // Các dòng đỉnh được sắp theo cột p3, nên các điểm được dựng lại đúng thứ tự.
            return new MapAnnotation(type, new ArrayList<double[]>(vertices.values()));
        }
    }

    private static void stageAnnotationVertex(List<StagedAnnotation> staged, MapAnnotation.Type type,
            int ordinal, int vertex, double lat, double lon) {
        for (StagedAnnotation candidate : staged) {
            if (candidate.type == type && candidate.ordinal == ordinal) {
                candidate.vertices.put(vertex, new double[] {lat, lon});
                return;
            }
        }
        StagedAnnotation created = new StagedAnnotation(type, ordinal);
        created.vertices.put(vertex, new double[] {lat, lon});
        staged.add(created);
    }

    // endregion
}
