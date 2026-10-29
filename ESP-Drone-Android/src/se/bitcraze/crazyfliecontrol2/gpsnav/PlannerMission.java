package se.bitcraze.crazyfliecontrol2.gpsnav;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * A mission in the QGC WPL 110 format, read and written with the exact
 * semantics of Mission Planner — the reader is {@code WaypointFile.
 * ReadWaypointFile} and the writer is {@code FlightPlanner.savewaypoints()}
 * (PROTOCOL.md §12). Files produced here are byte-identical to Mission
 * Planner's for the same mission, and anything Mission Planner loads loads
 * here; the quirks below are deliberate, not oversights.
 * <p>
 * Một nhiệm vụ theo định dạng QGC WPL 110, đọc và ghi với đúng ngữ nghĩa của
 * Mission Planner — đầu đọc là {@code WaypointFile.ReadWaypointFile}, đầu ghi
 * là {@code FlightPlanner.savewaypoints()} (PROTOCOL.md mục 12). Tệp tạo ra ở
 * đây khớp từng byte với tệp Mission Planner cho cùng nhiệm vụ, và mọi nội dung
 * Mission Planner nạp được đều nạp được ở đây; các điểm lạ dưới đây là cố ý,
 * không phải thiếu sót.
 *
 * <p>Reader rules, copied from Mission Planner:
 * <ul>
 *   <li>the header line must contain "QGC WPL" or the file is rejected;</li>
 *   <li>lines starting with '#' are comments and are skipped;</li>
 *   <li>fields split on tab, space or comma, empty fields dropped;</li>
 *   <li>a row with 9 or fewer fields is skipped silently;</li>
 *   <li>if the first parsed row's index field is not the string "0", a blank
 *       home row is inserted before it — and the blank is inserted even when
 *       that row then fails to parse;</li>
 *   <li>columns are index, current, frame, command, p1..p4, lat, lng, alt —
 *       the command is an MAV_CMD name or its number;</li>
 *   <li>command 99 is a legacy waypoint id and is rewritten to 0;</li>
 *   <li>a row that cannot be parsed is reported and skipped, the rest of the
 *       file still loads.</li>
 * </ul>
 * Quy tắc đọc, sao chép từ Mission Planner:
 * <ul>
 *   <li>dòng đầu phải chứa "QGC WPL" thì tệp mới được chấp nhận;</li>
 *   <li>dòng bắt đầu bằng '#' là chú thích và bị bỏ qua;</li>
 *   <li>trường phân tách bằng tab, dấu cách hoặc phẩy, trường rỗng bị bỏ;</li>
 *   <li>dòng có từ 9 trường trở xuống bị bỏ qua im lặng;</li>
 *   <li>nếu ô chỉ số của dòng đầu tiên phân tích được không đúng chuỗi "0", một
 *       dòng home trống sẽ được chèn vào trước nó — và dòng trống vẫn được chèn
 *       kể cả khi dòng đó sau đó phân tích thất bại;</li>
 *   <li>các cột là chỉ số, current, frame, lệnh, p1..p4, vĩ độ, kinh độ, độ cao —
 *       lệnh là tên MAV_CMD hoặc số của nó;</li>
 *   <li>lệnh 99 là id waypoint kiểu cũ và được viết lại thành 0;</li>
 *   <li>dòng không phân tích được sẽ được báo và bỏ qua, phần còn lại của tệp
 *       vẫn được nạp.</li>
 * </ul>
 */
public final class PlannerMission {

    /** Mission Planner writes this header and requires it in every file. */
    /** Mission Planner ghi dòng tiêu đề này và đòi hỏi nó trong mọi tệp. */
    public static final String WPL_HEADER = "QGC WPL 110";

    private final PlannerItem mHome = blankHome();
    private final List<PlannerItem> mItems = new ArrayList<>();
    private final List<String> mInvalidLines = new ArrayList<>();

    // Mission Planner's new Locationwp(): every field zero, frame included.
    // new Locationwp() của Mission Planner: mọi trường bằng 0, kể cả frame.
    private static PlannerItem blankHome() {
        PlannerItem home = new PlannerItem();
        home.frame = 0;
        home.command = 0;
        return home;
    }

    /** Home is row 0 of a mission file — the reference for every altitude. */
    /** Home là dòng 0 của tệp nhiệm vụ — gốc tham chiếu cho mọi độ cao. */
    public PlannerItem home() {
        return mHome;
    }

    /** The mission rows, in file order (the file's seq 1..n). */
    /** Các dòng nhiệm vụ, theo thứ tự tệp (seq 1..n trong tệp). */
    public List<PlannerItem> items() {
        return mItems;
    }

    /** Lines the reader could not parse, kept so the screen can report them. */
    /** Các dòng đầu đọc không phân tích được, giữ lại để màn hình có thể báo. */
    public List<String> invalidLines() {
        return mInvalidLines;
    }

    public int size() {
        return mItems.size();
    }

    public void add(PlannerItem item) {
        mItems.add(item);
    }

    public void clear() {
        mItems.clear();
        mInvalidLines.clear();
    }

    /**
     * Parse a whole file. Throws only when the header is missing — that is
     * Mission Planner's "Invalid Waypoint file". Bad rows are collected in
     * {@link #invalidLines()} and skipped, matching its per-line report.
     * <p>
     * Phân tích cả tệp. Chỉ ném lỗi khi thiếu dòng tiêu đề — đó là "Invalid
     * Waypoint file" của Mission Planner. Dòng hỏng được gom vào
     * {@link #invalidLines()} và bỏ qua, giống như việc nó báo từng dòng.
     */
    public static PlannerMission parse(String text) throws IOException {
        PlannerMission mission = new PlannerMission();
        if (text == null) {
            throw new IOException("Invalid Waypoint file");
        }
        String[] lines = text.split("\r?\n", -1);
        if (lines.length == 0 || !lines[0].contains("QGC WPL")) {
            throw new IOException("Invalid Waypoint file");
        }

        // The reference reads into one list whose row 0 is home. Keep the same
        // shape while parsing — the blank-home quirk only reads naturally that
        // way — and split home from the mission rows at the end.
        // Bản gốc đọc vào một danh sách mà dòng 0 là home. Giữ đúng hình dạng đó
        // khi phân tích — điểm lạ home trống chỉ tự nhiên theo cách đó — rồi tách
        // home khỏi các dòng nhiệm vụ ở cuối.
        List<PlannerItem> rows = new ArrayList<>();
        int wpCount = 0;
        for (int i = 1; i < lines.length; i++) {
            String line = lines[i];
            if (line.startsWith("#")) {
                continue;
            }
            // Split(new[] { '\t', ' ', ',' }, RemoveEmptyEntries). Java's
            // split keeps the empty token a leading separator makes, so drop
            // empty fields here to match RemoveEmptyEntries exactly.
            // Split(new[] { '\t', ' ', ',' }, RemoveEmptyEntries). split của Java
            // giữ token rỗng mà dấu phân cách ở đầu dòng sinh ra, nên loại các
            // trường rỗng ở đây để khớp đúng RemoveEmptyEntries.
            List<String> fields = new ArrayList<>();
            for (String field : line.split("[\t ,]+", -1)) {
                if (!field.isEmpty()) {
                    fields.add(field);
                }
            }
            String[] items = fields.toArray(new String[0]);
            if (items.length <= 9) {
                continue;
            }
            try {
                // Check to see if the first wp is index 0/home. If it is not
                // index 0, add a blank home point. (Mission Planner's words —
                // the blank is added before the row is parsed and stays when
                // the parse then fails.)
                // Kiểm tra xem waypoint đầu có phải index 0/home không. Nếu không
                // phải index 0, thêm một điểm home trống. (Nguyên văn Mission
                // Planner — dòng trống được thêm trước khi phân tích dòng và ở lại
                // kể cả khi việc phân tích thất bại.)
                if (wpCount == 0 && !"0".equals(items[0])) {
                    rows.add(blankHome());
                }

                PlannerItem temp = new PlannerItem();
                temp.frame = Integer.parseInt(items[2]);
                temp.command = PlannerItem.parseCommand(items[3]);
                if (temp.command < 0) {
                    // Enum.Parse threw there → "Line invalid".
                    // Enum.Parse ném lỗi ở đó → "Line invalid".
                    throw new IllegalArgumentException("bad command");
                }
                temp.p1 = Float.parseFloat(items[4]);
                if (temp.command == 99) {
                    // A legacy waypoint id: Mission Planner rewrites it to 0.
                    // Id waypoint kiểu cũ: Mission Planner viết lại thành 0.
                    temp.command = 0;
                }
                temp.alt = (float) Double.parseDouble(items[10]);
                temp.lat = Double.parseDouble(items[8]);
                temp.lon = Double.parseDouble(items[9]);
                temp.p2 = (float) Double.parseDouble(items[5]);
                temp.p3 = (float) Double.parseDouble(items[6]);
                temp.p4 = (float) Double.parseDouble(items[7]);

                rows.add(temp);
                wpCount++;
            } catch (Exception ex) {
                // Mission Planner shows "Line invalid" and keeps reading.
                // Mission Planner báo "Line invalid" rồi vẫn đọc tiếp.
                mission.mInvalidLines.add(line);
            }
        }

        if (!rows.isEmpty()) {
            mission.replaceHome(rows.get(0));
            for (int i = 1; i < rows.size(); i++) {
                mission.mItems.add(rows.get(i));
            }
        }
        return mission;
    }

    private void replaceHome(PlannerItem home) {
        mHome.frame = home.frame;
        mHome.command = home.command;
        mHome.p1 = home.p1;
        mHome.p2 = home.p2;
        mHome.p3 = home.p3;
        mHome.p4 = home.p4;
        mHome.lat = home.lat;
        mHome.lon = home.lon;
        mHome.alt = home.alt;
    }

    /**
     * Write the file exactly like Mission Planner's {@code savewaypoints()}:
     * the header, one home row with frame 0 and command 16, then the mission
     * rows with seq 1..n. Number formats are its string formats — seven
     * decimals for the home lat/lng, eight for the rows' p1..p4/lat/lng, six
     * for every altitude, invariant culture (dot decimal).
     * <p>
     * Ghi tệp đúng như {@code savewaypoints()} của Mission Planner: dòng tiêu
     * đề, một dòng home với frame 0 và lệnh 16, rồi các dòng nhiệm vụ đánh số
     * seq 1..n. Định dạng số là định dạng chuỗi của nó — bảy chữ số thập phân cho
     * vĩ/kinh độ home, tám cho p1..p4/vĩ/kinh độ của dòng, sáu cho mọi độ cao,
     * văn hóa bất biến (dấu chấm thập phân).
     */
    public String format() {
        StringBuilder sb = new StringBuilder();
        sb.append(WPL_HEADER).append('\n');

        // Home row: "0\t1\t0\t16\t0\t0\t0\t0\t<lat>\t<lng>\t<alt>\t1". Mission
        // Planner falls back to an all-zero row when its home fields will not
        // parse; here that is the non-finite check below.
        // Dòng home: "0\t1\t0\t16\t0\t0\t0\t0\t<lat>\t<lng>\t<alt>\t1". Mission
        // Planner chuyển sang dòng toàn số 0 khi ô home không phân tích được; ở
        // đây là phép kiểm không hữu hạn bên dưới.
        if (Double.isFinite(mHome.lat) && Double.isFinite(mHome.lon)
                && Float.isFinite(mHome.alt)) {
            sb.append(String.format(Locale.US, "0\t1\t0\t16\t0\t0\t0\t0\t%.7f\t%.7f\t%.6f\t1",
                    mHome.lat, mHome.lon, (double) mHome.alt));
        } else {
            sb.append("0\t1\t0\t0\t0\t0\t0\t0\t0\t0\t0\t1");
        }
        sb.append('\n');

        for (int a = 0; a < mItems.size(); a++) {
            PlannerItem it = mItems.get(a);
            // seq is the row index + 1; current is always 0 for mission rows.
            // seq là chỉ số dòng + 1; current luôn 0 cho dòng nhiệm vụ.
            sb.append(String.format(Locale.US,
                    "%d\t0\t%d\t%d\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.6f\t1",
                    a + 1, it.frame, it.command,
                    (double) it.p1, (double) it.p2, (double) it.p3, (double) it.p4,
                    it.lat, it.lon, (double) it.alt));
            sb.append('\n');
        }
        return sb.toString();
    }
}
