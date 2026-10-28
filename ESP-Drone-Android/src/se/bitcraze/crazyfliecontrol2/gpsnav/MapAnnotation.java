package se.bitcraze.crazyfliecontrol2.gpsnav;

import java.util.ArrayList;
import java.util.List;

/**
 * A map annotation: something on the map the mission must be aware of that cannot be reliably
 * auto-determined. Ported from Valetudo's map annotations ({@code hypfer/valetudo} 2026.07.0,
 * {@code ValetudoMapAnnotation}): the same generic concept — {@code type} plus {@code points[]},
 * at least two points — re-specified for a flying robot.
 *
 * <p>Một chú thích bản đồ: một thứ trên bản đồ mà sứ mệnh cần biết nhưng không thể tự đoán một
 * cách đáng tin cậy. Chuyển thể từ map annotations của Valetudo ({@code hypfer/valetudo} 2026.07.0,
 * {@code ValetudoMapAnnotation}): cùng một khái niệm tổng quát — {@code type} cộng {@code points[]},
 * ít nhất hai điểm — được định nghĩa lại cho robot bay.
 *
 * <p>These things existing in the map make the mission runner behave differently in some way
 * relating to the location they are at (PROTOCOL.md section 10): a keep-out zone holds the drone
 * that drifts into it, a fly-through corridor is the only sanctioned crossing and the leg is
 * routed through it, a wind zone demands an arrival confirmation before the runner advances.
 *
 * <p>Việc các thứ này tồn tại trên bản đồ khiến bộ chạy sứ mệnh cư xử khác đi tại vị trí của
 * chúng (PROTOCOL.md mục 10): vùng cấm bay giữ drone lạc vào trong, hành lang cho phép bay qua
 * là lối băng qua duy nhất được chấp thuận và chặng bay được định tuyến xuyên qua nó, vùng gió
 * đòi hỏi xác nhận đã tới trước khi bộ chạy chuyển tiếp.
 *
 * <p>Line types (corridors) carry 2 points; area types (keep-out, wind) carry the 4 corners of a
 * quad, in the order min/min, max/min, max/max, min/max when placed — like Valetudo's ramp
 * structure. Placement builds the shape from two taps, then Valetudo's on-map handles edit it:
 * ✕ deletes, ✛ drags a line's endpoint, ↻ rotates and ↖↘ scales the areas (see {@link #snap()}).
 *
 * <p>Các kiểu đường (hành lang) mang 2 điểm; các kiểu vùng (cấm bay, gió) mang 4 góc của khối
 * tứ giác, theo thứ tự min/min, max/min, max/max, min/max khi đặt — như cấu trúc ramp của
 * Valetudo. Đặt hình từ hai lần chạm, sau đó các tay nắm trên bản đồ của Valetudo sửa nó:
 * ✕ để xoá, ✛ kéo đầu mút của đường, ↻ xoay và ↖↘ co giãn các vùng (xem {@link #snap()}).
 */
public class MapAnnotation {

    /** The annotation kinds. Wire values ride the mission file (PROTOCOL.md section 10). */
    /** Các loại chú thích. Giá trị truyền nằm trong tệp sứ mệnh (PROTOCOL.md mục 10). */
    public enum Type {
        /** Area the drone must not fly over (people, trees, a building the tiles do not show). */
        /** Vùng drone không được bay qua (người, cây, tòa nhà mà bản đồ không hiển thị). */
        KEEPOUT(0, "Keep-out zone"),
        /** Line: the gate or gap that exists but the map cannot show — the sanctioned crossing. */
        /** Đường: khe hở hoặc cổng tồn tại nhưng bản đồ không hiển thị — lối băng qua được chấp thuận. */
        CORRIDOR(1, "Fly-through corridor"),
        /** Area of unsteady air the tiles look calm in — arrive with confirmation. */
        /** Vùng khí không ổn định trong khi bản đồ trông phẳng lặng — tới nơi phải xác nhận. */
        WIND(2, "Wind zone");

        /** Value in the mission file's {@code p1} column. */
        /** Giá trị trong cột {@code p1} của tệp sứ mệnh. */
        public final int wireValue;

        /** Name used on the map, the menus and every status message. */
        /** Tên dùng trên bản đồ, trong menu và mọi thông báo trạng thái. */
        public final String label;

        Type(int wireValue, String label) {
            this.wireValue = wireValue;
            this.label = label;
        }

        public static Type fromWire(int value) {
            for (Type type : values()) {
                if (type.wireValue == value) {
                    return type;
                }
            }
            return null;
        }
    }

    private final Type mType;
    private final List<double[]> mPoints;

    /**
     * @param type   the annotation kind
     * @param points {@code {latitude, longitude}} pairs in degrees; at least 2, exactly 4 for
     *               the area types (the rectangle's corners)
     * @param points các cặp {@code {vĩ độ, kinh độ}} tính bằng độ; ít nhất 2, đúng 4 với các
     *               kiểu vùng (các góc của hình chữ nhật)
     */
    public MapAnnotation(Type type, List<double[]> points) {
        if (type == null) {
            throw new IllegalArgumentException("annotation requires a type");
        }
        if (points == null || points.size() < 2) {
            // Valetudo's rule, kept verbatim: annotations require at least 2 points.
            // Quy tắc của Valetudo, giữ nguyên: chú thích cần ít nhất 2 điểm.
            throw new IllegalArgumentException("annotations require at least 2 points");
        }
        this.mType = type;
        List<double[]> copy = new ArrayList<double[]>(points.size());
        for (double[] point : points) {
            copy.add(new double[] {point[0], point[1]});
        }
        this.mPoints = copy;
    }

    /** Two-tap line (corridor): the taps are the two endpoints. */
    /** Đường từ hai lần chạm (hành lang): hai lần chạm là hai đầu mút. */
    public static MapAnnotation line(Type type, double lat0, double lon0, double lat1, double lon1) {
        List<double[]> points = new ArrayList<double[]>(2);
        points.add(new double[] {lat0, lon0});
        points.add(new double[] {lat1, lon1});
        return new MapAnnotation(type, points);
    }

    /**
     * Two-tap area: the taps are opposite corners; the rect is axis-aligned and expanded to the
     * 4-point corner order the areas carry.
     *
     * <p>Vùng từ hai lần chạm: hai lần chạm là hai góc đối diện; hình chữ nhật thẳng trục và được
     * mở rộng sang thứ tự 4 góc mà các vùng mang.
     */
    public static MapAnnotation rectFromCorners(
            Type type, double lat0, double lon0, double lat1, double lon1) {
        double minLat = Math.min(lat0, lat1);
        double maxLat = Math.max(lat0, lat1);
        double minLon = Math.min(lon0, lon1);
        double maxLon = Math.max(lon0, lon1);
        List<double[]> points = new ArrayList<double[]>(4);
        points.add(new double[] {minLat, minLon});
        points.add(new double[] {maxLat, minLon});
        points.add(new double[] {maxLat, maxLon});
        points.add(new double[] {minLat, maxLon});
        return new MapAnnotation(type, points);
    }

    public Type getType() {
        return mType;
    }

    public List<double[]> getPoints() {
        return mPoints;
    }

    public double[] getPoint(int index) {
        return mPoints.get(index);
    }

    public int size() {
        return mPoints.size();
    }

    /** Whether this is an area type (4 corners) rather than a 2-point line. */
    /** Có phải kiểu vùng (4 góc) thay vì đường 2 điểm hay không. */
    public boolean isArea() {
        return mType == Type.KEEPOUT || mType == Type.WIND;
    }

    /** Centre of the geometry — the via point a leg is routed through for corridors. */
    /** Tâm của hình học — điểm trung gian mà chặng bay được định tuyến qua với hành lang. */
    public double[] center() {
        double lat = 0.0;
        double lon = 0.0;
        for (double[] point : mPoints) {
            lat += point[0];
            lon += point[1];
        }
        return new double[] {lat / mPoints.size(), lon / mPoints.size()};
    }

    /**
     * Whether the point is within {@code toleranceMeters} of this annotation — the map's tap
     * hit-test: a tap inside an area selects it, and a tap near a corridor line selects it.
     *
     * <p>Điểm có nằm trong {@code toleranceMeters} của chú thích này hay không — phép thử chạm của
     * bản đồ: chạm vào trong một vùng để chọn nó, và chạm gần đường hành lang để chọn nó.
     */
    public boolean nearPoint(double lat, double lon, double toleranceMeters) {
        double latRef = lat;
        double[] p = toXY(lat, lon, latRef);
        double tol = toleranceMeters / METERS_PER_DEGREE;
        if (mType == Type.CORRIDOR) {
            double[] g = toXY(mPoints.get(0)[0], mPoints.get(0)[1], latRef);
            double[] h = toXY(mPoints.get(1)[0], mPoints.get(1)[1], latRef);
            return pointSegmentDistance(p, g, h) <= tol;
        }
        if (containsPoint(lat, lon)) {
            return true;
        }
        double[] poly = polygonXY(latRef);
        int n = poly.length / 2;
        for (int i = 0; i < n; i++) {
            double[] g = new double[] {poly[2 * i], poly[2 * i + 1]};
            double[] h = new double[] {poly[2 * ((i + 1) % n)], poly[2 * ((i + 1) % n) + 1]};
            if (pointSegmentDistance(p, g, h) <= tol) {
                return true;
            }
        }
        return false;
    }

    // region planar geometry
    //
    // Zones are tens of metres across, so every test runs in a local plane (longitude scaled by
    // cos(latitude), like GeoMath.distanceMeters). Good to centimetres at these sizes.
    // Các vùng rộng vài chục mét, nên mọi phép thử chạy trong một mặt phẳng cục bộ (kinh độ nhân
    // cos(vĩ độ), như GeoMath.distanceMeters). Chính xác đến centimét ở kích thước này.

    /** Metres per degree on both local-plane axes (the plane keeps degrees on both). */
    /** Mét mỗi độ trên cả hai trục mặt phẳng cục bộ (mặt phẳng giữ độ trên cả hai trục). */
    private static final double METERS_PER_DEGREE = 111320.0;

    /** Whether (lat, lon) lies inside this annotation. Areas only — a line contains nothing. */
    /** Có nằm trong chú thích này hay không. Chỉ với vùng — một đường không chứa gì cả. */
    public boolean containsPoint(double lat, double lon) {
        if (!isArea()) {
            return false;
        }
        double[] p = toXY(lat, lon, lat);
        double[] poly = polygonXY(lat);
        return pointInPolygon(p[0], p[1], poly);
    }

    /** Whether the straight leg between the two points touches this annotation. */
    /** Chặng thẳng giữa hai điểm có chạm vào chú thích này hay không. */
    public boolean crossesSegment(double lat0, double lon0, double lat1, double lon1) {
        return segmentCrossing(lat0, lon0, lat1, lon1) != null;
    }

    /**
     * First point where the leg enters this annotation, or null when it misses it. For corridors
     * that is where the leg crosses the gate line (the via point); for areas where it enters the
     * rectangle.
     *
     * <p>Điểm mà chặng bay đi vào chú thích này, hoặc null khi không chạm. Với hành lang đó là
     * nơi chặng cắt qua đường cổng (điểm trung gian); với vùng là nơi đi vào hình chữ nhật.
     */
    public double[] segmentCrossing(double lat0, double lon0, double lat1, double lon1) {
        double latRef = (lat0 + lat1) / 2.0;
        double[] a = toXY(lat0, lon0, latRef);
        double[] b = toXY(lat1, lon1, latRef);

        if (mType == Type.CORRIDOR) {
            double[] g = toXY(mPoints.get(0)[0], mPoints.get(0)[1], latRef);
            double[] h = toXY(mPoints.get(1)[0], mPoints.get(1)[1], latRef);
            double[] hit = segmentIntersection(a, b, g, h);
            return hit == null ? null : fromXY(hit, latRef);
        }

        // Area: the first polygon edge the leg crosses, or the leg starts inside.
        // Vùng: cạnh đầu tiên của đa giác mà chặng cắt qua, hoặc chặng bắt đầu bên trong.
        if (containsPoint(lat0, lon0)) {
            return new double[] {lat0, lon0};
        }
        double[] poly = polygonXY(latRef);
        int n = poly.length / 2;
        double[] best = null;
        double bestT = Double.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            double[] g = new double[] {poly[2 * i], poly[2 * i + 1]};
            double[] h = new double[] {poly[2 * ((i + 1) % n)], poly[2 * ((i + 1) % n) + 1]};
            double[] hit = segmentIntersection(a, b, g, h);
            if (hit != null) {
                double t = (hit[0] - a[0]) * (b[0] - a[0]) + (hit[1] - a[1]) * (b[1] - a[1]);
                if (t < bestT) {
                    bestT = t;
                    best = hit;
                }
            }
        }
        return best == null ? null : fromXY(best, latRef);
    }

    /** Local-plane x/y of a point; longitude is scaled by cos(latRef) to keep metres even. */
    /** Tọa độ x/y mặt phẳng cục bộ của một điểm; kinh độ nhân cos(latRef) để giữ mét đều nhau. */
    private static double[] toXY(double lat, double lon, double latRef) {
        return new double[] {
            lon * Math.cos(Math.toRadians(latRef)),
            lat
        };
    }

    private static double[] fromXY(double[] xy, double latRef) {
        return new double[] {
            xy[1],
            xy[0] / Math.cos(Math.toRadians(latRef))
        };
    }

    private double[] polygonXY(double latRef) {
        double[] poly = new double[mPoints.size() * 2];
        for (int i = 0; i < mPoints.size(); i++) {
            double[] xy = toXY(mPoints.get(i)[0], mPoints.get(i)[1], latRef);
            poly[2 * i] = xy[0];
            poly[2 * i + 1] = xy[1];
        }
        return poly;
    }

    private static boolean pointInPolygon(double x, double y, double[] poly) {
        int n = poly.length / 2;
        boolean inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = poly[2 * i];
            double yi = poly[2 * i + 1];
            double xj = poly[2 * j];
            double yj = poly[2 * j + 1];
            if ((yi > y) != (yj > y)
                    && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
                inside = !inside;
            }
        }
        return inside;
    }

    /**
     * Intersection of segments ab and cd in the local plane, or null when they do not cross.
     * Giao của hai đoạn ab và cd trong mặt phẳng cục bộ, hoặc null khi chúng không cắt nhau.
     */
    private static double[] segmentIntersection(double[] a, double[] b, double[] c, double[] d) {
        double r0 = b[0] - a[0];
        double r1 = b[1] - a[1];
        double s0 = d[0] - c[0];
        double s1 = d[1] - c[1];
        double denom = r0 * s1 - r1 * s0;
        if (Math.abs(denom) < 1e-12) {
            return null; // parallel (song song)
        }
        double t = ((c[0] - a[0]) * s1 - (c[1] - a[1]) * s0) / denom;
        double u = ((c[0] - a[0]) * r1 - (c[1] - a[1]) * r0) / denom;
        if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) {
            return null;
        }
        return new double[] {a[0] + t * r0, a[1] + t * r1};
    }

    /** Distance from point p to segment ab in the local plane. */
    /** Khoảng cách từ điểm p đến đoạn ab trong mặt phẳng cục bộ. */
    private static double pointSegmentDistance(double[] p, double[] a, double[] b) {
        double abx = b[0] - a[0];
        double aby = b[1] - a[1];
        double apx = p[0] - a[0];
        double apy = p[1] - a[1];
        double ab2 = abx * abx + aby * aby;
        double t = ab2 == 0.0 ? 0.0 : (apx * abx + apy * aby) / ab2;
        t = Math.max(0.0, Math.min(1.0, t));
        double dx = apx - t * abx;
        double dy = apy - t * aby;
        return Math.hypot(dx, dy);
    }

    // endregion

    // region handle editing (Valetudo's LineClientStructure / RampClientStructure model)
    //
    // Valetudo edits annotations on the map itself: a ✕ handle deletes, a ✛ handle moves a
    // line's endpoint, ↻ rotates a quad and ↖↘ scales it; dragging the body translates. Every
    // drag is snapped when released (postProcess): integer sizes and a 5° angle. These methods
    // are that model's geometry, in local-plane metres.
    // Valetudo sửa chú thích ngay trên bản đồ: tay nắm ✕ để xoá, tay nắm ✛ để kéo đầu mút của
    // đường, ↻ để xoay khối và ↖↘ để co giãn; kéo thân hình để di chuyển. Mọi thao tác kéo được
    // hút lại khi nhả (postProcess): kích thước nguyên mét và góc bội của 5°. Các phương thức này
    // là phần hình học của mô hình đó, theo mét trong mặt phẳng cục bộ.

    /** Shortest line we keep in metres — shorter than this the shape cannot be grabbed again. */
    /** Đường ngắn nhất giữ lại, theo mét — ngắn hơn sẽ không nắm lại được hình. */
    public static final double MIN_LINE_METERS = 1.0;

    /** Shortest area edge in metres (Valetudo clamps its ramp the same way). */
    /** Cạnh vùng ngắn nhất, theo mét (Valetudo cũng kẹp ramp như vậy). */
    public static final double MIN_AREA_METERS = 2.0;

    /**
     * Body drag: move every point by a lat/lon delta in degrees — Valetudo's
     * {@code LineClientStructure.translate} / {@code RampClientStructure.translate} body branch.
     *
     * <p>Kéo thân hình: dời mọi điểm theo chênh lệch vĩ/kinh độ — nhánh thân hình trong
     * {@code translate} của Valetudo.
     */
    public void translateDegrees(double dLat, double dLon) {
        for (double[] point : mPoints) {
            point[0] += dLat;
            point[1] += dLon;
        }
    }

    /**
     * Endpoint drag (the line's ✛ handle): reposition one vertex — Valetudo's line resize branch.
     *
     * <p>Kéo đầu mút (tay nắm ✛ của đường): đặt lại một đỉnh — nhánh co giãn của đường trong
     * Valetudo.
     */
    public void movePointTo(int index, double lat, double lon) {
        double[] point = mPoints.get(index);
        point[0] = lat;
        point[1] = lon;
    }

    /**
     * Rotate handle: rotate every point about the centroid by the angle the pointer swept around
     * it — Valetudo's {@code RampClientStructure} rotate branch.
     *
     * <p>Tay nắm xoay: xoay mọi điểm quanh tâm theo góc mà con trỏ đã quét — nhánh xoay của
     * {@code RampClientStructure} trong Valetudo.
     */
    public void rotateByTouches(double lastLat, double lastLon, double curLat, double curLon) {
        double latRef = center()[0];
        double[] c = toXYMeters(center()[0], center()[1], latRef);
        double[] last = toXYMeters(lastLat, lastLon, latRef);
        double[] cur = toXYMeters(curLat, curLon, latRef);
        double delta = Math.atan2(cur[1] - c[1], cur[0] - c[0])
                - Math.atan2(last[1] - c[1], last[0] - c[0]);
        if (delta > Math.PI) {
            delta -= 2.0 * Math.PI;
        } else if (delta < -Math.PI) {
            delta += 2.0 * Math.PI;
        }
        double cos = Math.cos(delta);
        double sin = Math.sin(delta);
        for (double[] point : mPoints) {
            double[] xy = toXYMeters(point[0], point[1], latRef);
            double dx = xy[0] - c[0];
            double dy = xy[1] - c[1];
            double[] rotated = fromXYMeters(
                    new double[] {c[0] + (dx * cos - dy * sin), c[1] + (dx * sin + dy * cos)},
                    latRef);
            point[0] = rotated[0];
            point[1] = rotated[1];
        }
    }

    /**
     * Scale handle: rebuild the far corners so the quad reaches the pointer — Valetudo's resize
     * branch (project the pointer onto the two axes from p0, each extent floored at
     * {@link #MIN_AREA_METERS}).
     *
     * <p>Tay nắm co giãn: dựng lại các góc xa để khối chạm tới con trỏ — nhánh co giãn của
     * Valetudo (chiếu con trỏ lên hai trục từ p0, mỗi chiều kẹp tối thiểu {@link #MIN_AREA_METERS}).
     */
    public void resizeByPointer(double pointerLat, double pointerLon) {
        if (!isArea() || mPoints.size() < 4) {
            return;
        }
        double latRef = center()[0];
        double[] p0 = toXYMeters(mPoints.get(0)[0], mPoints.get(0)[1], latRef);
        double[] p1 = toXYMeters(mPoints.get(1)[0], mPoints.get(1)[1], latRef);
        double[] p3 = toXYMeters(mPoints.get(3)[0], mPoints.get(3)[1], latRef);
        double[] ptr = toXYMeters(pointerLat, pointerLon, latRef);

        double lenW = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]);
        double lenH = Math.hypot(p3[0] - p0[0], p3[1] - p0[1]);
        double[] uW = lenW < 1e-6
                ? new double[] {1.0, 0.0}
                : new double[] {(p1[0] - p0[0]) / lenW, (p1[1] - p0[1]) / lenW};
        double[] uH = lenH < 1e-6
                ? new double[] {0.0, 1.0}
                : new double[] {(p3[0] - p0[0]) / lenH, (p3[1] - p0[1]) / lenH};

        double mouseVecX = ptr[0] - p0[0];
        double mouseVecY = ptr[1] - p0[1];
        double newWidth = Math.max(MIN_AREA_METERS, mouseVecX * uW[0] + mouseVecY * uW[1]);
        double newHeight = Math.max(MIN_AREA_METERS, mouseVecX * uH[0] + mouseVecY * uH[1]);

        writeAreaCorners(p0, uW, uH, newWidth, newHeight, latRef);
    }

    /** Rebuild p1/p2/p3 from p0 along the two unit axes (dựng lại p1/p2/p3 từ p0 theo hai trục). */
    private void writeAreaCorners(double[] p0, double[] uW, double[] uH,
            double width, double height, double latRef) {
        double[][] corners = new double[][] {
            p0,
            {p0[0] + width * uW[0], p0[1] + width * uW[1]},
            {p0[0] + width * uW[0] + height * uH[0], p0[1] + width * uW[1] + height * uH[1]},
            {p0[0] + height * uH[0], p0[1] + height * uH[1]}
        };
        for (int i = 0; i < 4; i++) {
            double[] point = fromXYMeters(corners[i], latRef);
            mPoints.get(i)[0] = point[0];
            mPoints.get(i)[1] = point[1];
        }
    }

    /**
     * Snap when the drag ends (Valetudo's {@code postProcess}): round p0 to whole metres, snap
     * the angle to 5° and the sizes to whole metres, and never leave a shape too small to edit.
     *
     * <p>Hút lại khi thả tay (postProcess của Valetudo): làm tròn p0 về mét nguyên, hút góc về
     * bội 5° và kích thước về mét nguyên, không để hình nhỏ quá mức còn sửa được.
     */
    public void snap() {
        double latRef = center()[0];
        if (isArea() && mPoints.size() >= 4) {
            snapArea(latRef);
        } else {
            snapLine(latRef);
        }
    }

    /** Line snap (Valetudo's {@code LineClientStructure.postProcess}). */
    /** Hút đường (postProcess của {@code LineClientStructure}). */
    private void snapLine(double latRef) {
        double[] p0 = toXYMeters(mPoints.get(0)[0], mPoints.get(0)[1], latRef);
        double[] p1 = toXYMeters(mPoints.get(1)[0], mPoints.get(1)[1], latRef);
        p0[0] = Math.round(p0[0]);
        p0[1] = Math.round(p0[1]);

        double deltaX = Math.abs(p0[0] - p1[0]);
        double deltaY = Math.abs(p0[1] - p1[1]);
        double distance = Math.hypot(deltaX, deltaY);

        // Keep the line long enough to grab again (Valetudo nudges and retries).
        // Giữ đường đủ dài để nắm lại được (Valetudo dịch nhẹ rồi thử lại).
        for (int guard = 0; distance < MIN_LINE_METERS && guard < 8; guard++) {
            if (p1[0] > p0[0]) {
                p1[0]++;
            } else if (p1[0] < p0[0]) {
                p1[0]--;
            } else {
                p1[0]++;
            }
            if (p1[1] > p0[1]) {
                p1[1]++;
            } else if (p1[1] < p0[1]) {
                p1[1]--;
            } else {
                p1[1]++;
            }
            deltaX = Math.abs(p0[0] - p1[0]);
            deltaY = Math.abs(p0[1] - p1[1]);
            distance = Math.hypot(deltaX, deltaY);
        }

        double angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double newAngle = Math.round(angle / 5.0) * 5.0;
        double roundedDistance = Math.max(MIN_LINE_METERS, Math.round(distance));

        double xOffset = roundedDistance * Math.cos(Math.toRadians(newAngle));
        double yOffset = roundedDistance * Math.sin(Math.toRadians(newAngle));
        if (p0[0] > p1[0]) {
            xOffset = -xOffset;
        }
        if (p0[1] > p1[1]) {
            yOffset = -yOffset;
        }

        double[] end = fromXYMeters(new double[] {p0[0] + xOffset, p0[1] + yOffset}, latRef);
        double[] start = fromXYMeters(p0, latRef);
        mPoints.get(0)[0] = start[0];
        mPoints.get(0)[1] = start[1];
        mPoints.get(1)[0] = end[0];
        mPoints.get(1)[1] = end[1];
    }

    /** Area snap (Valetudo's {@code RampClientStructure.postProcess}). */
    /** Hút vùng (postProcess của {@code RampClientStructure}). */
    private void snapArea(double latRef) {
        double[][] xy = new double[4][];
        for (int i = 0; i < 4; i++) {
            xy[i] = toXYMeters(mPoints.get(i)[0], mPoints.get(i)[1], latRef);
        }

        double cx = (xy[0][0] + xy[1][0] + xy[2][0] + xy[3][0]) / 4.0;
        double cy = (xy[0][1] + xy[1][1] + xy[2][1] + xy[3][1]) / 4.0;

        double wTop = Math.hypot(xy[1][0] - xy[0][0], xy[1][1] - xy[0][1]);
        double wBot = Math.hypot(xy[2][0] - xy[3][0], xy[2][1] - xy[3][1]);
        double width = (wTop + wBot) / 2.0;

        double hLeft = Math.hypot(xy[3][0] - xy[0][0], xy[3][1] - xy[0][1]);
        double hRight = Math.hypot(xy[2][0] - xy[1][0], xy[2][1] - xy[1][1]);
        double height = (hLeft + hRight) / 2.0;

        double angleRad = Math.atan2(xy[1][1] - xy[0][1], xy[1][0] - xy[0][0]);
        double snappedDeg = Math.round(Math.toDegrees(angleRad) / 5.0) * 5.0;
        angleRad = Math.toRadians(snappedDeg);

        double rW = Math.max(MIN_AREA_METERS, Math.round(width));
        double rH = Math.max(MIN_AREA_METERS, Math.round(height));

        double snappedLeft = Math.round(cx - (rW / 2.0));
        double snappedTop = Math.round(cy - (rH / 2.0));
        double rCx = snappedLeft + (rW / 2.0);
        double rCy = snappedTop + (rH / 2.0);

        double halfW = rW / 2.0;
        double halfH = rH / 2.0;
        double cos = Math.cos(angleRad);
        double sin = Math.sin(angleRad);

        double[][] corners = new double[][] {
            {rCx + (-halfW * cos - -halfH * sin), rCy + (-halfW * sin + -halfH * cos)},
            {rCx + (halfW * cos - -halfH * sin), rCy + (halfW * sin + -halfH * cos)},
            {rCx + (halfW * cos - halfH * sin), rCy + (halfW * sin + halfH * cos)},
            {rCx + (-halfW * cos - halfH * sin), rCy + (-halfW * sin + halfH * cos)}
        };
        for (int i = 0; i < 4; i++) {
            double[] point = fromXYMeters(corners[i], latRef);
            mPoints.get(i)[0] = point[0];
            mPoints.get(i)[1] = point[1];
        }
    }

    /** Local-plane x/y in metres (tọa độ x/y mặt phẳng cục bộ theo mét). */
    private static double[] toXYMeters(double lat, double lon, double latRef) {
        return new double[] {
            lon * Math.cos(Math.toRadians(latRef)) * METERS_PER_DEGREE,
            lat * METERS_PER_DEGREE
        };
    }

    /** Inverse of {@link #toXYMeters} (nghịch đảo của {@link #toXYMeters}). */
    private static double[] fromXYMeters(double[] xy, double latRef) {
        return new double[] {
            xy[1] / METERS_PER_DEGREE,
            xy[0] / (METERS_PER_DEGREE * Math.cos(Math.toRadians(latRef)))
        };
    }

    // endregion
}
