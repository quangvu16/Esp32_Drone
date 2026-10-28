package se.bitcraze.crazyfliecontrol2.gpsnav;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Point;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.drawable.Drawable;
import android.util.AttributeSet;
import android.view.MotionEvent;

import java.util.ArrayList;
import java.util.List;

import org.osmdroid.api.IGeoPoint;
import org.osmdroid.events.MapEventsReceiver;
import org.osmdroid.tileprovider.MapTileProviderBasic;
import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.CustomZoomButtonsController;
import org.osmdroid.views.MapView;
import org.osmdroid.views.Projection;
import org.osmdroid.views.overlay.CopyrightOverlay;
import org.osmdroid.views.overlay.MapEventsOverlay;
import org.osmdroid.views.overlay.Marker;
import org.osmdroid.views.overlay.Overlay;
import org.osmdroid.views.overlay.Polygon;
import org.osmdroid.views.overlay.Polyline;
import org.osmdroid.views.overlay.TilesOverlay;

/**
 * Mission Planner style interactive map for planning and flying GPS missions.
 *
 * <p>Bản đồ tương tác kiểu Mission Planner để lập kế hoạch và bay các sứ mệnh GPS.
 *
 * <p>Draws the plan (home, numbered waypoints with acceptance circles, legs), highlights the active
 * waypoint, and overlays the drone's ESTIMATED position and track. That label is not decoration:
 * the drone has no GPS and no magnetometer, so everything drawn for it is accelerometer dead
 * reckoning that drifts metres per minute (PROTOCOL.md section 6).
 *
 * <p>Vẽ kế hoạch (home, các điểm hành trình đánh số kèm vòng tròn bán kính chấp nhận, các chặng),
 * làm nổi bật điểm hành trình đang active, và lớp phủ vị trí cùng vết bay ƯỚC TÍNH của drone. Nhãn
 * đó không phải trang trí: drone không có GPS cũng không có la bàn từ, nên mọi thứ vẽ cho nó đều là
 * tự dẫn đường quán tính từ gia tốc kế, trôi vài mét mỗi phút (PROTOCOL.md mục 6).
 *
 * <p>Based on osmdroid 6.1.20. {@code Configuration.getInstance().setUserAgentValue(...)} must run
 * before this view is inflated — the hosting activity does it in {@code onCreate}.
 *
 * <p>Dựa trên osmdroid 6.1.20. {@code Configuration.getInstance().setUserAgentValue(...)} phải chạy
 * trước khi view này được nạp — activity chứa nó gọi trong {@code onCreate}.
 */
public class MissionMapView extends MapView {

    public static final int MAP_TYPE_SATELLITE = 0;
    public static final int MAP_TYPE_HYBRID = 1;
    public static final int MAP_TYPE_STREET = 2;
    public static final int MAP_TYPE_TERRAIN = 3;

    /** Callback for taps on the map. */
    /** Callback cho các cú chạm lên bản đồ. */
    public interface OnMapTapListener {
        void onMapTap(double latitude, double longitude);
    }

    /**
     * Callbacks for on-map editing with Valetudo's handle model (PROTOCOL.md section 10): the ✕
     * handle deletes the selected annotation, and any handle or body drag mutates it in place and
     * snaps on release.
     *
     * <p>Callback cho việc sửa ngay trên bản đồ theo mô hình tay nắm của Valetudo (PROTOCOL.md
     * mục 10): tay nắm ✕ xoá chú thích đang chọn, và mọi thao tác kéo tay nắm hoặc thân hình sẽ
     * sửa nó tại chỗ rồi hút lại khi nhả.
     */
    public interface OnAnnotationEditListener {
        /** The ✕ handle was tapped (chạm tay nắm ✕). */
        void onAnnotationDeleteRequested();

        /** A drag ended and the geometry has been snapped (kéo xong, hình đã được hút). */
        void onAnnotationEdited();
    }

    /**
     * Callback for placement gestures, ported from the Vacuum Map Card's MANUAL_RECTANGLE: a
     * single-finger drag rubber-bands a rectangle and the listener receives the finished corners.
     *
     * <p>Callback cho các cử chỉ đặt hình, chuyển thể từ MANUAL_RECTANGLE của Vacuum Map Card:
     * kéo một ngón tay sẽ phác một hình chữ nhật và listener nhận các góc đã hoàn tất.
     */
    public interface OnPlacementListener {
        /** A rectangle drag finished; corners are normalised to N/W/S/E (kéo xong, góc đã chuẩn hoá). */
        void onRectPlaced(double northLat, double westLon, double southLat, double eastLon);
    }

    /** No placement gesture: taps go to {@link OnMapTapListener} (không đặt gì: chạm chuyển cho listener). */
    public static final int PLACE_NONE = 0;

    /** Drag-a-rectangle placement (đặt bằng cách kéo hình chữ nhật). */
    public static final int PLACE_RECT = 1;

    private OnMapTapListener mOnMapTapListener;
    private OnAnnotationEditListener mOnAnnotationEditListener;
    private OnPlacementListener mOnPlacementListener;

    /** The selected annotation being edited by the handles, or null. */
    /** Chú thích đang chọn để các tay nắm sửa, hoặc null. */
    private MapAnnotation mEditAnnotation;

    /** Handle editing allowed? The Mission screen is read-only, the Indoor Map screen edits. */
    /** Có cho phép sửa bằng tay nắm không? Màn Mission chỉ xem, màn Indoor Map thì sửa. */
    private boolean mAnnotationsEditable = true;

    /** Placement gesture in progress ({@link #PLACE_NONE} / {@link #PLACE_RECT}). */
    /** Cử chỉ đặt hình đang chạy ({@link #PLACE_NONE} / {@link #PLACE_RECT}). */
    private int mPlacementMode = PLACE_NONE;

    // Rubber-band rectangle of an in-progress PLACE_RECT drag, in geo and screen space.
    // Hình chữ nhật phác của thao tác PLACE_RECT đang kéo, theo toạ độ địa lý và màn hình.
    private GeoPoint mPlaceStart;
    private GeoPoint mPlaceCurrent;
    private float mPlaceStartX;
    private float mPlaceStartY;

    /** Set when a second finger takes the gesture back for pinch-zoom (đặt khi ngón thứ hai lấy lại cử chỉ để phóng to). */
    private boolean mPlaceAbandoned;

    private PlacementOverlay mPlacementOverlay;

    private static final int DRAG_NONE = 0;
    private static final int DRAG_DELETE = 1;
    private static final int DRAG_ENDPOINT = 2;
    private static final int DRAG_BODY = 3;
    private static final int DRAG_ROTATE = 4;
    private static final int DRAG_SCALE = 5;

    private int mDragMode = DRAG_NONE;
    private IGeoPoint mDragLast;
    private float mDragStartX;
    private float mDragStartY;
    private boolean mDragMoved;

    /** Handle icon diameter in dp (đường kính biểu tượng tay nắm theo dp). */
    private static final float HANDLE_SIZE_DP = 28f;

    /** Touch radius around a handle, as a fraction of its size (bán kính vùng chạm). */
    private static final float HANDLE_HIT_SCALE = 0.85f;

    /** Drag starts on a shape's body within this many metres (kéo thân hình trong phạm vi này). */
    private static final double BODY_DRAG_TOLERANCE_M = 15.0;

    /** Movement in dp before a drag counts as one (số dp di chuyển trước khi tính là kéo). */
    private static final float DRAG_SLOP_DP = 4f;

    /** Movement in dp before a placement drag becomes a rectangle (số dp để thao tác đặt thành hình chữ nhật). */
    private static final float PLACE_SLOP_DP = 8f;

    /** One handle's screen placement (vị trí màn hình của một tay nắm). */
    private static final class HandleSpot {
        final int kind;
        final float x;
        final float y;

        HandleSpot(int kind, float x, float y) {
            this.kind = kind;
            this.x = x;
            this.y = y;
        }
    }

    private Marker mDroneMarker;
    private Marker mHomeMarker;
    private Marker mGuidedMarker;
    private final List<Marker> mWaypointMarkers = new ArrayList<Marker>();
    private final List<Polygon> mWaypointAcceptanceCircles = new ArrayList<Polygon>();
    private final List<AnnotationOverlay> mAnnotationOverlays = new ArrayList<AnnotationOverlay>();
    private Polyline mMissionPolyline;
    private Polyline mTrackPolyline;
    private final List<GeoPoint> mTrackPoints = new ArrayList<GeoPoint>();
    private TilesOverlay mLabelsOverlay;
    private CopyrightOverlay mCopyrightOverlay;
    private int mMapType = MAP_TYPE_SATELLITE;

    private double mDroneLatitude = Double.NaN;
    private double mDroneLongitude = Double.NaN;
    private boolean mHasCenteredOnDrone;

    private Drawable mDroneDrawable;
    private Drawable mHomeDrawable;
    private Drawable mGuidedDrawable;

    public MissionMapView(Context context) {
        this(context, null);
    }

    public MissionMapView(Context context, AttributeSet attrs) {
        super(context, attrs);
        initMap(context);
    }

    private void initMap(Context context) {
        setMultiTouchControls(true);
        getZoomController().setVisibility(CustomZoomButtonsController.Visibility.SHOW_AND_FADEOUT);
        applyMapType();

        // Zoom 17 shows the satellite backdrop at a scale where one drone's route is legible.
        // Zoom 17 hiện nền vệ tinh ở tỷ lệ mà lộ trình của một drone còn đọc được.
        getController().setZoom(17);

        mDroneDrawable = createDroneDrawable(context);
        mHomeDrawable = createHomeDrawable(context);
        mGuidedDrawable = createGuidedDrawable(context);

        MapEventsReceiver receiver =
                new MapEventsReceiver() {
                    @Override
                    public boolean singleTapConfirmedHelper(GeoPoint p) {
                        if (mOnMapTapListener != null) {
                            mOnMapTapListener.onMapTap(p.getLatitude(), p.getLongitude());
                        }
                        return true;
                    }

                    @Override
                    public boolean longPressHelper(GeoPoint p) {
                        return false;
                    }
                };
        getOverlays().add(0, new MapEventsOverlay(receiver));

        // On top of everything: the rubber band of an in-progress placement drag.
        // Nằm trên mọi lớp: khung phác của thao tác đặt hình đang kéo.
        mPlacementOverlay = new PlacementOverlay();
        getOverlays().add(mPlacementOverlay);
    }

    public void setOnMapTapListener(OnMapTapListener listener) {
        this.mOnMapTapListener = listener;
    }

    public void setOnAnnotationEditListener(OnAnnotationEditListener listener) {
        this.mOnAnnotationEditListener = listener;
    }

    public void setOnPlacementListener(OnPlacementListener listener) {
        this.mOnPlacementListener = listener;
    }

    /** Chooses the placement gesture ({@link #PLACE_NONE} / {@link #PLACE_RECT}). */
    /** Chọn cử chỉ đặt hình ({@link #PLACE_NONE} / {@link #PLACE_RECT}). */
    public void setPlacementMode(int mode) {
        this.mPlacementMode = mode;
        cancelPlacementRect();
        invalidate();
    }

    /**
     * Turns Valetudo's handle editing on or off. Read-only screens (Mission) get no handles and
     * no handle drags; the Indoor Map screen edits.
     *
     * <p>Bật/tắt việc sửa bằng tay nắm của Valetudo. Màn chỉ xem (Mission) không có tay nắm và
     * không kéo được; màn Indoor Map thì có.
     */
    public void setAnnotationsEditable(boolean editable) {
        this.mAnnotationsEditable = editable;
        if (!editable) {
            mDragMode = DRAG_NONE;
            mDragLast = null;
        }
        invalidate();
    }

    /** Switches the map backdrop, keeping the mission overlays in place. */
    /** Đổi nền bản đồ, giữ nguyên các lớp phủ của sứ mệnh. */
    public void setMapType(int type) {
        this.mMapType = type;
        applyMapType();
    }

    public int getMapType() {
        return mMapType;
    }

    private void applyMapType() {
        if (mLabelsOverlay != null) {
            getOverlays().remove(mLabelsOverlay);
            mLabelsOverlay = null;
        }

        switch (mMapType) {
            case MAP_TYPE_HYBRID:
                // GMap.NET's GoogleHybridMapProvider composites the label tile over the satellite
                // tile, so hybrid draws GOOGLE_LABELS on top of GOOGLE_SATELLITE.
                // GoogleHybridMapProvider của GMap.NET ghép tile nhãn chồng lên tile vệ tinh,
                // nên chế độ hybrid vẽ GOOGLE_LABELS lên trên GOOGLE_SATELLITE.
                setTileSource(MapTileSources.GOOGLE_SATELLITE);
                mLabelsOverlay = new TilesOverlay(
                        new MapTileProviderBasic(getContext(), MapTileSources.GOOGLE_LABELS),
                        getContext());
                mLabelsOverlay.setLoadingBackgroundColor(Color.TRANSPARENT);
                mLabelsOverlay.setLoadingLineColor(Color.TRANSPARENT);
                // Index 0 draws the labels just above the base tiles, underneath every mission
                // marker and path. MapEventsOverlay is also at 0 but paints nothing.
                // Chỉ số 0 vẽ nhãn ngay trên tile nền, nằm dưới mọi chỉ dấu và đường của sứ mệnh.
                // MapEventsOverlay cũng ở vị trí 0 nhưng không vẽ gì.
                getOverlays().add(0, mLabelsOverlay);
                break;
            case MAP_TYPE_STREET:
                setTileSource(MapTileSources.GOOGLE_STREET);
                break;
            case MAP_TYPE_TERRAIN:
                setTileSource(MapTileSources.GOOGLE_TERRAIN);
                break;
            case MAP_TYPE_SATELLITE:
            default:
                setTileSource(MapTileSources.GOOGLE_SATELLITE);
                break;
        }

        if (mCopyrightOverlay == null) {
            mCopyrightOverlay = new CopyrightOverlay(getContext());
        }
        if (!getOverlays().contains(mCopyrightOverlay)) {
            getOverlays().add(mCopyrightOverlay);
        }
        invalidate();
    }

    /**
     * Places the drone marker. The title carries the word "estimated" on purpose: this is dead
     * reckoning, not GPS.
     *
     * <p>Đặt chỉ dấu của drone. Tiêu đề cố tình mang chữ "estimated" (ước tính): đây là tự dẫn
     * đường quán tính, không phải GPS.
     */
    public void setDrone(double latitude, double longitude, double headingDegrees) {
        if (Double.isNaN(latitude) || Double.isNaN(longitude)) {
            return;
        }
        this.mDroneLatitude = latitude;
        this.mDroneLongitude = longitude;

        GeoPoint dronePoint = new GeoPoint(latitude, longitude);

        if (mDroneMarker == null) {
            mDroneMarker = new Marker(this);
            mDroneMarker.setTitle("Drone (estimated)");
            mDroneMarker.setSnippet("dead reckoning - drifts metres per minute");
            mDroneMarker.setIcon(mDroneDrawable);
            mDroneMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER);
            mDroneMarker.setFlat(true);
            getOverlays().add(mDroneMarker);
        }

        mDroneMarker.setPosition(dronePoint);
        mDroneMarker.setRotation((float) headingDegrees);

        if (!mHasCenteredOnDrone) {
            getController().setCenter(dronePoint);
            mHasCenteredOnDrone = true;
        }

        invalidate();
    }

    public void setHome(double latitude, double longitude) {
        if (Double.isNaN(latitude) || Double.isNaN(longitude)) {
            if (mHomeMarker != null) {
                getOverlays().remove(mHomeMarker);
                mHomeMarker = null;
            }
            return;
        }

        GeoPoint homePoint = new GeoPoint(latitude, longitude);
        if (mHomeMarker == null) {
            mHomeMarker = new Marker(this);
            mHomeMarker.setTitle("Home (RTL)");
            mHomeMarker.setIcon(mHomeDrawable);
            mHomeMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER);
            getOverlays().add(mHomeMarker);
        }
        mHomeMarker.setPosition(homePoint);
        invalidate();
    }

    /** Shows (or with NaN clears) the one-shot GUIDED "go to here" target. */
    /** Hiện (hoặc xoá nếu là NaN) mục tiêu GUIDED dùng một lần "go to here". */
    public void setGuidedTarget(double latitude, double longitude) {
        if (Double.isNaN(latitude) || Double.isNaN(longitude)) {
            if (mGuidedMarker != null) {
                getOverlays().remove(mGuidedMarker);
                mGuidedMarker = null;
            }
            invalidate();
            return;
        }

        GeoPoint targetPoint = new GeoPoint(latitude, longitude);
        if (mGuidedMarker == null) {
            mGuidedMarker = new Marker(this);
            mGuidedMarker.setTitle("Go to here");
            mGuidedMarker.setIcon(mGuidedDrawable);
            mGuidedMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER);
            getOverlays().add(mGuidedMarker);
        }
        mGuidedMarker.setPosition(targetPoint);
        invalidate();
    }

    public void setWaypoints(List<Waypoint> updatedWaypoints, int activeIndex) {
        for (Marker marker : mWaypointMarkers) {
            getOverlays().remove(marker);
        }
        mWaypointMarkers.clear();

        for (Polygon circle : mWaypointAcceptanceCircles) {
            getOverlays().remove(circle);
        }
        mWaypointAcceptanceCircles.clear();

        if (mMissionPolyline != null) {
            getOverlays().remove(mMissionPolyline);
            mMissionPolyline = null;
        }

        if (updatedWaypoints != null && !updatedWaypoints.isEmpty()) {
            List<GeoPoint> pathPoints = new ArrayList<GeoPoint>();

            for (int i = 0; i < updatedWaypoints.size(); i++) {
                Waypoint wp = updatedWaypoints.get(i);
                GeoPoint point = new GeoPoint(wp.getLatitude(), wp.getLongitude());
                pathPoints.add(point);

                boolean isActive = i == activeIndex;

                Polygon circle = new Polygon(this);
                circle.setPoints(
                        Polygon.pointsAsCircle(point, Math.max(1.0, wp.getAcceptanceRadiusMeters())));
                circle.setFillColor(
                        isActive ? Color.parseColor("#33FFB300") : Color.parseColor("#330288D1"));
                circle.setStrokeColor(
                        isActive ? Color.parseColor("#80FFB300") : Color.parseColor("#800288D1"));
                circle.setStrokeWidth(2f);
                getOverlays().add(circle);
                mWaypointAcceptanceCircles.add(circle);

                Marker wpMarker = new Marker(this);
                wpMarker.setPosition(point);
                wpMarker.setTitle("WP " + (i + 1) + (isActive ? " (Active)" : ""));
                wpMarker.setIcon(createWaypointDrawable(getContext(), i + 1, isActive));
                wpMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER);
                getOverlays().add(wpMarker);
                mWaypointMarkers.add(wpMarker);
            }

            mMissionPolyline = new Polyline();
            mMissionPolyline.setPoints(pathPoints);
            mMissionPolyline.setColor(Color.parseColor("#FF0288D1"));
            mMissionPolyline.setWidth(6f);
            getOverlays().add(1, mMissionPolyline);
        }

        invalidate();
    }

    /**
     * Replaces the map-annotation layer (PROTOCOL.md section 10). {@code selectedIndex} marks the
     * tapped annotation as active — it shows Valetudo's editing handles — or -1 for none. The three
     * styles are ported from Valetudo's map structures: grooved green bar for corridors, red block
     * for keep-out zones, cyan curtain waves for wind zones.
     *
     * <p>Thay lớp chú thích bản đồ (PROTOCOL.md mục 10). {@code selectedIndex} đánh dấu chú thích
     * đã chạm là active — nó hiện các tay nắm sửa của Valetudo — hoặc -1 nếu không có. Ba kiểu vẽ
     * được chuyển thể từ các cấu trúc bản đồ của Valetudo: thanh xanh lá có rãnh cho hành lang,
     * khối đỏ cho vùng cấm bay, sóng rèm xanh cyan cho vùng gió.
     */
    public void setAnnotations(List<MapAnnotation> annotations, int selectedIndex) {
        for (AnnotationOverlay overlay : mAnnotationOverlays) {
            getOverlays().remove(overlay);
        }
        mAnnotationOverlays.clear();

        mEditAnnotation = (mAnnotationsEditable && annotations != null && selectedIndex >= 0
                && selectedIndex < annotations.size()) ? annotations.get(selectedIndex) : null;

        if (annotations != null) {
            for (int i = 0; i < annotations.size(); i++) {
                mAnnotationOverlays.add(new AnnotationOverlay(annotations.get(i), i == selectedIndex));
            }
            // Above the tiles, below the plan and the markers — the zones are context, not targets.
            // Trên tile nền, dưới kế hoạch và các chỉ dấu — các vùng là bối cảnh, không phải mục tiêu.
            getOverlays().addAll(1, mAnnotationOverlays);
        }
        invalidate();
    }

    /** Replaces the whole estimated track. Points are {@code {lat, lon}} pairs in degrees. */
    /** Thay toàn bộ vết bay ước tính. Các điểm là cặp {@code {lat, lon}} tính bằng độ. */
    public void setTrack(List<double[]> trackPoints) {
        mTrackPoints.clear();
        if (trackPoints != null) {
            for (double[] pt : trackPoints) {
                mTrackPoints.add(new GeoPoint(pt[0], pt[1]));
            }
        }
        rebuildTrackPolyline();
    }

    /** Appends one estimated position fix to the breadcrumb track. */
    /** Nối thêm một vị trí ước tính vào vết bánh mì (breadcrumb track). */
    public void appendTrackPoint(double latitude, double longitude) {
        mTrackPoints.add(new GeoPoint(latitude, longitude));
        rebuildTrackPolyline();
    }

    private void rebuildTrackPolyline() {
        if (mTrackPolyline != null) {
            getOverlays().remove(mTrackPolyline);
            mTrackPolyline = null;
        }

        if (!mTrackPoints.isEmpty()) {
            mTrackPolyline = new Polyline();
            mTrackPolyline.setPoints(mTrackPoints);
            mTrackPolyline.setColor(Color.parseColor("#FF00E676")); // bright green breadcrumbs
            // (vết bánh mì màu xanh lá sáng)
            mTrackPolyline.setWidth(5f);
            getOverlays().add(1, mTrackPolyline);
        }

        invalidate();
    }

    // region custom marker icon generators

    private Drawable createWaypointDrawable(Context context, int number, boolean isActive) {
        int size = 72;
        Bitmap bitmap = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(bitmap);

        Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        fillPaint.setStyle(Paint.Style.FILL);
        fillPaint.setColor(isActive ? Color.parseColor("#FFFFB300") : Color.parseColor("#FF0288D1"));

        Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        strokePaint.setStyle(Paint.Style.STROKE);
        strokePaint.setStrokeWidth(4f);
        strokePaint.setColor(Color.WHITE);

        Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(32f);
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setFakeBoldText(true);

        canvas.drawCircle(size / 2f, size / 2f, size / 2f - 4f, fillPaint);
        canvas.drawCircle(size / 2f, size / 2f, size / 2f - 4f, strokePaint);

        float yPos = (size / 2f) - ((textPaint.descent() + textPaint.ascent()) / 2f);
        canvas.drawText(String.valueOf(number), size / 2f, yPos, textPaint);

        return new BitmapDrawable(context.getResources(), bitmap);
    }

    private Drawable createDroneDrawable(Context context) {
        int size = 80;
        Bitmap bitmap = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(bitmap);

        Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        fillPaint.setStyle(Paint.Style.FILL);
        fillPaint.setColor(Color.parseColor("#FF00E5FF"));

        Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        strokePaint.setStyle(Paint.Style.STROKE);
        strokePaint.setStrokeWidth(4f);
        strokePaint.setColor(Color.WHITE);

        // Arrow points "up" so marker rotation == compass heading.
        // Mũi tên chỉ "lên" để góc xoay của chỉ dấu == hướng mũi la bàn.
        Path arrowPath = new Path();
        arrowPath.moveTo(size / 2f, 8f);
        arrowPath.lineTo(size - 14f, size - 14f);
        arrowPath.lineTo(size / 2f, size - 26f);
        arrowPath.lineTo(14f, size - 14f);
        arrowPath.close();

        canvas.drawPath(arrowPath, fillPaint);
        canvas.drawPath(arrowPath, strokePaint);

        return new BitmapDrawable(context.getResources(), bitmap);
    }

    private Drawable createHomeDrawable(Context context) {
        return createLabelledCircle(context, "H", Color.parseColor("#FFFF6D00"));
    }

    private Drawable createGuidedDrawable(Context context) {
        return createLabelledCircle(context, "G", Color.parseColor("#FF8E24AA"));
    }

    private Drawable createLabelledCircle(Context context, String label, int fillColor) {
        int size = 72;
        Bitmap bitmap = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(bitmap);

        Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        fillPaint.setStyle(Paint.Style.FILL);
        fillPaint.setColor(fillColor);

        Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        strokePaint.setStyle(Paint.Style.STROKE);
        strokePaint.setStrokeWidth(4f);
        strokePaint.setColor(Color.WHITE);

        Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(32f);
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setFakeBoldText(true);

        canvas.drawCircle(size / 2f, size / 2f, size / 2f - 4f, fillPaint);
        canvas.drawCircle(size / 2f, size / 2f, size / 2f - 4f, strokePaint);

        float yPos = (size / 2f) - ((textPaint.descent() + textPaint.ascent()) / 2f);
        canvas.drawText(label, size / 2f, yPos, textPaint);

        return new BitmapDrawable(context.getResources(), bitmap);
    }

    // endregion

    // region annotation overlays + handle editing (Valetudo's map client structures)

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        // Placement beats everything: in a placement mode every touch is a placement gesture,
        // so taps must not select annotations or drop waypoints.
        // Đặt hình thắng tất cả: trong chế độ đặt, mọi chạm đều là cử chỉ đặt hình, nên chạm
        // không được chọn chú thích hay thả điểm hành trình.
        if (mPlacementMode != PLACE_NONE) {
            if (handlePlacementTouch(event)) {
                return true;
            }
            return super.onTouchEvent(event);
        }
        // Handles beat the map's own gestures: a touch that starts on a control handle or on the
        // selected shape's body is consumed here, so the map does not pan underneath the drag.
        // Tay nắm thắng cử chỉ của bản đồ: chạm bắt đầu trên tay nắm hoặc thân hình đang chọn sẽ
        // bị tiêu thụ ở đây, để bản đồ không cuộn theo khi đang kéo.
        MapAnnotation edit = mEditAnnotation;
        if (edit != null && mAnnotationsEditable) {
            if (event.getActionMasked() == MotionEvent.ACTION_DOWN) {
                if (beginAnnotationDrag(edit, event.getX(), event.getY())) {
                    return true;
                }
            } else if (mDragMode != DRAG_NONE) {
                handleAnnotationDrag(edit, event);
                return true;
            }
        }
        return super.onTouchEvent(event);
    }

    /**
     * The Vacuum Map Card's MANUAL_RECTANGLE gesture: one finger rubber-bands a rectangle and
     * {@link OnPlacementListener#onRectPlaced} gets the finished corners. A tap smaller than
     * {@link #PLACE_SLOP_DP} places nothing, and a second finger cancels the rectangle and hands
     * the rest of the gesture back to the map so pinch-zoom still works.
     *
     * <p>Cử chỉ MANUAL_RECTANGLE của Vacuum Map Card: một ngón tay phác hình chữ nhật và
     * {@link OnPlacementListener#onRectPlaced} nhận các góc hoàn tất. Chạm nhỏ hơn
     * {@link #PLACE_SLOP_DP} thì không đặt gì, và ngón thứ hai huỷ hình chữ nhật rồi trả cử chỉ
     * lại cho bản đồ để phóng to bằng hai ngón vẫn hoạt động.
     */
    private boolean handlePlacementTouch(MotionEvent event) {
        int action = event.getActionMasked();

        if (action == MotionEvent.ACTION_DOWN) {
            mPlaceAbandoned = false;
            cancelPlacementRect();
            Projection projection = getProjection();
            IGeoPoint p = projection.fromPixels((int) event.getX(), (int) event.getY());
            mPlaceStart = new GeoPoint(p.getLatitude(), p.getLongitude());
            mPlaceCurrent = mPlaceStart;
            mPlaceStartX = event.getX();
            mPlaceStartY = event.getY();
            invalidate();
            return true;
        }
        if (mPlaceAbandoned) {
            return false;
        }
        if (action == MotionEvent.ACTION_POINTER_DOWN || event.getPointerCount() > 1) {
            cancelPlacementRect();
            mPlaceAbandoned = true;
            invalidate();
            return false;
        }
        if (mPlaceStart == null) {
            return true;
        }
        if (action == MotionEvent.ACTION_MOVE) {
            Projection projection = getProjection();
            IGeoPoint p = projection.fromPixels((int) event.getX(), (int) event.getY());
            mPlaceCurrent = new GeoPoint(p.getLatitude(), p.getLongitude());
            invalidate();
            return true;
        }
        if (action == MotionEvent.ACTION_UP) {
            float slop = PLACE_SLOP_DP * getResources().getDisplayMetrics().density;
            boolean moved = Math.hypot(event.getX() - mPlaceStartX, event.getY() - mPlaceStartY) > slop;
            if (moved && mPlaceCurrent != null && mOnPlacementListener != null) {
                double northLat = Math.max(mPlaceStart.getLatitude(), mPlaceCurrent.getLatitude());
                double southLat = Math.min(mPlaceStart.getLatitude(), mPlaceCurrent.getLatitude());
                double westLon = Math.min(mPlaceStart.getLongitude(), mPlaceCurrent.getLongitude());
                double eastLon = Math.max(mPlaceStart.getLongitude(), mPlaceCurrent.getLongitude());
                mOnPlacementListener.onRectPlaced(northLat, westLon, southLat, eastLon);
            }
            cancelPlacementRect();
            invalidate();
            return true;
        }
        if (action == MotionEvent.ACTION_CANCEL) {
            cancelPlacementRect();
            invalidate();
            return true;
        }
        return true;
    }

    /** Drops the in-progress rubber band (bỏ khung phác đang vẽ dở). */
    private void cancelPlacementRect() {
        mPlaceStart = null;
        mPlaceCurrent = null;
    }

    /**
     * Starts a handle/body drag when the touch lands on one of Valetudo's control elements —
     * ✕ delete at p0, ✛ move at p1 for lines, ↻ rotate at p1 and ↖↘ scale at p2 for areas.
     *
     * <p>Bắt đầu kéo tay nắm/thân hình khi chạm trúng một trong các phần điều khiển của Valetudo —
     * ✕ xoá tại p0, ✛ di chuyển tại p1 với đường, ↻ xoay tại p1 và ↖↘ co giãn tại p2 với vùng.
     */
    private boolean beginAnnotationDrag(MapAnnotation annotation, float x, float y) {
        Projection projection = getProjection();
        float density = getResources().getDisplayMetrics().density;
        float hitRadius = HANDLE_SIZE_DP * density * HANDLE_HIT_SCALE;

        for (HandleSpot spot : handleSpots(annotation, projection)) {
            float dx = x - spot.x;
            float dy = y - spot.y;
            if (dx * dx + dy * dy <= hitRadius * hitRadius) {
                mDragMode = spot.kind == AnnotationHandles.DELETE ? DRAG_DELETE
                        : spot.kind == AnnotationHandles.MOVE ? DRAG_ENDPOINT
                        : spot.kind == AnnotationHandles.ROTATE ? DRAG_ROTATE
                        : DRAG_SCALE;
                mDragLast = projection.fromPixels((int) x, (int) y);
                mDragStartX = x;
                mDragStartY = y;
                mDragMoved = false;
                return true;
            }
        }

        IGeoPoint p = projection.fromPixels((int) x, (int) y);
        if (annotation.nearPoint(p.getLatitude(), p.getLongitude(), BODY_DRAG_TOLERANCE_M)) {
            mDragMode = DRAG_BODY;
            mDragLast = p;
            mDragStartX = x;
            mDragStartY = y;
            mDragMoved = false;
            return true;
        }
        return false;
    }

    /**
     * Applies one drag step exactly like Valetudo's {@code translate} branches, then snaps when
     * the finger lifts ({@code postProcess}). A tap on the ✕ handle without dragging deletes.
     *
     * <p>Áp dụng một bước kéo đúng như các nhánh {@code translate} của Valetudo, rồi hút lại khi
     * nhả tay ({@code postProcess}). Chạm tay nắm ✕ mà không kéo thì xoá.
     */
    private void handleAnnotationDrag(MapAnnotation annotation, MotionEvent event) {
        Projection projection = getProjection();
        int action = event.getActionMasked();

        if (action == MotionEvent.ACTION_MOVE) {
            IGeoPoint cur = projection.fromPixels((int) event.getX(), (int) event.getY());
            float density = getResources().getDisplayMetrics().density;
            float slop = DRAG_SLOP_DP * density;
            if (Math.hypot(event.getX() - mDragStartX, event.getY() - mDragStartY) > slop) {
                mDragMoved = true;
            }
            switch (mDragMode) {
                case DRAG_ENDPOINT:
                    annotation.movePointTo(1, cur.getLatitude(), cur.getLongitude());
                    break;
                case DRAG_BODY:
                    if (mDragLast != null) {
                        annotation.translateDegrees(
                                cur.getLatitude() - mDragLast.getLatitude(),
                                cur.getLongitude() - mDragLast.getLongitude());
                    }
                    break;
                case DRAG_ROTATE:
                    if (mDragLast != null) {
                        annotation.rotateByTouches(
                                mDragLast.getLatitude(), mDragLast.getLongitude(),
                                cur.getLatitude(), cur.getLongitude());
                    }
                    break;
                case DRAG_SCALE:
                    annotation.resizeByPointer(cur.getLatitude(), cur.getLongitude());
                    break;
                default:
                    break;
            }
            mDragLast = cur;
            invalidate();
            return;
        }

        if (action == MotionEvent.ACTION_UP || action == MotionEvent.ACTION_CANCEL) {
            if (mDragMode == DRAG_DELETE && !mDragMoved && action == MotionEvent.ACTION_UP) {
                if (mOnAnnotationEditListener != null) {
                    mOnAnnotationEditListener.onAnnotationDeleteRequested();
                }
            } else if (mDragMoved && mDragMode != DRAG_DELETE) {
                annotation.snap();
                if (mOnAnnotationEditListener != null) {
                    mOnAnnotationEditListener.onAnnotationEdited();
                }
            }
            mDragMode = DRAG_NONE;
            mDragLast = null;
            mDragMoved = false;
            invalidate();
        }
    }

    /**
     * Screen positions of the selected annotation's handles, laid out exactly like Valetudo: the
     * ✕ sits on p0, then the ✛ on p1 for a line, or the ↻ on p1 and ↖↘ on p2 for a quad.
     *
     * <p>Vị trí màn hình của các tay nắm trên chú thích đang chọn, bố trí đúng như Valetudo: ✕ tại
     * p0, rồi ✛ tại p1 với đường, hoặc ↻ tại p1 và ↖↘ tại p2 với khối tứ giác.
     */
    private static List<HandleSpot> handleSpots(MapAnnotation annotation, Projection projection) {
        List<HandleSpot> spots = new ArrayList<HandleSpot>(3);
        Point p0 = projection.toPixels(
                new GeoPoint(annotation.getPoint(0)[0], annotation.getPoint(0)[1]), null);
        spots.add(new HandleSpot(AnnotationHandles.DELETE, p0.x, p0.y));
        Point p1 = projection.toPixels(
                new GeoPoint(annotation.getPoint(1)[0], annotation.getPoint(1)[1]), null);
        if (annotation.isArea() && annotation.size() >= 4) {
            spots.add(new HandleSpot(AnnotationHandles.ROTATE, p1.x, p1.y));
            Point p2 = projection.toPixels(
                    new GeoPoint(annotation.getPoint(2)[0], annotation.getPoint(2)[1]), null);
            spots.add(new HandleSpot(AnnotationHandles.SCALE, p2.x, p2.y));
        } else {
            spots.add(new HandleSpot(AnnotationHandles.MOVE, p1.x, p1.y));
        }
        return spots;
    }

    /** The angle the handle icons rotate to — the p0→p1 edge, like {@code drawRotatedIcon}. */
    /** Góc xoay của biểu tượng tay nắm — cạnh p0→p1, như {@code drawRotatedIcon}. */
    private static float handleAngleDegrees(MapAnnotation annotation, Projection projection) {
        Point p0 = projection.toPixels(
                new GeoPoint(annotation.getPoint(0)[0], annotation.getPoint(0)[1]), null);
        Point p1 = projection.toPixels(
                new GeoPoint(annotation.getPoint(1)[0], annotation.getPoint(1)[1]), null);
        return (float) Math.toDegrees(Math.atan2(p1.y - p0.y, p1.x - p0.x));
    }

    /**
     * One annotation's layer. Colours and line language follow Valetudo's client structures:
     * {@code ThresholdClientStructure} (grooved green bar) for corridors,
     * {@code NoGoAreaClientStructure} (red block) for keep-out zones and
     * {@code RampClientStructure} (black halo under a dashed active border) plus
     * {@code CurtainClientStructure} (tapered sine waves) for wind zones. The selected shape
     * additionally wears Valetudo's editing handles.
     *
     * <p>Lớp của một chú thích. Màu và ngôn ngữ đường vẽ theo các cấu trúc client của Valetudo:
     * {@code ThresholdClientStructure} (thanh xanh lá có rãnh) cho hành lang,
     * {@code NoGoAreaClientStructure} (khối đỏ) cho vùng cấm bay và
     * {@code RampClientStructure} (viền đen dưới biên đứt nét khi active) cùng
     * {@code CurtainClientStructure} (sóng sine thon dần ở hai đầu) cho vùng gió. Hình đang chọn
     * còn mang thêm các tay nắm sửa của Valetudo.
     */
    private static class AnnotationOverlay extends Overlay {

        private final MapAnnotation mAnnotation;
        private final boolean mSelected;

        AnnotationOverlay(MapAnnotation annotation, boolean selected) {
            this.mAnnotation = annotation;
            this.mSelected = selected;
        }

        @Override
        public void draw(Canvas canvas, MapView mapView, boolean shadow) {
            if (shadow) {
                return;
            }
            Projection projection = mapView.getProjection();
            float density = mapView.getResources().getDisplayMetrics().density;
            if (mAnnotation.getType() == MapAnnotation.Type.CORRIDOR) {
                drawCorridor(canvas, projection, density);
            } else {
                drawArea(canvas, projection, density);
            }
            if (mSelected && ((MissionMapView) mapView).mAnnotationsEditable) {
                drawHandles(canvas, projection, density);
            }
        }

        /** The control handles of the active shape (các tay nắm điều khiển của hình đang active). */
        private void drawHandles(Canvas canvas, Projection projection, float density) {
            float size = HANDLE_SIZE_DP * density;
            float angle = handleAngleDegrees(mAnnotation, projection);
            for (HandleSpot spot : handleSpots(mAnnotation, projection)) {
                AnnotationHandles.draw(canvas, spot.kind, spot.x, spot.y, size, angle);
            }
        }

        /**
         * Keep-out / wind areas, ported from {@code RampClientStructure}: black halo under the
         * coloured border, and the border goes solid + dashed [8,6] while the shape is active.
         *
         * <p>Vùng cấm bay / vùng gió, chuyển thể từ {@code RampClientStructure}: viền đen nằm dưới
         * biên màu, và biên chuyển thành đặc + đứt nét [8,6] khi hình đang active.
         */
        private void drawArea(Canvas canvas, Projection projection, float density) {
            Path path = new Path();
            float left = Float.MAX_VALUE;
            float top = Float.MAX_VALUE;
            float right = -Float.MAX_VALUE;
            float bottom = -Float.MAX_VALUE;
            for (int i = 0; i < mAnnotation.size(); i++) {
                double[] point = mAnnotation.getPoint(i);
                Point px = projection.toPixels(new GeoPoint(point[0], point[1]), null);
                if (i == 0) {
                    path.moveTo(px.x, px.y);
                } else {
                    path.lineTo(px.x, px.y);
                }
                left = Math.min(left, px.x);
                top = Math.min(top, px.y);
                right = Math.max(right, px.x);
                bottom = Math.max(bottom, px.y);
            }
            path.close();

            boolean keepOut = mAnnotation.getType() == MapAnnotation.Type.KEEPOUT;
            // NoGoArea red rgb(239,68,68) / curtain cyan rgb(6,182,212).
            // Đỏ của NoGoArea rgb(239,68,68) / cyan của curtain rgb(6,182,212).
            int color = keepOut ? Color.parseColor("#EF4444") : Color.parseColor("#06B6D4");

            // RampClientStructure: fill 0.15 active / 0.1 inactive, halo rgba(0,0,0,0.6) lw 7
            // under a lw-5 border that goes solid + dashed [8,6] while active.
            Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
            fill.setStyle(Paint.Style.FILL);
            fill.setColor(mSelected
                    ? (keepOut ? 0x26EF4444 : 0x2606B6D4)   // rgba(colour, 0.15)
                    : (keepOut ? 0x1AEF4444 : 0x1A06B6D4)); // rgba(colour, 0.10)

            Paint halo = new Paint(Paint.ANTI_ALIAS_FLAG);
            halo.setStyle(Paint.Style.STROKE);
            halo.setStrokeWidth(7 * density);
            halo.setColor(0x99000000); // rgba(0,0,0,0.6)

            Paint stroke = new Paint(Paint.ANTI_ALIAS_FLAG);
            stroke.setStyle(Paint.Style.STROKE);
            stroke.setStrokeWidth(5 * density);
            if (mSelected) {
                stroke.setColor(color);
                stroke.setPathEffect(new DashPathEffect(
                        new float[] {8 * density, 6 * density}, 0f));
            } else {
                stroke.setColor(color & 0x00FFFFFF | 0xB3000000); // colour at 0.7 alpha
            }

            canvas.drawPath(path, fill);
            canvas.drawPath(path, halo);
            canvas.drawPath(path, stroke);

            if (!keepOut) {
                // Clip to the quad: the waves tile the bounding box, so a rotated
                // wind zone would otherwise leak waves past its edges.
                // Cắt theo khối tứ giác: các sóng lát theo khung bao, nên vùng gió
                // đã xoay sẽ làm sóng tràn ra ngoài mép nếu không cắt.
                int saveCount = canvas.save();
                canvas.clipPath(path);
                drawCurtainWaves(canvas, left, top, right, bottom, density);
                canvas.restoreToCount(saveCount);
            }
        }

        /**
         * CurtainClientStructure's signature waves, tiled across a wind zone: each row is a sine
         * with Valetudo's per-end fade — smoothstep over 0.35× the wavelength at both ends — and
         * the black halo lies under the cyan stroke.
         *
         * <p>Những sóng đặc trưng của CurtainClientStructure, lát khắp vùng gió: mỗi hàng là một
         * đường sine với độ mờ hai đầu của Valetudo — smoothstep trên 0.35× bước sóng ở mỗi đầu —
         * và viền đen nằm dưới nét cyan.
         */
        private void drawCurtainWaves(Canvas canvas, float left, float top, float right,
                float bottom, float density) {
            float waveLength = 24 * density; // Valetudo's 6 map units at our scale (6 đơn vị bản đồ)
            float amplitude = 4 * density; // ...and its 1 map unit of amplitude (1 đơn vị biên độ)
            float rowGap = 12 * density;
            float fadeLength = waveLength * 0.35f; // Valetudo's fadeLength (chiều dài độ mờ)
            float step = 2 * density; // Valetudo's considerHiDPI(2)

            Paint haloPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
            haloPaint.setStyle(Paint.Style.STROKE);
            haloPaint.setStrokeWidth(5 * density);
            haloPaint.setColor(0x99000000); // rgba(0,0,0,0.6)

            Paint wavePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
            wavePaint.setStyle(Paint.Style.STROKE);
            wavePaint.setStrokeWidth(3 * density);
            wavePaint.setColor(Color.parseColor("#06B6D4"));

            float width = right - left;
            if (width <= 0) {
                return;
            }
            for (float y = top + rowGap / 2f; y < bottom; y += rowGap) {
                Path wave = new Path();
                boolean started = false;
                for (float x = 0f; x <= width + 0.5f; x += step) {
                    float rawSine = (float) Math.sin((x / waveLength) * (Math.PI * 2.0));
                    float distToEdge = Math.min(x, width - x);
                    float fadeFactor = Math.max(0f, Math.min(1f, distToEdge / fadeLength));
                    float smoothScale = fadeFactor * fadeFactor * (3f - 2f * fadeFactor);
                    float wy = y + rawSine * amplitude * smoothScale;
                    float px = left + x;
                    if (!started) {
                        wave.moveTo(px, wy);
                        started = true;
                    } else {
                        wave.lineTo(px, wy);
                    }
                }
                canvas.drawPath(wave, haloPaint);
                canvas.drawPath(wave, wavePaint);
            }
        }

        /**
         * Corridors: ThresholdClientStructure's grooved green bar along the gate line — fill
         * rgba(16,185,129,0.6), border rgb(16,185,129) and 8 grooves PARALLEL to the line in
         * rgba(16,185,129,0.9).
         *
         * <p>Hành lang: thanh xanh lá có rãnh của ThresholdClientStructure dọc theo đường cổng —
         * nền rgba(16,185,129,0.6), viền rgb(16,185,129) và 8 rãnh SONG SONG với đường, màu
         * rgba(16,185,129,0.9).
         */
        private void drawCorridor(Canvas canvas, Projection projection, float density) {
            double[] a = mAnnotation.getPoint(0);
            double[] b = mAnnotation.getPoint(1);
            Point p0 = projection.toPixels(new GeoPoint(a[0], a[1]), null);
            Point p1 = projection.toPixels(new GeoPoint(b[0], b[1]), null);
            float dx = p1.x - p0.x;
            float dy = p1.y - p0.y;
            float length = (float) Math.hypot(dx, dy);
            if (length < 1f) {
                return;
            }
            float nx = -dy / length;
            float ny = dx / length;
            float half = 4 * density; // the bar's half width (nửa chiều rộng thanh)

            Path bar = new Path();
            bar.moveTo(p0.x + nx * half, p0.y + ny * half);
            bar.lineTo(p1.x + nx * half, p1.y + ny * half);
            bar.lineTo(p1.x - nx * half, p1.y - ny * half);
            bar.lineTo(p0.x - nx * half, p0.y - ny * half);
            bar.close();

            // ThresholdClientStructure: fill rgba(16,185,129,0.6), stroke rgb(16,185,129).
            Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
            fill.setStyle(Paint.Style.FILL);
            fill.setColor(0x9910B981);

            Paint stroke = new Paint(Paint.ANTI_ALIAS_FLAG);
            stroke.setStyle(Paint.Style.STROKE);
            stroke.setStrokeWidth(1 * density);
            stroke.setColor(Color.parseColor("#10B981"));

            canvas.drawPath(bar, fill);
            canvas.drawPath(bar, stroke);

            // 8 grooves running the length of the bar (8 rãnh chạy dọc thanh).
            Paint groove = new Paint(Paint.ANTI_ALIAS_FLAG);
            groove.setStyle(Paint.Style.STROKE);
            groove.setStrokeWidth(Math.max(1f, 0.5f * density));
            groove.setColor(0xE610B981); // rgba(16,185,129,0.9)
            float step = (2f * half) / 9f;
            for (int i = 1; i <= 8; i++) {
                float offset = -half + step * i;
                canvas.drawLine(
                        p0.x + nx * offset,
                        p0.y + ny * offset,
                        p1.x + nx * offset,
                        p1.y + ny * offset,
                        groove);
            }
        }
    }

    /**
     * The rubber band of an in-progress PLACE_RECT drag, drawn in the Vacuum Map Card's selection
     * colours: solid emerald edge rgba(16,185,129,0.9) over a faint emerald fill
     * rgba(16,185,129,0.15).
     *
     * <p>Khung phác của thao tác PLACE_RECT đang kéo, vẽ theo màu chọn của Vacuum Map Card: viền
     * xanh ngọc đặc rgba(16,185,129,0.9) trên nền xanh ngọc nhạt rgba(16,185,129,0.15).
     */
    private class PlacementOverlay extends Overlay {

        @Override
        public void draw(Canvas canvas, MapView mapView, boolean shadow) {
            if (shadow || mPlaceStart == null || mPlaceCurrent == null) {
                return;
            }
            Projection projection = mapView.getProjection();
            float density = mapView.getResources().getDisplayMetrics().density;
            Point a = projection.toPixels(mPlaceStart, null);
            Point b = projection.toPixels(mPlaceCurrent, null);
            float left = Math.min(a.x, b.x);
            float top = Math.min(a.y, b.y);
            float right = Math.max(a.x, b.x);
            float bottom = Math.max(a.y, b.y);

            Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
            fill.setStyle(Paint.Style.FILL);
            fill.setColor(0x2610B981); // rgba(16,185,129,0.15)

            Paint stroke = new Paint(Paint.ANTI_ALIAS_FLAG);
            stroke.setStyle(Paint.Style.STROKE);
            stroke.setStrokeWidth(2 * density);
            stroke.setColor(0xE610B981); // rgba(16,185,129,0.9)

            canvas.drawRect(left, top, right, bottom, fill);
            canvas.drawRect(left, top, right, bottom, stroke);
        }
    }

    // endregion
}
