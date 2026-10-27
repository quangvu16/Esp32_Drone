package se.bitcraze.crazyfliecontrol2.gpsnav;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.drawable.Drawable;
import android.util.AttributeSet;

import java.util.ArrayList;
import java.util.List;

import org.osmdroid.events.MapEventsReceiver;
import org.osmdroid.tileprovider.MapTileProviderBasic;
import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.CustomZoomButtonsController;
import org.osmdroid.views.MapView;
import org.osmdroid.views.overlay.CopyrightOverlay;
import org.osmdroid.views.overlay.MapEventsOverlay;
import org.osmdroid.views.overlay.Marker;
import org.osmdroid.views.overlay.Polygon;
import org.osmdroid.views.overlay.Polyline;
import org.osmdroid.views.overlay.TilesOverlay;

/**
 * Mission Planner style interactive map for planning and flying GPS missions.
 *
 * <p>Draws the plan (home, numbered waypoints with acceptance circles, legs), highlights the active
 * waypoint, and overlays the drone's ESTIMATED position and track. That label is not decoration:
 * the drone has no GPS and no magnetometer, so everything drawn for it is accelerometer dead
 * reckoning that drifts metres per minute (PROTOCOL.md section 6).
 *
 * <p>Based on osmdroid 6.1.20. {@code Configuration.getInstance().setUserAgentValue(...)} must run
 * before this view is inflated — the hosting activity does it in {@code onCreate}.
 */
public class MissionMapView extends MapView {

    public static final int MAP_TYPE_SATELLITE = 0;
    public static final int MAP_TYPE_HYBRID = 1;
    public static final int MAP_TYPE_STREET = 2;
    public static final int MAP_TYPE_TERRAIN = 3;

    /** Callback for taps on the map. */
    public interface OnMapTapListener {
        void onMapTap(double latitude, double longitude);
    }

    private OnMapTapListener mOnMapTapListener;

    private Marker mDroneMarker;
    private Marker mHomeMarker;
    private Marker mGuidedMarker;
    private final List<Marker> mWaypointMarkers = new ArrayList<Marker>();
    private final List<Polygon> mWaypointAcceptanceCircles = new ArrayList<Polygon>();
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
    }

    public void setOnMapTapListener(OnMapTapListener listener) {
        this.mOnMapTapListener = listener;
    }

    /** Switches the map backdrop, keeping the mission overlays in place. */
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
                setTileSource(MapTileSources.GOOGLE_SATELLITE);
                mLabelsOverlay = new TilesOverlay(
                        new MapTileProviderBasic(getContext(), MapTileSources.GOOGLE_LABELS),
                        getContext());
                mLabelsOverlay.setLoadingBackgroundColor(Color.TRANSPARENT);
                mLabelsOverlay.setLoadingLineColor(Color.TRANSPARENT);
                // Index 0 draws the labels just above the base tiles, underneath every mission
                // marker and path. MapEventsOverlay is also at 0 but paints nothing.
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

    /** Replaces the whole estimated track. Points are {@code {lat, lon}} pairs in degrees. */
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
}
