package se.bitcraze.crazyfliecontrol2.gpsnav;

import android.content.DialogInterface;
import android.content.res.ColorStateList;
import android.graphics.Typeface;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.view.View;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.content.ContextCompat;

import com.google.android.material.button.MaterialButton;

import com.espressif.espdrone.android.R;

import org.osmdroid.config.Configuration;
import org.osmdroid.util.GeoPoint;

import java.util.ArrayList;
import java.util.List;

import se.bitcraze.crazyflie.lib.crazyflie.Crazyflie;
import se.bitcraze.crazyflie.lib.crtp.FlightControlPacket;
import se.bitcraze.crazyfliecontrol2.DroneSession;
import se.bitcraze.crazyfliecontrol2.FlyHeader;

/**
 * Indoor Map screen (PROTOCOL.md section 10): a page of its own for the Valetudo map annotations,
 * laid out like piotrmachowski's Vacuum Map Card — title bar, map, mode chips, action icons,
 * object tiles, status. It edits the shared plan in {@link NavStore}, so the zones drawn here are
 * enforced by the Mission screen's runner.
 *
 * <p>Màn Indoor Map (PROTOCOL.md mục 10): trang riêng cho các chú thích bản đồ kiểu Valetudo,
 * bố trí theo Vacuum Map Card của piotrmachowski — thanh tiêu đề, bản đồ, chip chế độ, biểu tượng
 * hành động, ô đối tượng, trạng thái. Màn sửa kế hoạch dùng chung trong {@link NavStore}, nên các
 * vùng vẽ ở đây được trình chạy sứ mệnh của màn Mission thực thi.
 *
 * <p>The modes port the card's MapMode/SelectionType set exactly: Select is the default and owns
 * Valetudo's handle editing; Keep-out and Wind are MANUAL_RECTANGLE (drag a rectangle); Corridor
 * is MANUAL_PATH (tap two endpoints); Go to is MANUAL_POINT (tap the target). In the placement
 * modes every map touch is a placement gesture — no selection, no waypoint drops.
 *
 * <p>Các chế độ chuyển thể đúng bộ MapMode/SelectionType của thẻ: Select là mặc định và nắm việc
 * sửa bằng tay nắm của Valetudo; Keep-out và Wind là MANUAL_RECTANGLE (kéo hình chữ nhật);
 * Corridor là MANUAL_PATH (chạm hai đầu mút); Go to là MANUAL_POINT (chạm vào mục tiêu). Trong
 * các chế độ đặt hình, mọi thao tác chạm lên bản đồ đều là cử chỉ đặt — không chọn, không thả điểm.
 */
public class IndoorMapActivity extends AppCompatActivity {

    /** Tap tolerance for selecting a map annotation: fingers are imprecise. */
    /** Dung sai chạm để chọn một chú thích bản đồ: ngón tay không chính xác. */
    private static final double TAP_TOLERANCE_M = 10.0;

    private static final int MODE_SELECT = 0;
    private static final int MODE_KEEPOUT = 1;
    private static final int MODE_CORRIDOR = 2;
    private static final int MODE_WIND = 3;
    private static final int MODE_GOTO = 4;

    private final Navigator mNavigator = NavStore.navigator();

    private int mMode = MODE_SELECT;
    private int mSelectedAnnotation = -1;
    private int mSelectedWaypoint = -1;

    // Corridor (MANUAL_PATH) needs the first endpoint between taps.
    // Corridor (MANUAL_PATH) cần nhớ đầu mút đầu tiên giữa hai lần chạm.
    private double mCorridorStartLat = Double.NaN;
    private double mCorridorStartLon = Double.NaN;

    private MissionMapView mMapView;
    private TextView mStatusText;
    private TextView mTilesEmpty;
    private LinearLayout mTilesContainer;
    private MaterialButton mModeSelect;
    private MaterialButton mModeKeepout;
    private MaterialButton mModeCorridor;
    private MaterialButton mModeWind;
    private MaterialButton mModeGoto;
    private FlyHeader mHeader;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // osmdroid reads its configuration before inflating a MapView, and a missing user agent
        // gets whole apps blocked by the tile servers.
        // osmdroid đọc cấu hình trước khi nạp MapView, và thiếu user agent sẽ khiến các máy chủ
        // tile chặn cả ứng dụng.
        Configuration.getInstance()
                .load(this, PreferenceManager.getDefaultSharedPreferences(this));
        Configuration.getInstance().setUserAgentValue(getPackageName());
        setContentView(R.layout.activity_indoor_map);

        mMapView = findViewById(R.id.indoor_map);
        mStatusText = findViewById(R.id.indoor_status);
        mTilesEmpty = findViewById(R.id.indoor_tiles_empty);
        mTilesContainer = findViewById(R.id.indoor_tiles);
        mModeSelect = findViewById(R.id.indoor_mode_select);
        mModeKeepout = findViewById(R.id.indoor_mode_keepout);
        mModeCorridor = findViewById(R.id.indoor_mode_corridor);
        mModeWind = findViewById(R.id.indoor_mode_wind);
        mModeGoto = findViewById(R.id.indoor_mode_goto);

        // The Indoor Map owns annotation editing; the Mission screen shows the same zones read-only.
        // Indoor Map nắm việc sửa chú thích; màn Mission hiện các vùng đó ở chế độ chỉ xem.
        mMapView.setAnnotationsEditable(true);

        mMapView.setOnMapTapListener(new MissionMapView.OnMapTapListener() {
            @Override
            public void onMapTap(double latitude, double longitude) {
                IndoorMapActivity.this.onMapTap(latitude, longitude);
            }
        });

        // Valetudo's on-map handles: ✕ deletes the selected annotation, and any handle or body
        // drag has already mutated it in place and snapped on release (PROTOCOL.md section 10).
        // Các tay nắm trên bản đồ của Valetudo: ✕ xoá chú thích đang chọn, và mọi thao tác kéo
        // tay nắm hoặc thân hình đã sửa nó tại chỗ rồi hút lại khi nhả (PROTOCOL.md mục 10).
        mMapView.setOnAnnotationEditListener(new MissionMapView.OnAnnotationEditListener() {
            @Override
            public void onAnnotationDeleteRequested() {
                deleteSelected();
            }

            @Override
            public void onAnnotationEdited() {
                refreshUi();
                updateStatus();
            }
        });

        // MANUAL_RECTANGLE: the drag is done, build the zone from the finished corners.
        // MANUAL_RECTANGLE: thao tác kéo xong, dựng vùng từ các góc đã hoàn tất.
        mMapView.setOnPlacementListener(new MissionMapView.OnPlacementListener() {
            @Override
            public void onRectPlaced(double northLat, double westLon, double southLat, double eastLon) {
                MapAnnotation.Type type = mMode == MODE_WIND
                        ? MapAnnotation.Type.WIND
                        : MapAnnotation.Type.KEEPOUT;
                MapAnnotation annotation = MapAnnotation.rectFromCorners(
                        type, southLat, westLon, northLat, eastLon);
                annotation.snap();
                addAndSelect(annotation);
                toast(type.label + " placed - tap Select to adjust the handles");
                updateStatus();
            }
        });

        mModeSelect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                setMode(MODE_SELECT);
            }
        });
        mModeKeepout.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                setMode(MODE_KEEPOUT);
            }
        });
        mModeCorridor.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                setMode(MODE_CORRIDOR);
            }
        });
        mModeWind.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                setMode(MODE_WIND);
            }
        });
        mModeGoto.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                setMode(MODE_GOTO);
            }
        });

        findViewById(R.id.indoor_delete).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                deleteSelected();
            }
        });
        findViewById(R.id.indoor_clear).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                confirmClearAnnotations();
            }
        });
        findViewById(R.id.indoor_center).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onCenter();
            }
        });

        // Shared header: sections, status, CUT MOTORS (redesign stage 3).
        // Thanh tiêu đề dùng chung: các mục, trạng thái, CUT MOTORS (giai đoạn thiết kế lại 3).
        mHeader = new FlyHeader(this, findViewById(R.id.indoor_header), FlyHeader.INDOOR,
                new Runnable() {
                    @Override
                    public void run() {
                        onPanic();
                    }
                });

        // A target set on the Mission screen shows here too — one target, both screens.
        // Mục tiêu đặt ở màn Mission cũng hiện ở đây — một mục tiêu, cả hai màn.
        Waypoint guided = NavStore.getGuidedTarget();
        if (guided != null) {
            mMapView.setGuidedTarget(guided.getLatitude(), guided.getLongitude());
        }

        setMode(MODE_SELECT);
        refreshUi();
    }

    @Override
    protected void onResume() {
        super.onResume();
        mHeader.start();
        refreshUi();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mHeader.stop();
    }

    // region modes + map gestures

    private void setMode(int mode) {
        mMode = mode;
        mCorridorStartLat = Double.NaN;
        mCorridorStartLon = Double.NaN;
        // Rect modes rubber-band on drag; the rest use taps. Both suppress handle editing.
        // Chế độ hình chữ nhật phác bằng cách kéo; các chế độ còn lại dùng chạm. Cả hai đều
        // ngắt việc sửa bằng tay nắm.
        mMapView.setPlacementMode(mode == MODE_KEEPOUT || mode == MODE_WIND
                ? MissionMapView.PLACE_RECT
                : MissionMapView.PLACE_NONE);
        mMapView.setAnnotationsEditable(mode == MODE_SELECT);
        applyModeUi();
        updateStatus();
    }

    /** The active chip wears the segment row's selected look (chip active mang kiểu đang chọn). */
    private void applyModeUi() {
        MaterialButton[] chips = {mModeSelect, mModeKeepout, mModeCorridor, mModeWind, mModeGoto};
        for (int i = 0; i < chips.length; i++) {
            boolean active = i == mMode;
            chips[i].setBackgroundTintList(ColorStateList.valueOf(color(
                    active ? R.color.fly_surface : android.R.color.transparent)));
            chips[i].setTextColor(color(active ? R.color.fly_ink : R.color.fly_muted));
            chips[i].setIconTint(ColorStateList.valueOf(color(
                    active ? R.color.fly_ink : R.color.fly_muted)));
            chips[i].setTypeface(null, active ? Typeface.BOLD : Typeface.NORMAL);
            chips[i].setSelected(active);
        }
    }

    private void onMapTap(double latitude, double longitude) {
        if (mMode == MODE_KEEPOUT || mMode == MODE_WIND) {
            // Drag gestures only — a tap places nothing (chỉ có cử chỉ kéo — chạm không đặt gì).
            return;
        }
        if (mMode == MODE_CORRIDOR) {
            onCorridorTap(latitude, longitude);
            return;
        }
        if (mMode == MODE_GOTO) {
            // MANUAL_POINT: the tap IS the target (MANUAL_POINT: cú chạm chính là mục tiêu).
            NavStore.setGuidedTarget(new Waypoint(latitude, longitude));
            mMapView.setGuidedTarget(latitude, longitude);
            mSelectedAnnotation = -1;
            mSelectedWaypoint = -1;
            refreshUi();
            updateStatus();
            return;
        }
        // Select mode: a tap on a zone picks it (handles appear), any other tap clears the
        // selection. Waypoints are picked from the tiles row.
        // Chế độ Chọn: chạm vào vùng thì chọn nó (tay nắm hiện ra), chạm chỗ khác thì bỏ chọn.
        // Điểm hành trình được chọn từ hàng ô.
        synchronized (mNavigator) {
            List<MapAnnotation> annotations = mNavigator.getMission().getAnnotations();
            for (int i = 0; i < annotations.size(); i++) {
                if (annotations.get(i).nearPoint(latitude, longitude, TAP_TOLERANCE_M)) {
                    mSelectedAnnotation = i;
                    mSelectedWaypoint = -1;
                    refreshUi();
                    updateStatus();
                    return;
                }
            }
        }
        mSelectedAnnotation = -1;
        refreshUi();
        updateStatus();
    }

    /** MANUAL_PATH: the first tap holds an endpoint, the second completes the corridor. */
    /** MANUAL_PATH: lần chạm đầu giữ một đầu mút, lần thứ hai hoàn tất hành lang. */
    private void onCorridorTap(double latitude, double longitude) {
        if (Double.isNaN(mCorridorStartLat)) {
            mCorridorStartLat = latitude;
            mCorridorStartLon = longitude;
            updateStatus();
            return;
        }
        MapAnnotation annotation = MapAnnotation.line(MapAnnotation.Type.CORRIDOR,
                mCorridorStartLat, mCorridorStartLon, latitude, longitude);
        annotation.snap();
        addAndSelect(annotation);
        mCorridorStartLat = Double.NaN;
        mCorridorStartLon = Double.NaN;
        toast("Corridor placed - tap Select to adjust the handles");
        updateStatus();
    }

    // endregion

    // region selection, tiles and actions

    private void addAndSelect(MapAnnotation annotation) {
        synchronized (mNavigator) {
            Mission mission = mNavigator.getMission();
            mission.addAnnotation(annotation);
            mSelectedAnnotation = mission.getAnnotations().size() - 1;
        }
        mSelectedWaypoint = -1;
        refreshUi();
    }

    /** Delete the selected object: the tapped zone first, else the selected waypoint. */
    /** Xoá đối tượng đang chọn: vùng đã chạm trước, sau đó điểm hành trình đang chọn. */
    private void deleteSelected() {
        if (mSelectedAnnotation >= 0) {
            MapAnnotation removed;
            synchronized (mNavigator) {
                Mission mission = mNavigator.getMission();
                if (mSelectedAnnotation >= mission.getAnnotations().size()) {
                    mSelectedAnnotation = -1;
                    refreshUi();
                    return;
                }
                removed = mission.getAnnotation(mSelectedAnnotation);
                mission.removeAnnotation(mSelectedAnnotation);
            }
            mSelectedAnnotation = -1;
            refreshUi();
            updateStatus();
            toast((removed != null ? removed.getType().label : "Annotation") + " deleted");
            return;
        }
        if (mSelectedWaypoint >= 0) {
            synchronized (mNavigator) {
                mNavigator.getMission().removeWaypoint(mSelectedWaypoint);
            }
            mSelectedWaypoint = -1;
            refreshUi();
            updateStatus();
            toast("Waypoint deleted");
            return;
        }
        toast("Nothing selected - tap a zone or a tile first");
    }

    /**
     * Bulk delete asks first: a dialog item looks like every other entry and a mis-tap would drop
     * every zone at once (same confirmation as the old Mission-page clear).
     *
     * <p>Xoá hàng loạt cần xác nhận trước: mục hộp thoại trông giống mọi mục khác và một lần chạm
     * nhầm sẽ xoá mất mọi vùng cùng lúc (cùng xác nhận như nút xoá hết cũ ở trang Mission).
     */
    private void confirmClearAnnotations() {
        new AlertDialog.Builder(this)
                .setTitle("Clear all annotations?")
                .setMessage("All keep-out zones, corridors and wind zones will be removed.")
                .setPositiveButton("Clear", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        synchronized (mNavigator) {
                            mNavigator.getMission().clearAnnotations();
                        }
                        mSelectedAnnotation = -1;
                        refreshUi();
                        updateStatus();
                        toast("All annotations cleared");
                    }
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    /**
     * Center on the selected object, else the first plan object, else home. The screen has no
     * live drone feed — the Mission screen owns telemetry — so "drone" is not an option here.
     *
     * <p>Căn giữa theo đối tượng đang chọn, sau đó đối tượng đầu tiên của kế hoạch, rồi home. Màn
     * hình này không có luồng dữ liệu drone trực tiếp — màn Mission nắm telemetry — nên "drone"
     * không phải một lựa chọn ở đây.
     */
    private void onCenter() {
        double lat = Double.NaN;
        double lon = Double.NaN;
        synchronized (mNavigator) {
            Mission mission = mNavigator.getMission();
            List<MapAnnotation> annotations = mission.getAnnotations();
            List<Waypoint> waypoints = mission.getWaypoints();
            if (mSelectedAnnotation >= 0 && mSelectedAnnotation < annotations.size()) {
                double[] c = annotations.get(mSelectedAnnotation).center();
                lat = c[0];
                lon = c[1];
            } else if (mSelectedWaypoint >= 0 && mSelectedWaypoint < waypoints.size()) {
                lat = waypoints.get(mSelectedWaypoint).getLatitude();
                lon = waypoints.get(mSelectedWaypoint).getLongitude();
            } else if (!annotations.isEmpty()) {
                double[] c = annotations.get(0).center();
                lat = c[0];
                lon = c[1];
            } else if (!waypoints.isEmpty()) {
                lat = waypoints.get(0).getLatitude();
                lon = waypoints.get(0).getLongitude();
            } else if (mission.hasHome()) {
                lat = mission.getHomeLatitude();
                lon = mission.getHomeLongitude();
            }
        }
        if (Double.isNaN(lat)) {
            toast("Nothing to center on yet");
            return;
        }
        mMapView.getController().setCenter(new GeoPoint(lat, lon));
    }

    /** CUT MOTORS, same one-tap path as every screen: emergency, then stop the navigator. */
    /** CUT MOTORS, cùng đường một chạm như mọi màn hình: emergency, rồi dừng navigator. */
    private void onPanic() {
        Crazyflie crazyflie = DroneSession.get();
        if (crazyflie != null) {
            crazyflie.sendPacket(FlightControlPacket.emergency());
        }
        synchronized (mNavigator) {
            mNavigator.stop();
        }
        toast("CUT MOTORS - motors off");
        refreshUi();
        updateStatus();
    }

    // endregion

    // region UI updates

    private void refreshUi() {
        synchronized (mNavigator) {
            Mission mission = mNavigator.getMission();
            int activeIndex = mNavigator.getMode() == Navigator.NavMode.MISSION
                    ? mNavigator.getActiveIndex()
                    : -1;
            mMapView.setWaypoints(new ArrayList<Waypoint>(mission.getWaypoints()), activeIndex);
            mMapView.setAnnotations(
                    new ArrayList<MapAnnotation>(mission.getAnnotations()), mSelectedAnnotation);
            if (mission.hasHome()) {
                mMapView.setHome(mission.getHomeLatitude(), mission.getHomeLongitude());
            }
            refreshTiles(mission);
        }
    }

    /**
     * The tiles row: one tile per object ("Keep-out 1", "Corridor 1", "Wind 1" per type ordinal,
     * then "WP 1"…), tap = select + center — the Vacuum Map Card's predefined-selection tiles.
     *
     * <p>Hàng ô: mỗi đối tượng một ô ("Keep-out 1", "Corridor 1", "Wind 1" theo thứ tự trong
     * từng kiểu, rồi "WP 1"…), chạm = chọn + căn giữa — các ô predefined-selection của Vacuum
     * Map Card.
     */
    private void refreshTiles(Mission mission) {
        mTilesContainer.removeAllViews();
        int keepout = 0;
        int corridor = 0;
        int wind = 0;

        List<MapAnnotation> annotations = mission.getAnnotations();
        for (int i = 0; i < annotations.size(); i++) {
            MapAnnotation.Type type = annotations.get(i).getType();
            int ordinal;
            String shortName;
            switch (type) {
                case KEEPOUT:
                    ordinal = ++keepout;
                    shortName = "Keep-out";
                    break;
                case CORRIDOR:
                    ordinal = ++corridor;
                    shortName = "Corridor";
                    break;
                case WIND:
                default:
                    ordinal = ++wind;
                    shortName = "Wind";
                    break;
            }
            addTile(iconForType(type), shortName + " " + ordinal, i == mSelectedAnnotation, i, -1);
        }

        List<Waypoint> waypoints = mission.getWaypoints();
        for (int i = 0; i < waypoints.size(); i++) {
            addTile(R.drawable.ic_btn_goto, "WP " + (i + 1), i == mSelectedWaypoint, -1, i);
        }

        mTilesEmpty.setVisibility(annotations.isEmpty() && waypoints.isEmpty()
                ? View.VISIBLE
                : View.GONE);
    }

    private void addTile(int iconRes, String label, boolean selected,
            final int annotationIndex, final int waypointIndex) {
        View tile = getLayoutInflater().inflate(R.layout.indoor_tile, mTilesContainer, false);
        tile.setBackgroundResource(selected
                ? R.drawable.bg_indoor_tile_selected
                : R.drawable.bg_indoor_tile);
        ((ImageView) tile.findViewById(R.id.indoor_tile_icon)).setImageResource(iconRes);
        ((TextView) tile.findViewById(R.id.indoor_tile_label)).setText(label);
        tile.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mSelectedAnnotation = annotationIndex;
                mSelectedWaypoint = waypointIndex;
                refreshUi();
                updateStatus();
                onCenter();
            }
        });
        mTilesContainer.addView(tile);
    }

    /** Valetudo's custom glyphs per annotation type, as on the Mission screen's old Add menu. */
    /** Glyph riêng của Valetudo cho từng kiểu chú thích, như menu Thêm cũ của màn Mission. */
    private static int iconForType(MapAnnotation.Type type) {
        switch (type) {
            case KEEPOUT:
                return R.drawable.ic_annotation_keepout;
            case CORRIDOR:
                return R.drawable.ic_annotation_corridor;
            case WIND:
                return R.drawable.ic_annotation_wind;
            default:
                return R.drawable.ic_annotation_select;
        }
    }

    /** One status line that names the current mode and what its gestures do. */
    /** Một dòng trạng thái nêu chế độ hiện tại và các cử chỉ của nó làm gì. */
    private void updateStatus() {
        StringBuilder line = new StringBuilder();
        switch (mMode) {
            case MODE_KEEPOUT:
                line.append("Keep-out zone - drag a rectangle on the map");
                break;
            case MODE_WIND:
                line.append("Wind zone - drag a rectangle on the map");
                break;
            case MODE_CORRIDOR:
                line.append("Corridor - tap the ")
                        .append(Double.isNaN(mCorridorStartLat) ? "first" : "second")
                        .append(" endpoint");
                break;
            case MODE_GOTO:
                line.append(NavStore.getGuidedTarget() != null
                        ? "Go to target set - start GUIDED from the Mission page."
                        : "Go to - tap the map to set the target");
                break;
            case MODE_SELECT:
            default:
                if (mSelectedAnnotation >= 0) {
                    MapAnnotation selected;
                    synchronized (mNavigator) {
                        selected = mNavigator.getMission().getAnnotation(mSelectedAnnotation);
                    }
                    line.append(selected != null ? selected.getType().label : "Annotation")
                            .append(" selected - drag the handles, or use Delete");
                } else if (mSelectedWaypoint >= 0) {
                    line.append("WP ").append(mSelectedWaypoint + 1)
                            .append(" selected - use Delete to remove it");
                } else {
                    line.append("Select mode - tap a zone to edit it with handles");
                }
                break;
        }
        mStatusText.setText(line.toString());
    }

    private int color(int res) {
        return ContextCompat.getColor(this, res);
    }

    private void toast(String message) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show();
    }

    // endregion
}
