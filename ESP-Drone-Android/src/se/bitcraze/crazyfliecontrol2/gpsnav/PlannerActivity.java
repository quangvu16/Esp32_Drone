package se.bitcraze.crazyfliecontrol2.gpsnav;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationManager;
import android.net.Uri;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.BaseAdapter;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.espressif.espdrone.android.R;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import se.bitcraze.crazyflie.lib.crazyflie.Crazyflie;
import se.bitcraze.crazyflie.lib.crtp.FlightControlPacket;
import se.bitcraze.crazyflie.lib.crtp.NavPacket;
import se.bitcraze.crazyfliecontrol2.DroneSession;
import se.bitcraze.crazyfliecontrol2.FlyHeader;

/**
 * The Plan screen — a phone port of Mission Planner's Flight Plan screen: the
 * mission-item grid (# / Command / P1-P4 / Lat / Lng / Alt) over a map, file
 * load/save in the QGC WPL 110 format and upload/download/start against the
 * drone's onboard mission engine (PROTOCOL.md §12). The tab order mirrors
 * Mission Planner's own: Fly (Flight Data) → Plan (Flight Plan).
 * <p>
 * Màn Plan — bản chuyển thể điện thoại của màn Flight Plan trong Mission
 * Planner: lưới mục nhiệm vụ (# / Lệnh / P1-P4 / Vĩ độ / Kinh độ / Độ cao) trên
 * bản đồ, nạp/lưu tệp theo định dạng QGC WPL 110 và tải lên/tải về/bay với
 * động cơ nhiệm vụ trên bo của drone (PROTOCOL.md mục 12). Thứ tự tab lặp đúng
 * Mission Planner: Fly (Flight Data) → Plan (Flight Plan).
 *
 * <p>Mission files are edited with the exact Mission Planner semantics (see
 * {@link PlannerMission}); the transfer runs the MISSION_* handshake of §12
 * over CRTP port 7 channel 2 with a 500 ms resend. Tapping the map adds a
 * waypoint — the acceptance radius starts at the firmware's 2.5 m default.
 * Tệp nhiệm vụ được sửa với đúng ngữ nghĩa Mission Planner (xem
 * {@link PlannerMission}); việc truyền chạy bắt tay MISSION_* của mục 12 trên
 * CRTP port 7 kênh 2 với chu kỳ gửi lại 500 ms. Chạm bản đồ thêm một waypoint —
 * bán kính chấp nhận bắt đầu ở mức mặc định 2.5 m của firmware.
 */
public class PlannerActivity extends AppCompatActivity implements MissionTransfer.Listener {

    private static final int REQUEST_OPEN_WAYPOINTS = 1;
    private static final int REQUEST_SAVE_WAYPOINTS = 2;
    private static final int REQUEST_LOCATION_PERMISSION = 3;

    /** The firmware's default acceptance radius for NAV_WAYPOINT (§12). */
    /** Bán kính chấp nhận mặc định của firmware cho NAV_WAYPOINT (mục 12). */
    private static final float DEFAULT_WAYPOINT_RADIUS = 2.5f;

    private FlyHeader mHeader;
    private TextView mStatusText;
    private MissionMapView mMapView;
    private ListView mGrid;
    private View mEditStrip;
    private Spinner mCommandSpinner;
    private EditText mEditP1;
    private EditText mEditP2;
    private EditText mEditP3;
    private EditText mEditP4;
    private EditText mEditLat;
    private EditText mEditLon;
    private EditText mEditAlt;

    private final PlannerMission mMission = new PlannerMission();
    private GridAdapter mAdapter;
    private int mSelected = -1;
    private boolean mFilling;

    /** Map markers skip rows without a position (DO_JUMP and friends). */
    /** Marker bản đồ bỏ qua các dòng không có vị trí (DO_JUMP và đồng loại). */
    private final List<Integer> mMapIndices = new ArrayList<>();

    private float mDefaultAlt = 3.0f;
    private MissionTransfer mTransfer;
    private LocationManager mLocationManager;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // osmdroid reads its configuration before inflating a MapView, and a
        // missing user agent gets whole apps blocked by the tile servers.
        // osmdroid đọc cấu hình trước khi nạp MapView, và thiếu user agent sẽ
        // khiến các máy chủ tile chặn cả ứng dụng.
        org.osmdroid.config.Configuration.getInstance()
                .load(this, PreferenceManager.getDefaultSharedPreferences(this));
        org.osmdroid.config.Configuration.getInstance().setUserAgentValue(getPackageName());
        setContentView(R.layout.activity_planner);

        mStatusText = findViewById(R.id.planner_status);
        mMapView = findViewById(R.id.planner_map);
        mGrid = findViewById(R.id.planner_grid);
        mEditStrip = findViewById(R.id.planner_edit);
        mCommandSpinner = findViewById(R.id.planner_cmd);
        mEditP1 = findViewById(R.id.planner_p1);
        mEditP2 = findViewById(R.id.planner_p2);
        mEditP3 = findViewById(R.id.planner_p3);
        mEditP4 = findViewById(R.id.planner_p4);
        mEditLat = findViewById(R.id.planner_lat);
        mEditLon = findViewById(R.id.planner_lon);
        mEditAlt = findViewById(R.id.planner_alt);

        mAdapter = new GridAdapter();
        mGrid.setAdapter(mAdapter);
        mGrid.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                commitRow();
                selectRow(position);
            }
        });

        // Tapping the map adds a waypoint there — Mission Planner's map click.
        // Chạm bản đồ thêm một waypoint tại đó — map click của Mission Planner.
        mMapView.setOnMapTapListener(new MissionMapView.OnMapTapListener() {
            @Override
            public void onMapTap(double latitude, double longitude) {
                commitRow();
                // Keep-out zones veto the placement outright (PROTOCOL.md §10).
                // Vùng cấm bay phủ quyết việc đặt điểm ngay (PROTOCOL.md mục 10).
                String refusal = NavStore.navigator().checkWaypoint(latitude, longitude);
                if (refusal != null) {
                    toast("Cannot place waypoint: " + refusal);
                    return;
                }
                PlannerItem item = PlannerItem.waypoint(latitude, longitude, mDefaultAlt);
                item.p2 = DEFAULT_WAYPOINT_RADIUS;
                mMission.add(item);
                selectRow(mMission.size() - 1);
                refreshUi();
            }
        });

        // The row editor commits when a field loses focus or the command is
        // picked — the grid cells of Mission Planner commit on cell-exit.
        // Trình sửa dòng ghi nhận khi trường mất focus hoặc lệnh được chọn —
        // các ô lưới của Mission Planner ghi nhận khi rời ô.
        View.OnFocusChangeListener commit = new View.OnFocusChangeListener() {
            @Override
            public void onFocusChange(View v, boolean hasFocus) {
                if (!hasFocus) {
                    commitRow();
                }
            }
        };
        mEditP1.setOnFocusChangeListener(commit);
        mEditP2.setOnFocusChangeListener(commit);
        mEditP3.setOnFocusChangeListener(commit);
        mEditP4.setOnFocusChangeListener(commit);
        mEditLat.setOnFocusChangeListener(commit);
        mEditLon.setOnFocusChangeListener(commit);

        // Mission Planner's command column is a dropdown of the MAV_CMD names.
        // Cột lệnh của Mission Planner là một dropdown các tên MAV_CMD.
        mCommandSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (mFilling || mSelected < 0) {
                    return;
                }
                String picked = (String) parent.getItemAtPosition(position);
                int command = PlannerItem.parseCommand(picked);
                if (command >= 0) {
                    mMission.items().get(mSelected).command = command;
                    refreshUi();
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });

        findViewById(R.id.planner_add).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onAddWaypoint();
            }
        });
        findViewById(R.id.planner_wp_up).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onMoveWaypoint(-1);
            }
        });
        findViewById(R.id.planner_wp_down).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onMoveWaypoint(1);
            }
        });
        findViewById(R.id.planner_wp_remove).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onRemoveWaypoint();
            }
        });
        findViewById(R.id.planner_load).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onLoadMission();
            }
        });
        findViewById(R.id.planner_save).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onSaveMission();
            }
        });
        findViewById(R.id.planner_alt_set).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onSetAlt();
            }
        });
        findViewById(R.id.planner_upload).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onUpload();
            }
        });
        findViewById(R.id.planner_download).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onDownload();
            }
        });
        findViewById(R.id.planner_start).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onStartMission();
            }
        });

        // Shared header: sections, status, CUT MOTORS (redesign stage 3).
        // Thanh tiêu đề dùng chung: các mục, trạng thái, CUT MOTORS (giai đoạn
        // thiết kế lại 3).
        mHeader = new FlyHeader(this, findViewById(R.id.planner_header), FlyHeader.PLAN,
                new Runnable() {
                    @Override
                    public void run() {
                        onPanic();
                    }
                });
        mHeader.setLeaveGuard(new FlyHeader.LeaveGuard() {
            @Override
            public String blockReason() {
                // A transfer in flight dies with this screen.
                // Phiên truyền đang chạy sẽ chết theo màn hình này.
                return mTransfer != null && mTransfer.isBusy()
                        ? "Mission transfer is running - wait for it to finish"
                        : null;
            }
        });

        mTransfer = new MissionTransfer(this);
        mLocationManager = (LocationManager) getSystemService(LOCATION_SERVICE);

        requestLocationPermission();
        refreshUi();
    }

    @Override
    protected void onResume() {
        super.onResume();
        mHeader.start();
        mTransfer.attach(DroneSession.get());
    }

    @Override
    protected void onPause() {
        super.onPause();
        mHeader.stop();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        mTransfer.detach();
    }

    // ---- mission editing --------------------------------------------------

    private void selectRow(int position) {
        mSelected = position;
        refreshUi();
    }

    private void onAddWaypoint() {
        commitRow();
        PlannerItem item = PlannerItem.waypoint(0, 0, mDefaultAlt);
        item.p2 = DEFAULT_WAYPOINT_RADIUS;
        // Without a map position yet, place it on the last one so the marker
        // stack stays visible; the Lat/Lng cells move it where it belongs.
        // Chưa có vị trí trên bản đồ thì đặt lên vị trí cuối để các marker còn
        // nhìn thấy; ô Vĩ độ/Kinh độ sẽ đưa nó về đúng chỗ.
        if (mMission.size() > 0) {
            PlannerItem last = mMission.items().get(mMission.size() - 1);
            item.lat = last.lat;
            item.lon = last.lon;
        }
        mMission.add(item);
        selectRow(mMission.size() - 1);
        refreshUi();
    }

    private void onMoveWaypoint(int delta) {
        commitRow();
        if (mSelected < 0) {
            toast("Select a mission item first");
            return;
        }
        int to = mSelected + delta;
        if (to < 0 || to >= mMission.size()) {
            return;
        }
        List<PlannerItem> items = mMission.items();
        PlannerItem moved = items.remove(mSelected);
        items.add(to, moved);
        mSelected = to;
        refreshUi();
    }

    private void onRemoveWaypoint() {
        commitRow();
        if (mSelected < 0) {
            toast("Select a mission item first");
            return;
        }
        mMission.items().remove(mSelected);
        if (mSelected >= mMission.size()) {
            mSelected = mMission.size() - 1;
        }
        refreshUi();
    }

    /** Apply the altitude box to the selected row; else it is the new-WP alt. */
    /** Ghi ô độ cao vào dòng đang chọn; nếu không thì đó là độ cao cho WP mới. */
    private void onSetAlt() {
        float alt = parseFloat(mEditAlt, mDefaultAlt);
        mDefaultAlt = alt;
        if (mSelected >= 0) {
            commitRow();
            mMission.items().get(mSelected).alt = alt;
            refreshUi();
            toast("Altitude set");
        } else {
            toast("New waypoints will use alt " + alt);
        }
    }

    /** Write the edit-strip fields into the selected row. */
    /** Ghi các trường trong dải sửa vào dòng đang chọn. */
    private void commitRow() {
        if (mFilling || mSelected < 0 || mSelected >= mMission.size()) {
            return;
        }
        PlannerItem item = mMission.items().get(mSelected);
        item.p1 = parseFloat(mEditP1, item.p1);
        item.p2 = parseFloat(mEditP2, item.p2);
        item.p3 = parseFloat(mEditP3, item.p3);
        item.p4 = parseFloat(mEditP4, item.p4);
        item.lat = parseDouble(mEditLat, item.lat);
        item.lon = parseDouble(mEditLon, item.lon);
    }

    private float parseFloat(EditText field, float fallback) {
        try {
            return Float.parseFloat(field.getText().toString().trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private double parseDouble(EditText field, double fallback) {
        try {
            return Double.parseDouble(field.getText().toString().trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    // ---- files ------------------------------------------------------------

    private void onLoadMission() {
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType("*/*"); // .waypoints has no registered MIME type
        // (.waypoints không có MIME type đã đăng ký)
        startActivityForResult(intent, REQUEST_OPEN_WAYPOINTS);
    }

    private void onSaveMission() {
        Intent intent = new Intent(Intent.ACTION_CREATE_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType("text/plain");
        intent.putExtra(Intent.EXTRA_TITLE, "mission.waypoints");
        startActivityForResult(intent, REQUEST_SAVE_WAYPOINTS);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (resultCode != RESULT_OK || data == null || data.getData() == null) {
            return;
        }
        Uri uri = data.getData();
        if (requestCode == REQUEST_OPEN_WAYPOINTS) {
            try {
                PlannerMission loaded = PlannerMission.parse(readAll(uri));
                replaceMission(loaded);
                if (!loaded.invalidLines().isEmpty()) {
                    toast("Mission loaded - " + loaded.invalidLines().size() + " invalid line(s) skipped");
                } else {
                    toast("Mission loaded");
                }
            } catch (IOException e) {
                toast("Load failed: " + e.getMessage());
            }
        } else if (requestCode == REQUEST_SAVE_WAYPOINTS) {
            commitRow();
            try {
                writeAll(uri, mMission.format());
                toast("Mission saved");
            } catch (IOException e) {
                toast("Save failed: " + e.getMessage());
            }
        }
    }

    /** Swap in a freshly loaded/downloaded mission (its home comes with it). */
    /** Thay bằng nhiệm vụ vừa nạp/tải về (home của nó đi kèm). */
    private void replaceMission(PlannerMission loaded) {
        mMission.clear();
        mMission.home().frame = loaded.home().frame;
        mMission.home().command = loaded.home().command;
        mMission.home().p1 = loaded.home().p1;
        mMission.home().p2 = loaded.home().p2;
        mMission.home().p3 = loaded.home().p3;
        mMission.home().p4 = loaded.home().p4;
        mMission.home().lat = loaded.home().lat;
        mMission.home().lon = loaded.home().lon;
        mMission.home().alt = loaded.home().alt;
        mMission.items().addAll(loaded.items());
        mSelected = mMission.size() == 0 ? -1 : 0;
        refreshUi();
    }

    private String readAll(Uri uri) throws IOException {
        InputStream in = getContentResolver().openInputStream(uri);
        if (in == null) {
            throw new IOException("cannot open " + uri);
        }
        try {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buffer = new byte[4096];
            int read;
            while ((read = in.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
            return new String(out.toByteArray(), StandardCharsets.UTF_8);
        } finally {
            in.close();
        }
    }

    private void writeAll(Uri uri, String contents) throws IOException {
        OutputStream out = getContentResolver().openOutputStream(uri);
        if (out == null) {
            throw new IOException("cannot open " + uri);
        }
        try {
            out.write(contents.getBytes(StandardCharsets.UTF_8));
        } finally {
            out.close();
        }
    }

    // ---- transfer (PROTOCOL.md §12) ---------------------------------------

    private void onUpload() {
        commitRow();
        ensureOrigin();
        mTransfer.attach(DroneSession.get());
        mTransfer.upload(mMission.items());
    }

    private void onDownload() {
        commitRow();
        mTransfer.attach(DroneSession.get());
        mTransfer.download();
    }

    private void onStartMission() {
        commitRow();
        ensureOrigin();
        mTransfer.attach(DroneSession.get());
        mTransfer.startMission();
    }

    /**
     * The mission items are WGS-84 degrees and the drone flies a dead-reckoned
     * local frame — the origin is the only bridge (§12). Send the mission's
     * home, or fall back to the phone's fix when the plan has no home yet.
     * <p>
     * Các mục nhiệm vụ là độ WGS-84 còn drone bay theo khung cục bộ tự dẫn
     * đường — điểm gốc là cây cầu duy nhất (mục 12). Gửi home của nhiệm vụ, hoặc
     * dùng vị trí GPS của điện thoại khi kế hoạch chưa có home.
     */
    private void ensureOrigin() {
        PlannerItem home = mMission.home();
        if (home.lat == 0 && home.lon == 0) {
            Location fix = lastKnownLocation();
            if (fix != null) {
                home.lat = fix.getLatitude();
                home.lon = fix.getLongitude();
            }
        }
        Crazyflie link = DroneSession.get();
        if (link != null && (home.lat != 0 || home.lon != 0)) {
            link.sendPacket(NavPacket.origin(home.lat, home.lon));
        } else {
            toast("No home set - the drone will use its stored origin");
        }
    }

    private Location lastKnownLocation() {
        if (mLocationManager == null
                || ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            return null;
        }
        try {
            Location location = mLocationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
            if (location == null) {
                location = mLocationManager.getLastKnownLocation(LocationManager.NETWORK_PROVIDER);
            }
            return location;
        } catch (IllegalArgumentException e) {
            return null;
        }
    }

    @Override
    public void onTransferStatus(String message) {
        mStatusText.setText(message);
    }

    @Override
    public void onUploadFinished(int result) {
        toast(result == MissionPacket.MISSION_ACCEPTED
                ? "Mission uploaded" : "Upload failed: " + missionResultName(result));
        refreshUi();
    }

    @Override
    public void onDownloadFinished(PlannerMission mission, int result) {
        if (mission == null || result != MissionPacket.MISSION_ACCEPTED) {
            toast("Download failed: " + missionResultName(result));
            return;
        }
        replaceMission(mission);
        toast("Mission downloaded (" + mission.size() + " items)");
    }

    @Override
    public void onCommandFinished(int cmd, boolean ok) {
        if (cmd == MissionPacket.CMD_START) {
            toast(ok ? "Mission started" : "Start refused");
        } else if (cmd == MissionPacket.CMD_SET_CURRENT) {
            toast(ok ? "Current item set" : "Bad item number");
        }
    }

    @Override
    public void onCurrentItem(int seq) {
        mStatusText.setText(String.format(Locale.US, "flying item %d of %d", seq + 1, mMission.size()));
    }

    private static String missionResultName(int result) {
        switch (result) {
            case MissionPacket.MISSION_ACCEPTED:
                return "accepted";
            case MissionPacket.MISSION_NO_SPACE:
                return "no space on the drone";
            case MissionPacket.MISSION_INVALID:
                return "invalid mission";
            case MissionPacket.MISSION_DENIED:
                return "denied";
            default:
                return "error " + result;
        }
    }

    private void onPanic() {
        Crazyflie crazyflie = DroneSession.get();
        if (crazyflie != null) {
            crazyflie.sendPacket(FlightControlPacket.emergency());
        }
        toast("CUT MOTORS - motors off");
    }

    // ---- UI ---------------------------------------------------------------

    private void refreshUi() {
        mAdapter.notifyDataSetChanged();

        // Map markers: rows that carry a position, in mission order.
        // Marker bản đồ: các dòng có vị trí, theo thứ tự nhiệm vụ.
        List<Waypoint> waypoints = new ArrayList<>();
        mMapIndices.clear();
        List<PlannerItem> items = mMission.items();
        for (int i = 0; i < items.size(); i++) {
            PlannerItem item = items.get(i);
            if (item.lat != 0 || item.lon != 0) {
                mMapIndices.add(i);
                waypoints.add(new Waypoint(item.lat, item.lon));
            }
        }
        int active = -1;
        for (int i = 0; i < mMapIndices.size(); i++) {
            if (mMapIndices.get(i) == mSelected) {
                active = i;
                break;
            }
        }
        mMapView.setWaypoints(waypoints, active);
        PlannerItem home = mMission.home();
        mStatusText.setText(String.format(Locale.US, "%d items · home %.6f, %.6f",
                items.size(), home.lat, home.lon));

        fillEditStrip();
    }

    private void fillEditStrip() {
        mFilling = true;
        try {
            if (mSelected < 0 || mSelected >= mMission.size()) {
                mEditStrip.setVisibility(View.GONE);
                return;
            }
            mEditStrip.setVisibility(View.VISIBLE);
            PlannerItem item = mMission.items().get(mSelected);
            mEditP1.setText(Float.toString(item.p1));
            mEditP2.setText(Float.toString(item.p2));
            mEditP3.setText(Float.toString(item.p3));
            mEditP4.setText(Float.toString(item.p4));
            mEditLat.setText(String.format(Locale.US, "%.7f", item.lat));
            mEditLon.setText(String.format(Locale.US, "%.7f", item.lon));

            // The dropdown lists the commands this firmware flies plus the
            // row's own (or "UNKNOWN", as Mission Planner labels the rest).
            // Dropdown liệt kê các lệnh firmware bay được cộng với lệnh của dòng
            // đó (hoặc "UNKNOWN", như Mission Planner đặt tên cho phần còn lại).
            List<String> names = new ArrayList<>();
            names.add("WAYPOINT");
            names.add("TAKEOFF");
            names.add("LAND");
            names.add("RETURN_TO_LAUNCH");
            names.add("DO_JUMP");
            String current = item.commandName();
            if (!names.contains(current)) {
                names.add(current);
            }
            if (!names.contains("UNKNOWN")) {
                names.add("UNKNOWN");
            }
            ArrayAdapter<String> adapter = new ArrayAdapter<>(this,
                    android.R.layout.simple_spinner_item, names);
            adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            mCommandSpinner.setAdapter(adapter);
            mCommandSpinner.setSelection(Math.max(names.indexOf(current), 0));
        } finally {
            mFilling = false;
        }
    }

    /** The grid row: # / Command / Alt, then P1-P4, then the position. */
    /** Dòng lưới: # / Lệnh / Độ cao, rồi P1-P4, rồi vị trí. */
    private final class GridAdapter extends BaseAdapter {
        @Override
        public int getCount() {
            return mMission.size();
        }

        @Override
        public Object getItem(int position) {
            return mMission.items().get(position);
        }

        @Override
        public long getItemId(int position) {
            return position;
        }

        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            View row = convertView;
            if (row == null) {
                row = getLayoutInflater().inflate(R.layout.planner_item_row, parent, false);
            }
            PlannerItem item = mMission.items().get(position);
            ((TextView) row.findViewById(R.id.planner_row_seq))
                    .setText("#" + (position + 1));
            ((TextView) row.findViewById(R.id.planner_row_cmd))
                    .setText(item.commandName());
            ((TextView) row.findViewById(R.id.planner_row_alt))
                    .setText("alt " + Float.toString(item.alt));
            ((TextView) row.findViewById(R.id.planner_row_params)).setText(String.format(Locale.US,
                    "P1 %s  P2 %s  P3 %s  P4 %s",
                    Float.toString(item.p1), Float.toString(item.p2),
                    Float.toString(item.p3), Float.toString(item.p4)));
            ((TextView) row.findViewById(R.id.planner_row_pos)).setText(String.format(Locale.US,
                    "%.7f, %.7f", item.lat, item.lon));
            return row;
        }
    }

    // ---- phone location permission ---------------------------------------

    private void requestLocationPermission() {
        // Android 12+ requires FINE and COARSE to be requested together (and both declared).
        // Android 12+ yêu cầu xin FINE và COARSE cùng nhau (và khai báo cả hai).
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                    new String[] {
                            Manifest.permission.ACCESS_FINE_LOCATION,
                            Manifest.permission.ACCESS_COARSE_LOCATION },
                    REQUEST_LOCATION_PERMISSION);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions,
            int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_LOCATION_PERMISSION
                && (grantResults.length == 0 || grantResults[0] != PackageManager.PERMISSION_GRANTED)) {
            toast("Location permission denied - home falls back to the file/drone origin");
        }
    }

    private void toast(String message) {
        Toast.makeText(this, message, Toast.LENGTH_LONG).show();
    }
}
