package se.bitcraze.crazyfliecontrol2.gpsnav;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.android.material.button.MaterialButton;

import com.espressif.espdrone.android.R;

import org.osmdroid.config.Configuration;
import org.osmdroid.util.GeoPoint;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import se.bitcraze.crazyflie.lib.crazyflie.Crazyflie;
import se.bitcraze.crazyflie.lib.crtp.FlightControlPacket;
import se.bitcraze.crazyflie.lib.crtp.NavPacket;
import se.bitcraze.crazyflie.lib.log.LogAdapter;
import se.bitcraze.crazyflie.lib.log.LogConfig;
import se.bitcraze.crazyflie.lib.log.Logg;
import se.bitcraze.crazyfliecontrol2.DroneSession;
import se.bitcraze.crazyfliecontrol2.FlyHeader;

/**
 * GPS mission navigation screen (PROTOCOL.md section 10).
 *
 * <p>Màn hình điều hướng sứ mệnh GPS (PROTOCOL.md mục 10).
 *
 * <p>The split of labour is the whole design: the drone has no GPS and no compass, so the phone
 * plans the mission and owns the launch frame, while the drone chases one north/east offset at a
 * time. Progress (arrival checks) comes from the drone's dead-reckoned {@code nav.pos_x}/{@code
 * nav.pos_y} — never from the phone's GPS, or a pilot walking away would "arrive" the drone at
 * waypoints it never reached. The one asymmetry: in FOLLOW the target may be the phone itself, so
 * the phone is the target source and the drone's estimate remains the progress source.
 *
 * <p>Chia việc chính là toàn bộ thiết kế: drone không có GPS cũng không có la bàn, nên điện thoại
 * lập kế hoạch sứ mệnh và nắm giữ khung cất cánh, còn drone lần theo từng độ lệch
 * bắc/đông một. Tiến độ (kiểm tra đến nơi) lấy từ {@code nav.pos_x}/{@code nav.pos_y} tự dẫn
 * đường quán tính của drone — không bao giờ từ GPS của điện thoại, nếu không người điều khiển
 * đi khỏi chỗ là drone "đến nơi" ở những điểm hành trình nó chưa hề chạm tới. Điểm bất đối xứng
 * duy nhất: trong FOLLOW, mục tiêu có thể là chính điện thoại, nên điện thoại là nguồn mục tiêu
 * và ước tính của drone vẫn là nguồn đo tiến độ.
 *
 * <p>Everything drawn for the drone is labelled "estimated": it is accelerometer dead reckoning
 * and drifts metres per minute. The firmware's {@code NAV_FWD_SIGN}/{@code NAV_LEFT_SIGN} are
 * unverified on real hardware (PROTOCOL.md section 6) and one may need flipping after the first
 * flight — nothing here assumes the dead reckoning is accurate.
 *
 * <p>Mọi thứ vẽ cho drone đều được gắn nhãn "estimated" (ước tính): đó là tự dẫn đường quán tính
 * từ gia tốc kế và trôi vài mét mỗi phút. {@code NAV_FWD_SIGN}/{@code NAV_LEFT_SIGN} trong firmware
 * chưa được kiểm chứng trên phần cứng thật (PROTOCOL.md mục 6) và có thể phải đảo dấu một cái sau
 * chuyến bay đầu tiên — không có gì ở đây giả định tự dẫn đường quán tính là chính xác.
 */
public class GpsNavActivity extends AppCompatActivity
        implements LocationListener, SensorEventListener {

    private static final String LOG_TAG = "GpsNavActivity";

    private static final int REQUEST_LOCATION_PERMISSION = 43;
    private static final int REQUEST_OPEN_WAYPOINTS = 44;
    private static final int REQUEST_SAVE_WAYPOINTS = 45;

    /** A TARGET is honoured for only 3 s after the last packet (PROTOCOL.md section 6). */
    /** Một TARGET chỉ có hiệu lực trong 3 giây sau gói tin cuối cùng (PROTOCOL.md mục 6). */
    private static final long TARGET_INTERVAL_MS = 200;

    /** Nav log block period: 20 Hz of pos/tgt/heading is plenty and fits the 25 byte budget. */
    /** Chu kỳ khối log nav: 20 Hz cho pos/tgt/heading là đủ và vừa ngân sách 25 byte. */
    private static final int LOG_PERIOD_MS = 50;

    private static final long LOG_RETRY_MS = 500;

    /** Mid-range hover thrust (valid range 8000..55000) until read from `flight.hover_thrust`. */
    /** Lực đẩy giữ độ cao mức trung (hợp lệ 8000..55000) cho tới khi đọc được `flight.hover_thrust`. */
    private static final int DEFAULT_HOVER_THRUST = 30000;

    /** Don't breadcrumb every 50 ms fix; one metre of movement is enough resolution. */
    /** Không ghi dấu bánh mì cho mọi bản vị 50 ms; dịch chuyển một mét là đủ độ phân giải. */
    private static final double TRACK_MIN_STEP_M = 1.0;

    /** Network fixes are only trusted while no fresh GPS fix exists. */
    /** Chỉ tin bản vị từ network khi chưa có bản vị GPS mới. */
    private static final long GPS_FIX_MAX_AGE_MS = 10000;

    // Drone link
    private Crazyflie mCrazyflie;
    private LogConfig mNavLog;
    private boolean mNavLogStarted;

    // Phone sensors: GPS defines the plan and launch frame, the compass gives ALIGN its alpha.
    // Cảm biến của điện thoại: GPS định nghĩa kế hoạch và khung cất cánh, la bàn
    // cung cấp alpha cho ALIGN.
    private LocationManager mLocationManager;
    private SensorManager mSensorManager;
    private Sensor mRotationSensor;
    private Sensor mOrientationSensor;
    private final float[] mRotationMatrix = new float[9];
    private final float[] mOrientation = new float[3];
    private volatile boolean mHasGpsFix;
    private long mLastGpsElapsed;
    private volatile double mPhoneLat = Double.NaN;
    private volatile double mPhoneLon = Double.NaN;
    private volatile float mPhoneAccuracy = Float.NaN;
    private volatile float mCompassHeadingDeg = Float.NaN;

    // Launch frame, fixed at ALIGN (PROTOCOL.md section 6).
    // Khung cất cánh, cố định tại ALIGN (PROTOCOL.md mục 6).
    private volatile boolean mLaunched;
    private volatile double mLaunchLat = Double.NaN;
    private volatile double mLaunchLon = Double.NaN;
    private volatile double mAlphaDeg = Double.NaN;

    // Navigation state
    private final Navigator mNavigator = new Navigator();
    private volatile boolean mEngaged;
    private boolean mLanding;
    private se.bitcraze.crazyfliecontrol2.FlyHeader mHeader;
    private boolean mTapAddsWaypoint = true;
    private double mGuidedLat = Double.NaN;
    private double mGuidedLon = Double.NaN;
    private int mSelectedWaypoint = -1;
    private int mFlightMode = -1;

    // Estimated-track bookkeeping for the map.
    // Ghi sổ vết bay ước tính cho bản đồ.
    private double mLastTrackLat = Double.NaN;
    private double mLastTrackLon = Double.NaN;
    private long mLastMapUpdateElapsed;
    private boolean mCenteredOnPhone;

    // Views
    private MissionMapView mMapView;
    private TextView mStatusText;
    private TextView mReadoutText;
    private ListView mWaypointList;
    private ArrayAdapter<String> mWaypointAdapter;
    private Button mPauseButton;
    private Button mMapTypeButton;
    private Button mTapModeButton;

    private final Handler mHandler = new Handler(Looper.getMainLooper());

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
        setContentView(R.layout.activity_gps_nav);

        mStatusText = findViewById(R.id.gpsnav_status);
        mReadoutText = findViewById(R.id.gpsnav_readout);
        mMapView = findViewById(R.id.gpsnav_map);
        mWaypointList = findViewById(R.id.gpsnav_waypoint_list);
        mPauseButton = findViewById(R.id.gpsnav_pause);
        mMapTypeButton = findViewById(R.id.gpsnav_map_type);
        mTapModeButton = findViewById(R.id.gpsnav_tap_mode);

        mWaypointAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1);
        mWaypointList.setAdapter(mWaypointAdapter);
        mWaypointList.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                mSelectedWaypoint = position;
            }
        });

        mMapView.setOnMapTapListener(new MissionMapView.OnMapTapListener() {
            @Override
            public void onMapTap(double latitude, double longitude) {
                if (mTapAddsWaypoint) {
                    synchronized (mNavigator) {
                        mNavigator.getMission().addWaypoint(new Waypoint(latitude, longitude));
                    }
                    refreshMissionUi();
                } else {
                    mGuidedLat = latitude;
                    mGuidedLon = longitude;
                    mMapView.setGuidedTarget(latitude, longitude);
                    updateStatusUi(null);
                }
            }
        });

        findViewById(R.id.gpsnav_mission).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onMission();
            }
        });
        findViewById(R.id.gpsnav_goto).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onGoToHere();
            }
        });
        findViewById(R.id.gpsnav_rtl).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onRtl();
            }
        });
        mPauseButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onPauseResume();
            }
        });
        findViewById(R.id.gpsnav_stop).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onStopNav();
            }
        });
        findViewById(R.id.gpsnav_launch).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onLaunch();
            }
        });
        findViewById(R.id.gpsnav_land).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onLand();
            }
        });
        mMapTypeButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onMapType();
            }
        });
        mTapModeButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onTapMode();
            }
        });
        // Shared header: sections, status, CUT MOTORS (redesign stage 3).
        // Thanh tiêu đề dùng chung: các mục, trạng thái, CUT MOTORS (giai đoạn thiết kế lại 3).
        mHeader = new FlyHeader(this, findViewById(R.id.gpsnav_header), FlyHeader.MISSION,
                new Runnable() {
                    @Override
                    public void run() {
                        onPanic();
                    }
                });
        mHeader.setLeaveGuard(new FlyHeader.LeaveGuard() {
            @Override
            public String blockReason() {
                // Leaving finishes this screen and its 5 Hz send loop with it.
                // Rời màn hình sẽ đóng màn này cùng vòng lặp gửi 5 Hz của nó.
                return (mLaunched || mEngaged)
                        ? "Mission screen is flying the drone - land it or use CUT MOTORS before switching"
                        : null;
            }
        });
        findViewById(R.id.gpsnav_wp_up).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onMoveWaypoint(-1);
            }
        });
        findViewById(R.id.gpsnav_wp_down).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onMoveWaypoint(1);
            }
        });
        findViewById(R.id.gpsnav_wp_remove).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onRemoveWaypoint();
            }
        });
        findViewById(R.id.gpsnav_load).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onLoadMission();
            }
        });
        findViewById(R.id.gpsnav_save).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onSaveMission();
            }
        });

        mLocationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

        requestLocationPermission();
        refreshMissionUi();
        updateStatusUi(null);
    }

    @Override
    protected void onResume() {
        super.onResume();
        startLocationUpdates();
        registerSensors();
        attachLink();
        updateStatusUi(null);
        mHeader.start();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mHeader.stop();
        if (mLocationManager != null) {
            mLocationManager.removeUpdates(this);
        }
        if (mSensorManager != null) {
            mSensorManager.unregisterListener(this);
        }
        // The 5 Hz send loop deliberately keeps running across onPause: a Storage Access
        // Framework dialog pauses this activity, and any 500 ms CRTP gap trips the failsafe
        // while the drone is armed. It is stopped in onDestroy().
        // Vòng lặp gửi 5 Hz cố ý chạy tiếp qua onPause: hộp thoại Storage Access Framework làm
        // activity này tạm dừng, và bất kỳ khoảng trống CRTP 500 ms nào cũng kích hoạt failsafe
        // khi drone đang armed. Vòng lặp được dừng trong onDestroy().
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        disengage();
        detachLink();
    }

    // region drone link

    private Crazyflie link() {
        Crazyflie crazyflie = DroneSession.get();
        if (crazyflie == null) {
            toast("Drone not connected");
        }
        return crazyflie;
    }

    private void attachLink() {
        Crazyflie crazyflie = DroneSession.get();
        if (crazyflie == mCrazyflie && (crazyflie == null || mNavLogStarted)) {
            return;
        }
        detachLink();
        mCrazyflie = crazyflie;
        if (mCrazyflie != null) {
            tryStartNavLog();
        }
    }

    private void detachLink() {
        mHandler.removeCallbacks(mLogRetry);
        if (mNavLog != null && mCrazyflie != null) {
            Logg logg = mCrazyflie.getLogg();
            if (logg != null) {
                logg.removeLogListener(mLogListener);
                if (mNavLogStarted) {
                    logg.stop(mNavLog);
                }
                logg.delete(mNavLog);
            }
        }
        mNavLog = null;
        mNavLogStarted = false;
        mCrazyflie = null;
    }

    private void tryStartNavLog() {
        if (mCrazyflie == null || mNavLogStarted) {
            return;
        }
        Logg logg = mCrazyflie.getLogg();
        // addConfig dereferences the TOC without a null check, so wait until it is fetched.
        // addConfig giải tham chiếu TOC mà không kiểm tra null, nên phải đợi TOC được lấy về.
        if (logg == null || logg.getToc() == null) {
            mHandler.removeCallbacks(mLogRetry);
            mHandler.postDelayed(mLogRetry, LOG_RETRY_MS);
            return;
        }
        if (mNavLog == null) {
            // nav.pos_x/pos_y/tgt_x/tgt_y + flight.heading + flight.mode = 21 B, under the
            // 25 byte data budget for one log block (PROTOCOL.md section 4).
            // nav.pos_x/pos_y/tgt_x/tgt_y + flight.heading + flight.mode = 21 B, nằm trong
            // ngân sách 25 byte dữ liệu cho một khối log (PROTOCOL.md mục 4).
            mNavLog = new LogConfig("nav", LOG_PERIOD_MS);
            mNavLog.addVariable("nav.pos_x");
            mNavLog.addVariable("nav.pos_y");
            mNavLog.addVariable("nav.tgt_x");
            mNavLog.addVariable("nav.tgt_y");
            mNavLog.addVariable("flight.heading");
            mNavLog.addVariable("flight.mode");
        }
        logg.addLogListener(mLogListener);
        logg.addConfig(mNavLog);
        logg.start(mNavLog);
        mNavLogStarted = true;
    }

    private final Runnable mLogRetry = new Runnable() {
        @Override
        public void run() {
            tryStartNavLog();
        }
    };

    private final LogAdapter mLogListener = new LogAdapter() {
        @Override
        public void logDataReceived(LogConfig logConfig, Map<String, Number> data, int timestamp) {
            if (logConfig != mNavLog) {
                return;
            }
            Number posX = data.get("nav.pos_x");
            Number posY = data.get("nav.pos_y");
            if (posX == null || posY == null) {
                return;
            }
            float x = posX.floatValue();
            float y = posY.floatValue();
            Number heading = data.get("flight.heading");
            final float headingDeg = heading != null ? heading.floatValue() : Float.NaN;
            Number mode = data.get("flight.mode");
            final int flightMode = mode != null ? mode.intValue() : -1;

            final Navigator.NavStatus status;
            synchronized (mNavigator) {
                mNavigator.setDroneHeading(headingDeg);
                status = mNavigator.step(x, y, mPhoneAccuracy);
            }

            final double xForUi = x;
            final double yForUi = y;
            mHandler.post(new Runnable() {
                @Override
                public void run() {
                    mFlightMode = flightMode;
                    onDroneTick(xForUi, yForUi, headingDeg, status);
                }
            });
        }
    };

    // endregion

    // region 5 Hz command loop

    private void engage() {
        mEngaged = true;
        mHandler.removeCallbacks(mSendLoop);
        mHandler.post(mSendLoop);
    }

    private void disengage() {
        mEngaged = false;
        mHandler.removeCallbacks(mSendLoop);
    }

    private final Runnable mSendLoop = new Runnable() {
        @Override
        public void run() {
            sendNavUpdate();
            if (mEngaged) {
                mHandler.postDelayed(this, TARGET_INTERVAL_MS);
            }
        }
    };

    private void sendNavUpdate() {
        Crazyflie crazyflie = DroneSession.get();
        if (crazyflie == null) {
            return;
        }
        if (mLanding) {
            // LAND is idempotent and must be repeated: a 500 ms CRTP gap trips the failsafe
            // while armed, which would cut the motors mid-descent.
            // LAND là lệnh idempotent và phải được lặp lại: khoảng trống CRTP
            // 500 ms sẽ kích hoạt failsafe khi đang armed, làm động cơ tắt giữa lúc hạ.
            crazyflie.sendPacket(FlightControlPacket.land());
            return;
        }
        if (!mLaunched) {
            return;
        }

        Navigator.NavMode mode;
        boolean paused;
        Waypoint target;
        synchronized (mNavigator) {
            mode = mNavigator.getMode();
            paused = mNavigator.isPaused();
            target = mNavigator.currentTarget();
        }

        if (mode != Navigator.NavMode.IDLE && !paused && target != null) {
            float[] northEast = target.toNorthEast(mLaunchLat, mLaunchLon);
            crazyflie.sendPacket(NavPacket.target(northEast[0], northEast[1]));
            // MISSION and GUIDED chase the target in FOLLOW; RTL hands over to RTH mode
            // (PROTOCOL.md section 10). SET_MODE is resent every tick on purpose: every
            // command is idempotent and the failsafe counts any CRTP gap.
            // MISSION và GUIDED đuổi theo mục tiêu ở chế độ FOLLOW; RTL giao cho chế độ RTH
            // (PROTOCOL.md mục 10). SET_MODE được gửi lại mỗi nhịp là cố ý: mọi lệnh đều
            // idempotent và failsafe tính mọi khoảng trống CRTP.
            crazyflie.sendPacket(FlightControlPacket.setMode(mode == Navigator.NavMode.RTL
                    ? FlightControlPacket.MODE_RTH
                    : FlightControlPacket.MODE_FOLLOW));
        } else {
            // Paused or idle: stop sending TARGET so the drone stops chasing after its 3 s
            // timeout, but keep SET_MODE flowing so the drone holds instead of tripping the
            // 500 ms failsafe.
            // Tạm dừng hoặc nhàn rỗi: ngừng gửi TARGET để drone ngừng đuổi theo sau khi hết
            // thời gian chờ 3 giây, nhưng vẫn gửi SET_MODE để drone giữ độ cao thay vì kích
            // hoạt failsafe 500 ms.
            crazyflie.sendPacket(FlightControlPacket.setMode(FlightControlPacket.MODE_HOLD));
        }
    }

    // endregion

    // region buttons

    private void onLaunch() {
        Crazyflie crazyflie = link();
        if (crazyflie == null) {
            return;
        }
        if (mLaunched) {
            toast("Already launched - after landing, press CUT MOTORS to reset and re-ALIGN");
            return;
        }
        if (Double.isNaN(mPhoneLat)) {
            toast("Waiting for a GPS fix");
            return;
        }
        if (mPhoneAccuracy > Navigator.MAX_POSITION_ACCURACY_M) {
            toast(String.format(Locale.US,
                    "GPS accuracy %.1f m - cannot ALIGN on a bad fix", mPhoneAccuracy));
            return;
        }
        if (Float.isNaN(mCompassHeadingDeg)) {
            toast("No compass heading - cannot ALIGN");
            return;
        }

        // The launch frame is defined once, here: the phone's position becomes the origin every
        // TARGET offset is measured from, and its compass bearing becomes alpha (PROTOCOL.md
        // section 6). The drone cannot measure that angle itself - it has no magnetometer.
        // Khung cất cánh được định nghĩa một lần, tại đây: vị trí của điện thoại
        // trở thành gốc mà mọi độ lệch TARGET được đo từ đó, và phương hướng la bàn của nó trở
        // thành alpha (PROTOCOL.md mục 6). Drone không tự đo được góc đó - nó không có cảm biến
        // từ kế.
        mLaunchLat = mPhoneLat;
        mLaunchLon = mPhoneLon;
        mAlphaDeg = mCompassHeadingDeg;
        synchronized (mNavigator) {
            mNavigator.markHome(mLaunchLat, mLaunchLon);
            mNavigator.setLaunchHeading(mAlphaDeg);
        }
        // ALIGN zeroes the dead reckoning and starts a fresh launch frame; RESET repeats the
        // zeroing and is harmless (both are idempotent).
        // ALIGN đưa tự dẫn đường quán tính về không và khởi tạo khung cất cánh mới;
        // RESET lặp lại việc đưa về không đó và vô hại (cả hai đều idempotent).
        crazyflie.sendPacket(NavPacket.align((float) mAlphaDeg));
        crazyflie.sendPacket(NavPacket.reset());
        crazyflie.sendPacket(FlightControlPacket.arm());
        crazyflie.sendPacket(FlightControlPacket.takeoff(DEFAULT_HOVER_THRUST));

        mLaunched = true;
        mLanding = false;
        mMapView.setHome(mLaunchLat, mLaunchLon);
        engage();
        toast(String.format(Locale.US, "Launch frame set: alpha=%.1f deg", mAlphaDeg));
        updateStatusUi(null);
    }

    private void onLand() {
        Crazyflie crazyflie = link();
        if (crazyflie == null) {
            return;
        }
        synchronized (mNavigator) {
            mNavigator.stop();
        }
        mLanding = true;
        engage();
        toast("Landing");
    }

    private void onPanic() {
        Crazyflie crazyflie = DroneSession.get();
        if (crazyflie != null) {
            crazyflie.sendPacket(FlightControlPacket.emergency());
        }
        synchronized (mNavigator) {
            mNavigator.stop();
        }
        mLanding = false;
        mLaunched = false;
        disengage();
        toast("CUT MOTORS - motors off");
        updateStatusUi(null);
    }

    private void onMission() {
        if (!requireLaunched()) {
            return;
        }
        boolean started;
        synchronized (mNavigator) {
            started = mNavigator.startMission();
        }
        if (!started) {
            toast("Mission is empty - tap the map to add waypoints");
            return;
        }
        mLanding = false;
        engage();
        refreshMissionUi();
    }

    private void onGoToHere() {
        if (!requireLaunched()) {
            return;
        }
        if (Double.isNaN(mGuidedLat) || Double.isNaN(mGuidedLon)) {
            toast("Tap the map first (tap mode: go-to)");
            return;
        }
        synchronized (mNavigator) {
            mNavigator.startGuided(mGuidedLat, mGuidedLon);
        }
        mLanding = false;
        engage();
    }

    private void onRtl() {
        if (!requireLaunched()) {
            return;
        }
        boolean started;
        synchronized (mNavigator) {
            started = mNavigator.startRtl();
        }
        if (!started) {
            toast("No launch point recorded");
            return;
        }
        mLanding = false;
        engage();
    }

    private void onPauseResume() {
        synchronized (mNavigator) {
            if (mNavigator.isPaused()) {
                mNavigator.resume();
            } else {
                mNavigator.pause();
            }
        }
        updateStatusUi(null);
    }

    private void onStopNav() {
        synchronized (mNavigator) {
            mNavigator.stop();
        }
        mMapView.setGuidedTarget(Double.NaN, Double.NaN);
        toast("Navigation stopped - drone holding");
        refreshMissionUi();
    }

    private boolean requireLaunched() {
        if (!mLaunched) {
            toast("Launch first (ALIGN fixes the launch frame)");
            return false;
        }
        return true;
    }

    private void onMapType() {
        int current = mMapView.getMapType();
        int next;
        String name;
        if (current == MissionMapView.MAP_TYPE_SATELLITE) {
            next = MissionMapView.MAP_TYPE_STREET;
            name = "street";
        } else if (current == MissionMapView.MAP_TYPE_STREET) {
            next = MissionMapView.MAP_TYPE_TERRAIN;
            name = "terrain";
        } else {
            next = MissionMapView.MAP_TYPE_SATELLITE;
            name = "satellite";
        }
        mMapView.setMapType(next);
        mMapTypeButton.setText("Map: " + name);
    }

    private void onTapMode() {
        mTapAddsWaypoint = !mTapAddsWaypoint;
        mTapModeButton.setText(mTapAddsWaypoint ? "Tap: add WP" : "Tap: go-to");
    }

    // endregion

    // region waypoint list editing

    private void onMoveWaypoint(int delta) {
        if (mSelectedWaypoint < 0) {
            toast("Select a waypoint first");
            return;
        }
        synchronized (mNavigator) {
            Mission mission = mNavigator.getMission();
            int to = mSelectedWaypoint + delta;
            if (to < 0 || to >= mission.size()) {
                return;
            }
            mission.moveWaypoint(mSelectedWaypoint, to);
            mSelectedWaypoint = to;
        }
        refreshMissionUi();
        mWaypointList.setItemChecked(mSelectedWaypoint, true);
    }

    private void onRemoveWaypoint() {
        if (mSelectedWaypoint < 0) {
            toast("Select a waypoint first");
            return;
        }
        synchronized (mNavigator) {
            mNavigator.getMission().removeWaypoint(mSelectedWaypoint);
        }
        mSelectedWaypoint = -1;
        refreshMissionUi();
    }

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
                String contents = readAll(uri);
                synchronized (mNavigator) {
                    mNavigator.loadMissionFile(contents);
                    if (mLaunched) {
                        // A loaded file's home row is a planning artifact; the launch point is
                        // the reference every TARGET offset is measured from.
                        // Dòng home trong file nạp vào chỉ là sản phẩm của việc lập kế hoạch;
                        // điểm cất cánh mới là gốc tham chiếu mà mọi độ lệch TARGET được đo từ đó.
                        mNavigator.markHome(mLaunchLat, mLaunchLon);
                    }
                }
                refreshMissionUi();
                toast("Mission loaded");
            } catch (IOException e) {
                toast("Load failed: " + e.getMessage());
            }
        } else if (requestCode == REQUEST_SAVE_WAYPOINTS) {
            String contents;
            synchronized (mNavigator) {
                contents = mNavigator.saveMissionFile();
            }
            try {
                writeAll(uri, contents);
                toast("Mission saved");
            } catch (IOException e) {
                toast("Save failed: " + e.getMessage());
            }
        }
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

    private void refreshMissionUi() {
        int activeIndex = -1;
        synchronized (mNavigator) {
            List<Waypoint> waypoints = mNavigator.getMission().getWaypoints();
            mWaypointAdapter.clear();
            for (int i = 0; i < waypoints.size(); i++) {
                Waypoint wp = waypoints.get(i);
                mWaypointAdapter.add(String.format(Locale.US, "%d: %.6f, %.6f",
                        i + 1, wp.getLatitude(), wp.getLongitude()));
            }
            mWaypointAdapter.notifyDataSetChanged();
            if (mNavigator.getMode() == Navigator.NavMode.MISSION) {
                activeIndex = mNavigator.getActiveIndex();
            }
            mMapView.setWaypoints(new ArrayList<Waypoint>(waypoints), activeIndex);
        }
    }

    // endregion

    // region phone GPS + compass

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
        } else {
            startLocationUpdates();
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions,
            int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_LOCATION_PERMISSION) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                startLocationUpdates();
            } else {
                toast("Location permission denied - GPS navigation unavailable");
            }
        }
    }

    private void startLocationUpdates() {
        if (mLocationManager == null
                || ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        try {
            mLocationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 500, 0.5f, this);
        } catch (IllegalArgumentException e) {
            logMissingProvider(LocationManager.GPS_PROVIDER);
        }
        try {
            mLocationManager.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 1000, 0f, this);
        } catch (IllegalArgumentException e) {
            logMissingProvider(LocationManager.NETWORK_PROVIDER);
        }
    }

    private void logMissingProvider(String provider) {
        Log.w(LOG_TAG, "No location provider: " + provider);
    }

    @Override
    public void onLocationChanged(Location location) {
        boolean fromGps = LocationManager.GPS_PROVIDER.equals(location.getProvider());
        // Network fixes are tens of metres off; only use them while no fresh GPS fix exists,
        // or the launch frame would be built on cell-tower geometry.
        // Bản vị từ network lệch vài chục mét; chỉ dùng khi chưa có bản vị GPS mới, nếu không
        // khung cất cánh sẽ được dựng trên hình học của các trạm phát sóng.
        if (!fromGps && mHasGpsFix
                && (SystemClock.elapsedRealtime() - mLastGpsElapsed) < GPS_FIX_MAX_AGE_MS) {
            return;
        }
        if (fromGps) {
            mHasGpsFix = true;
            mLastGpsElapsed = SystemClock.elapsedRealtime();
        }
        mPhoneLat = location.getLatitude();
        mPhoneLon = location.getLongitude();
        mPhoneAccuracy = location.hasAccuracy() ? location.getAccuracy() : Float.NaN;

        if (!mCenteredOnPhone) {
            // Centre the map once on the first fix so the pilot is not panning from (0, 0).
            // Căn giữa bản đồ một lần ở bản vị đầu tiên để người điều khiển không phải kéo từ (0, 0).
            mMapView.getController().setCenter(
                    new GeoPoint(mPhoneLat, mPhoneLon));
            mCenteredOnPhone = true;
        }
        updateStatusUi(null);
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {
    }

    @Override
    public void onProviderEnabled(String provider) {
    }

    @Override
    public void onProviderDisabled(String provider) {
    }

    private void registerSensors() {
        if (mSensorManager == null) {
            return;
        }
        mRotationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        if (mRotationSensor != null) {
            mSensorManager.registerListener(this, mRotationSensor, SensorManager.SENSOR_DELAY_UI);
            return;
        }
        // Rotation vector is missing on some cheap devices; the deprecated orientation sensor
        // still yields an azimuth and ALIGN needs one.
        // Rotation vector thiếu trên một số thiết bị rẻ tiền; cảm biến orientation đã deprecated
        // vẫn cho ra phương vị và ALIGN cần một giá trị như vậy.
        mOrientationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION);
        if (mOrientationSensor != null) {
            mSensorManager.registerListener(this, mOrientationSensor, SensorManager.SENSOR_DELAY_UI);
        } else {
            android.util.Log.w(LOG_TAG, "No compass sensor - ALIGN unavailable");
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            SensorManager.getRotationMatrixFromVector(mRotationMatrix, event.values);
            SensorManager.getOrientation(mRotationMatrix, mOrientation);
            // mOrientation[0] is the azimuth: rotation about the z axis from magnetic north,
            // which is the compass bearing ALIGN wants as alpha.
            // mOrientation[0] là phương vị: góc quay quanh trục z tính từ hướng từ bắc, chính là
            // phương hướng la bàn mà ALIGN cần làm alpha.
            mCompassHeadingDeg = (float) Math.toDegrees(mOrientation[0]);
        } else if (event.sensor.getType() == Sensor.TYPE_ORIENTATION) {
            mCompassHeadingDeg = event.values[0];
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    // endregion

    // region UI updates

    /** Called on the UI thread for every nav log tick. */
    /** Được gọi trên luồng UI cho mỗi nhịp log điều hướng. */
    private void onDroneTick(double posX, double posY, float headingDeg, Navigator.NavStatus status) {
        updateStatusUi(status);

        if (!mLaunched || Double.isNaN(mAlphaDeg)) {
            return;
        }

        double[] latLon = GeoMath.launchFrameToLatLon(
                (float) posX, (float) posY, mAlphaDeg, mLaunchLat, mLaunchLon);

        if (Double.isNaN(mLastTrackLat)
                || GeoMath.distanceMeters(mLastTrackLat, mLastTrackLon, latLon[0], latLon[1])
                >= TRACK_MIN_STEP_M) {
            mLastTrackLat = latLon[0];
            mLastTrackLon = latLon[1];
            mMapView.appendTrackPoint(latLon[0], latLon[1]);
        }

        long now = SystemClock.elapsedRealtime();
        if (now - mLastMapUpdateElapsed >= TARGET_INTERVAL_MS) {
            mLastMapUpdateElapsed = now;
            mMapView.setDrone(latLon[0], latLon[1],
                    Float.isNaN(headingDeg) ? 0.0 : headingDeg);
        }

        if (status.isArrived()) {
            refreshMissionUi();
        }
    }

    private void updateStatusUi(Navigator.NavStatus status) {
        if (status == null) {
            synchronized (mNavigator) {
                status = mNavigator.getLastStatus();
            }
        }
        float accuracy = mPhoneAccuracy;

        StringBuilder line = new StringBuilder();
        line.append("link: ").append(DroneSession.get() != null ? "connected" : "NOT CONNECTED");
        line.append("  |  GPS accuracy: ").append(Float.isNaN(accuracy)
                ? "no fix"
                : String.format(Locale.US, "%.1f m", accuracy));
        line.append("  |  mode: ").append(status != null ? status.getMode() : Navigator.NavMode.IDLE);
        if (status != null && status.getActiveWaypointIndex() >= 0) {
            line.append("  |  WP ").append(status.getActiveWaypointIndex() + 1);
        }
        if (mFlightMode >= 0) {
            line.append("  |  flight.mode ").append(mFlightMode);
        }
        if (status != null && !status.getMessage().isEmpty()) {
            line.append("  |  ").append(status.getMessage());
        }
        mStatusText.setText(line.toString());

        StringBuilder readout = new StringBuilder("to target: ");
        if (status == null || Float.isNaN(status.getDistanceMeters())) {
            readout.append("-");
        } else {
            readout.append(String.format(Locale.US, "%.1f m, bearing %.0f",
                    status.getDistanceMeters(), status.getBearingDegrees()));
            if (!Float.isNaN(status.getTurnErrorDegrees())) {
                readout.append(String.format(Locale.US, ", turn %+.0f (right +)",
                        status.getTurnErrorDegrees()));
            }
        }
        readout.append(mLaunched
                ? "  (from estimated drone position)"
                : "  (launch to fly)");
        mReadoutText.setText(readout.toString());

        mPauseButton.setText(isPaused() ? "Resume" : "Pause");
        // The icon follows the label: it shows what a tap will DO, like the text.
        // Biểu tượng theo nhãn: nó cho biết một lần chạm sẽ LÀM gì, giống như chữ.
        if (mPauseButton instanceof MaterialButton) {
            ((MaterialButton) mPauseButton).setIconResource(
                    isPaused() ? R.drawable.ic_btn_play : R.drawable.ic_btn_pause);
        }
    }

    private boolean isPaused() {
        synchronized (mNavigator) {
            return mNavigator.isPaused();
        }
    }

    private void toast(String message) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show();
    }

    // endregion
}
