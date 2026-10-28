package se.bitcraze.crazyfliecontrol2.tune;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.text.InputType;
import android.util.Log;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.TooltipCompat;
import androidx.core.content.ContextCompat;

import com.espressif.espdrone.android.R;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import se.bitcraze.crazyflie.lib.crazyflie.Crazyflie;
import se.bitcraze.crazyflie.lib.crtp.FlightControlPacket;
import se.bitcraze.crazyflie.lib.crtp.CrtpPacket;
import se.bitcraze.crazyflie.lib.log.LogAdapter;
import se.bitcraze.crazyflie.lib.log.LogConfig;
import se.bitcraze.crazyflie.lib.log.Logg;
import se.bitcraze.crazyflie.lib.param.Param;
import se.bitcraze.crazyflie.lib.param.ParamListener;
import se.bitcraze.crazyflie.lib.toc.VariableType;
import se.bitcraze.crazyfliecontrol2.DroneSession;
import se.bitcraze.crazyfliecontrol2.FlyHeader;

/**
 * PID tuning screen: six strip charts (angle/controller and P/I/D terms per axis), live
 * statistics, the 13 parameter editors with -10%/+10% nudges, Save/Load/Defaults, and a
 * safety-gated auto-tuner for one axis at a time.
 * Màn hình tinh chỉnh PID: sáu biểu đồ dải (góc/đầu ra điều khiển và các thành phần P/I/D theo
 * từng trục), thống kê trực tiếp, 13 trình sửa tham số với các nút -10%/+10%, Save/Load/Defaults,
 * và bộ tự tinh chỉnh có kiểm tra an toàn, mỗi lần một trục.
 *
 * <p>Telemetry comes from two log blocks at 20 Hz (PROTOCOL.md section 4): a Status block
 * that is always running and an axis tune block that follows whichever axis is selected.
 * The charts of a non-selected axis therefore show "Waiting for telemetry" until their
 * block is streaming.
 * <p>Dữ liệu từ xa đến từ hai khối log ở tần số 20 Hz (PROTOCOL.md mục 4): một khối Status
 * luôn chạy và một khối tune theo trục bám theo trục đang được chọn. Vì thế biểu đồ của trục
 * không được chọn sẽ hiển thị "Waiting for telemetry" cho đến khi khối của nó phát dữ liệu.
 *
 * <p>The connection is shared through {@link DroneSession}; nothing here owns the link.
 * <p>Kết nối được dùng chung qua {@link DroneSession}; không phần nào ở đây sở hữu liên kết.
 */
public class TuneActivity extends AppCompatActivity {

    private static final String LOG_TAG = "Crazyflie-TuneActivity";

    private static final int TUNE_PERIOD_MS = 50;          // 20 Hz charts
    // biểu đồ 20 Hz
    private static final int STATS_PERIOD_MS = 500;
    private static final int HOLD_TO_START_MS = 2000;
    private static final String STATUS_LOG_NAME = "TuneStatus";

    private static final String[] AXES = {"roll", "pitch", "yaw"};
    private static final String[] AXIS_UNITS = {" deg", " deg", " deg/s"};
    private static final String[] MODE_NAMES = {"MANUAL", "HOLD", "AUTO", "FOLLOW", "RTH"};

    // Colour-blind tolerant series colours (hard-coded on purpose: no res/values edits).
    // Màu chuỗi thân thiện với người mù màu (ghi cứng có chủ đích: không sửa res/values).
    private static final int COLOR_BLUE = 0xFF2196F3;
    private static final int COLOR_RED = 0xFFF44336;
    private static final int COLOR_GREEN = 0xFF4CAF50;
    private static final int COLOR_ORANGE = 0xFFFF9800;
    private static final int COLOR_PURPLE = 0xFF9C27B0;

    /** Public entry point; no extras needed, the link comes from {@link DroneSession}. */
    /** Điểm vào công khai; không cần extras, liên kết lấy từ {@link DroneSession}. */
    public static Intent newIntent(Context context) {
        return new Intent(context, TuneActivity.class);
    }

    private Crazyflie mCrazyflie;
    private Param mParam;
    private Logg mLogg;
    private LogConfig mStatusLogConfig;
    private LogConfig mTuneLogConfig;
    private LogAdapter mLogAdapter;

    private final ChartView[] mAngleCharts = new ChartView[3];
    private final ChartView[] mPidCharts = new ChartView[3];
    private final List<ParamEditor> mEditors = new ArrayList<>();
    private final List<ParamListener> mParamListeners = new ArrayList<>();

    private TextView mStatusLine;
    private FlyHeader mHeader;
    private TextView mStatsView;
    private LinearLayout mChartsHost;      // holds the active axis's two charts
    private TextView mTunerStatus;
    private TextView mMethodHint;
    private Spinner mAxisSpinner;
    private Spinner mMethodSpinner;
    private EditText mTrialsEdit;
    private EditText mTrialSecondsEdit;
    private Button mStartBtn;
    private Button mStopBtn;
    private ImageButton mPauseBtn;

    private String mAxis = "roll";
    private boolean mPaused;

    private volatile boolean mArmed;
    private volatile float mLastRoll;
    private volatile float mLastPitch;
    private volatile float mLastVbat;
    private volatile int mLastMode;
    private volatile int mLastAutoState;
    private boolean mPendingStart;

    private float mPrevAngle;
    private boolean mPrevAngleValid;

    private AutoTuner mTuner;

    // Windows for the live cost panel (the charts own their own buffers).
    // Các cửa sổ cho panel chi phí trực tiếp (các biểu đồ tự giữ bộ đệm riêng).
    private final Stats.Ring mAngleRing = new Stats.Ring(ChartView.CAPACITY);
    private final Stats.Ring mRateRing = new Stats.Ring(ChartView.CAPACITY);
    private final Stats.Ring mMotorRing = new Stats.Ring(ChartView.CAPACITY);
    private final Stats.Ring mControlRing = new Stats.Ring(ChartView.CAPACITY);

    private final Handler mMain = new Handler(Looper.getMainLooper());
    private long mHoldStart;

    // TOC readiness: how often to poll while the fetch is in flight (see onCreate).
    // Độ sẵn sàng của TOC: tần suất kiểm tra trong lúc đang lấy (xem onCreate).
    private static final int TOC_WAIT_MS = 250;
    private boolean mUiStarted;

    // ---- bench motor test (PROTOCOL.md §5, FC_CMD_MOTOR_TEST 0x0C) ----
    // The firmware holds the test only while the GUI re-sends within MOTOR_TEST_TIMEOUT_MS
    // (300 ms), so a dropped link or a closed screen stops the motors by itself. We re-send
    // at 200 ms for margin. Disarmed only — the firmware refuses it while armed.
    // ---- thử động cơ trên bàn (PROTOCOL.md §5, FC_CMD_MOTOR_TEST 0x0C) ----
    // Firmware chỉ giữ thử trong khi GUI gửi lại trong MOTOR_TEST_TIMEOUT_MS (300 ms), nên
    // mất liên kết hay đóng màn hình là động cơ tự dừng. Chúng ta gửi lại mỗi 200 ms cho dư
    // biên. Chỉ khi chưa armed — firmware từ chối khi đang armed.
    private static final int MOTOR_TEST_PERIOD_MS = 200;
    private static final int MOTOR_THRUST_FLOOR = FlightControlPacket.MOTOR_THRUST_MIN;   // 1000 µs
    private static final int MOTOR_THRUST_RANGE = FlightControlPacket.MOTOR_THRUST_MAX
            - FlightControlPacket.MOTOR_THRUST_MIN;                                       // 1000 µs span

    private Button mMotorStartBtn;
    private TextView mMotorStatus;
    private final SeekBar[] mMotorSliders = new SeekBar[4];
    private final TextView[] mMotorValues = new TextView[4];
    private volatile boolean mMotorTestRunning;

    private final Runnable mMotorTestTick = new Runnable() {
        @Override
        public void run() {
            if (!mMotorTestRunning) {
                return;
            }
            sendFlightControl(FlightControlPacket.motorTest(
                    true, motorThrust(0), motorThrust(1), motorThrust(2), motorThrust(3)));
            mMain.postDelayed(this, MOTOR_TEST_PERIOD_MS);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mCrazyflie = DroneSession.get();
        if (mCrazyflie == null) {
            Toast.makeText(this, "No drone connection - connect from the main screen first", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        mParam = mCrazyflie.getParam();
        mLogg = mCrazyflie.getLogg();

        if (isTocReady()) {
            startUi();
            return;
        }
        // The TOC fetch is still in flight right after connecting. Closing here with
        // "try again in a moment" was the load issue: the screen refused to open on
        // every early tap. Wait for the TOC instead, then build the real UI.
        // Lấy TOC vẫn đang chạy ngay sau khi kết nối. Đóng màn hình ở đây với thông báo
        // "thử lại sau" chính là lỗi load: màn hình từ chối mở mỗi khi bấm sớm. Thay vào
        // đó đợi TOC, rồi dựng giao diện thật.
        setContentView(buildLoadingView());
        mMain.postDelayed(mTocWait, TOC_WAIT_MS);
    }

    private boolean isTocReady() {
        return mParam != null && mLogg != null && mLogg.getToc() != null;
    }

    private void startUi() {
        if (mUiStarted || isFinishingOrDestroyed()) {
            return;
        }
        mUiStarted = true;
        setContentView(buildUi());
        setupParamListeners();
        setupLogging();
        mMain.post(mStatsTick);
    }

    /** Placeholder shown while the TOC fetch completes; replaced by the real UI. */
    /** Trang giữ chỗ hiển thị trong lúc lấy TOC; sẽ thay bằng giao diện thật. */
    private View buildLoadingView() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setGravity(Gravity.CENTER);
        root.setBackgroundColor(Color.parseColor("#EFF1F3"));
        TextView text = new TextView(this);
        text.setTextSize(TypedValue.COMPLEX_UNIT_SP, 14);
        text.setTextColor(0xFF3A4048);
        text.setText("Loading drone data...");
        root.addView(text, matchWrap());
        return root;
    }

    private final Runnable mTocWait = new Runnable() {
        @Override
        public void run() {
            if (isFinishingOrDestroyed()) {
                return;
            }
            if (isTocReady()) {
                startUi();
                return;
            }
            if (DroneSession.get() != mCrazyflie) {
                Toast.makeText(TuneActivity.this,
                        "Connection lost while loading", Toast.LENGTH_LONG).show();
                finish();
                return;
            }
            mMain.postDelayed(this, TOC_WAIT_MS);
        }
    };

    // ------------------------------------------------------------- UI construction

    private View buildUi() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(Color.parseColor("#EFF1F3"));

        root.addView(buildTopBar(), new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

        // Portrait: the active axis's two charts stay pinned at the top so the pilot can
        // watch the response while nudging gains, and everything else scrolls below them.
        // Dọc: hai biểu đồ của trục đang chọn được ghim trên cùng để người lái quan sát
        // đáp ứng trong khi chỉnh hệ số, phần còn lại cuộn bên dưới.
        createCharts();
        root.addView(buildChartsPane(), new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

        ScrollView panelScroll = new ScrollView(this);
        panelScroll.setFillViewport(true);
        panelScroll.addView(buildPanel(), new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));
        root.addView(panelScroll, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f));
        return root;
    }

    /** Shared header (sections, status, CUT MOTORS) with the tuner's status line under it. */
    /** Header dùng chung (các mục, trạng thái, CUT MOTORS) với dòng trạng thái của bộ tinh chỉnh bên dưới. */
    private View buildTopBar() {
        LinearLayout top = new LinearLayout(this);
        top.setOrientation(LinearLayout.VERTICAL);

        View header = getLayoutInflater().inflate(R.layout.view_fly_header, top, false);
        top.addView(header);
        mHeader = new FlyHeader(this, header, FlyHeader.TUNE, new Runnable() {
            @Override
            public void run() {
                panic();
            }
        });
        mHeader.setLeaveGuard(new FlyHeader.LeaveGuard() {
            @Override
            public String blockReason() {
                return (mTuner != null && mTuner.isRunning())
                        ? "Auto-tune is running - stop it or use CUT MOTORS before switching"
                        : null;
            }
        });

        mStatusLine = new TextView(this);
        mStatusLine.setTextSize(TypedValue.COMPLEX_UNIT_SP, 11);
        mStatusLine.setTextColor(0xFF3A4048);
        mStatusLine.setPadding(dp(12), dp(2), dp(12), dp(2));
        top.addView(mStatusLine, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));
        return top;
    }

    /** All six charts exist from the start; only the selected pair is attached. */
    /** Cả sáu biểu đồ có ngay từ đầu; chỉ cặp của trục đang chọn được gắn vào giao diện. */
    private void createCharts() {
        for (int a = 0; a < AXES.length; a++) {
            String axis = AXES[a];
            mAngleCharts[a] = buildAngleChart(axis);
            mPidCharts[a] = buildPidChart(axis);
        }
    }

    /** Pinned pane: axis + chart tools row, the active axis's two charts, live stats. */
    /** Khung ghim: hàng trục + công cụ biểu đồ, hai biểu đồ của trục đang chọn, thống kê. */
    private View buildChartsPane() {
        LinearLayout pane = new LinearLayout(this);
        pane.setOrientation(LinearLayout.VERTICAL);
        pane.setPadding(dp(8), dp(4), dp(8), dp(4));

        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        // The axis picker drives both the charts shown and the axis auto-tune works on,
        // so it lives with the charts instead of inside the Auto-tune section.
        // Bộ chọn trục điều khiển cả biểu đồ lẫn trục auto-tune, nên nằm cạnh biểu đồ.
        row.addView(caption("Axis"));
        mAxisSpinner = new Spinner(this);
        ArrayAdapter<String> axisAdapter = new ArrayAdapter<>(this,
                android.R.layout.simple_spinner_item,
                new String[] {"Roll", "Pitch", "Yaw"});
        axisAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        mAxisSpinner.setAdapter(axisAdapter);
        row.addView(mAxisSpinner, spinnerParams());
        // Chart tools are routine and reversible, so they are icon-only (with a
        // content description + long-press tooltip). Flight commands keep their text.
        // Các công cụ biểu đồ là thường quy và hoàn tác được, nên chỉ dùng icon (kèm
        // content description + tooltip khi nhấn giữ). Các lệnh bay vẫn giữ nguyên chữ.
        mPauseBtn = iconButton(R.drawable.ic_btn_pause, "Pause charts");
        mPauseBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mPaused = !mPaused;
                for (ChartView c : mAngleCharts) {
                    c.setPaused(mPaused);
                }
                for (ChartView c : mPidCharts) {
                    c.setPaused(mPaused);
                }
                String label = mPaused ? "Resume charts" : "Pause charts";
                mPauseBtn.setImageResource(mPaused ? R.drawable.ic_btn_play : R.drawable.ic_btn_pause);
                mPauseBtn.setContentDescription(label);
                TooltipCompat.setTooltipText(mPauseBtn, label);
            }
        });
        row.addView(mPauseBtn, toolButtonParams());
        ImageButton clear = iconButton(R.drawable.ic_btn_clear, "Clear charts");
        clear.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                for (ChartView c : mAngleCharts) {
                    c.clear();
                }
                for (ChartView c : mPidCharts) {
                    c.clear();
                }
                mAngleRing.clear();
                mRateRing.clear();
                mMotorRing.clear();
                mControlRing.clear();
            }
        });
        row.addView(clear, toolButtonParams());
        pane.addView(row, matchWrap());

        mChartsHost = new LinearLayout(this);
        mChartsHost.setOrientation(LinearLayout.VERTICAL);
        pane.addView(mChartsHost, matchWrap());
        showAxisCharts(axisIndex(mAxis));

        mStatsView = new TextView(this);
        mStatsView.setTextSize(TypedValue.COMPLEX_UNIT_SP, 10);
        mStatsView.setTypeface(Typeface.MONOSPACE);
        mStatsView.setTextColor(0xFF202428);
        mStatsView.setText("Waiting for telemetry");
        pane.addView(mStatsView, matchWrap());

        mAxisSpinner.setOnItemSelectedListener(new android.widget.AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(android.widget.AdapterView<?> parent, View view, int position, long id) {
                String axis = AXES[position];
                if (axis.equals(mAxis)) {
                    return;
                }
                if (mTuner != null && mTuner.isRunning()) {
                    // one axis at a time: switching mid-run is blocked
                    // mỗi lần một trục: đổi trục giữa chừng bị chặn
                    mAxisSpinner.setSelection(axisIndex(mAxis));
                    Toast.makeText(TuneActivity.this,
                            "Auto-tune is running on " + mAxis + " - stop it first", Toast.LENGTH_LONG).show();
                    return;
                }
                switchTuneBlock(axis);
            }

            @Override
            public void onNothingSelected(android.widget.AdapterView<?> parent) {
            }
        });
        return pane;
    }

    /**
     * Show the selected axis's two charts (ChartView draws its own legend, so the pair
     * needs no titles). The detached pairs keep their ring buffers, so switching back
     * shows the history they collected.
     */
    /**
     * Hiển thị hai biểu đồ của trục đang chọn (ChartView tự vẽ chú giải nên cặp này không
     * cần tiêu đề). Các cặp tháo ra vẫn giữ vùng đệm, nên đổi lại sẽ thấy lịch sử đã thu.
     */
    private void showAxisCharts(int idx) {
        if (mChartsHost == null) {
            return;
        }
        mChartsHost.removeAllViews();
        mChartsHost.addView(mAngleCharts[idx], chartLayoutParams());
        mChartsHost.addView(mPidCharts[idx], chartLayoutParams());
    }

    private LinearLayout.LayoutParams chartLayoutParams() {
        // ChartView's own measure wants 240dp tall; two pinned charts at that height
        // would leave no room for the panel below, so pin a compact exact height
        // (the exact LayoutParams override the 240dp onMeasure wish).
        // ChartView tự đo muốn cao 240dp; hai biểu đồ ghim cao thế sẽ không còn chỗ cho
        // khung bên dưới, nên ghim chiều cao gọn tường minh (LayoutParams chính xác
        // sẽ thắng mong muốn 240dp của onMeasure).
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(150));
        lp.topMargin = dp(4);
        return lp;
    }

    private LinearLayout.LayoutParams toolButtonParams() {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(dp(48), dp(48));
        lp.leftMargin = dp(6);
        return lp;
    }

    private ChartView buildAngleChart(String axis) {
        ChartView chart = new ChartView(this);
        boolean yaw = "yaw".equals(axis);
        List<ChartView.Series> series = new ArrayList<>();
        series.add(new ChartView.Series(capitalize(axis) + (yaw ? " rate" : " angle"),
                AXIS_UNITS[axisIndex(axis)], COLOR_BLUE, 1f));
        series.add(new ChartView.Series(capitalize(axis) + " controller", "", COLOR_ORANGE, 1f));
        chart.setSeries(series);
        return chart;
    }

    private ChartView buildPidChart(String axis) {
        ChartView chart = new ChartView(this);
        List<ChartView.Series> series = new ArrayList<>();
        series.add(new ChartView.Series(capitalize(axis) + " p", "", COLOR_RED, 1f));
        series.add(new ChartView.Series(capitalize(axis) + " i", "", COLOR_GREEN, 1f));
        series.add(new ChartView.Series(capitalize(axis) + " d", "", COLOR_PURPLE, 1f));
        chart.setSeries(series);
        chart.setSharedScale(true);
        chart.setUnit("");
        return chart;
    }

    private View buildPanel() {
        LinearLayout panel = new LinearLayout(this);
        panel.setOrientation(LinearLayout.VERTICAL);
        panel.setPadding(dp(8), dp(4), dp(8), dp(16));

        // Auto-tune (the Axis picker is pinned up with the charts)
        // Auto-tune (bộ chọn trục được ghim phía trên cùng biểu đồ)
        panel.addView(sectionLabel("Auto-tune (one axis at a time)"));

        LinearLayout methodRow = new LinearLayout(this);
        methodRow.setOrientation(LinearLayout.HORIZONTAL);
        methodRow.setGravity(Gravity.CENTER_VERTICAL);
        methodRow.addView(caption("Method"));
        mMethodSpinner = new Spinner(this);
        AutoTuner.Method[] methods = AutoTuner.Method.values();
        String[] labels = new String[methods.length];
        for (int i = 0; i < methods.length; i++) {
            labels[i] = methods[i].label;
        }
        ArrayAdapter<String> methodAdapter = new ArrayAdapter<>(this,
                android.R.layout.simple_spinner_item, labels);
        methodAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        mMethodSpinner.setAdapter(methodAdapter);
        methodRow.addView(mMethodSpinner, spinnerParams());
        panel.addView(methodRow, matchWrap());
        mMethodHint = new TextView(this);
        mMethodHint.setTextSize(TypedValue.COMPLEX_UNIT_SP, 11);
        mMethodHint.setTextColor(Color.parseColor("#5A6068"));
        mMethodHint.setText(methods[0].hint);
        panel.addView(mMethodHint, matchWrap());
        mMethodSpinner.setOnItemSelectedListener(new android.widget.AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(android.widget.AdapterView<?> parent, View view, int position, long id) {
                mMethodHint.setText(AutoTuner.Method.values()[position].hint);
            }

            @Override
            public void onNothingSelected(android.widget.AdapterView<?> parent) {
            }
        });

        LinearLayout trialsRow = new LinearLayout(this);
        trialsRow.setOrientation(LinearLayout.HORIZONTAL);
        trialsRow.setGravity(Gravity.CENTER_VERTICAL);
        trialsRow.addView(caption("Trials"));
        mTrialsEdit = numberField("40");
        trialsRow.addView(mTrialsEdit, smallFieldParams());
        trialsRow.addView(caption("Trial s"));
        mTrialSecondsEdit = numberField("6");
        trialsRow.addView(mTrialSecondsEdit, smallFieldParams());
        panel.addView(trialsRow, matchWrap());

        mStartBtn = new Button(this);
        mStartBtn.setAllCaps(false);
        mStartBtn.setTextColor(Color.WHITE);
        mStartBtn.setBackgroundColor(0xFF2E7D32);
        mStartBtn.setTextSize(TypedValue.COMPLEX_UNIT_SP, 12);
        setButtonIcon(mStartBtn, R.drawable.ic_btn_play, Color.WHITE);
        resetStartButton();
        mStartBtn.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (mTuner != null && mTuner.isRunning()) {
                            Toast.makeText(TuneActivity.this, "Already running", Toast.LENGTH_SHORT).show();
                            return true;
                        }
                        mHoldStart = SystemClock.elapsedRealtime();
                        mStartBtn.removeCallbacks(mHoldFire);
                        mStartBtn.removeCallbacks(mHoldTick);
                        mStartBtn.post(mHoldTick);
                        mStartBtn.postDelayed(mHoldFire, HOLD_TO_START_MS);
                        return true;
                    case MotionEvent.ACTION_UP:
                    case MotionEvent.ACTION_CANCEL:
                        mStartBtn.removeCallbacks(mHoldFire);
                        mStartBtn.removeCallbacks(mHoldTick);
                        resetStartButton();
                        return true;
                    default:
                        return false;
                }
            }
        });
        panel.addView(mStartBtn, matchWrap());

        mStopBtn = new Button(this);
        mStopBtn.setText("Stop tuner (after current trial)");
        mStopBtn.setAllCaps(false);
        setButtonIcon(mStopBtn, R.drawable.ic_btn_end, 0xFF212121);
        mStopBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mTuner != null) {
                    mTuner.stop();
                    setTunerStatus("Stopping after the current trial…");
                }
            }
        });
        panel.addView(mStopBtn, matchWrap());

        mTunerStatus = new TextView(this);
        mTunerStatus.setTextSize(TypedValue.COMPLEX_UNIT_SP, 11);
        mTunerStatus.setTypeface(Typeface.MONOSPACE);
        mTunerStatus.setTextColor(0xFF202428);
        mTunerStatus.setText("Idle");
        panel.addView(mTunerStatus, matchWrap());

        // Save / Load / Defaults (NVS; idempotent, the client retransmits every 500 ms)
        // Save / Load / Defaults (NVS; idempotent, client gửi lại mỗi 500 ms)
        LinearLayout nvRow = new LinearLayout(this);
        nvRow.setOrientation(LinearLayout.HORIZONTAL);
        nvRow.addView(nvsButton("Save", FlightControlPacket.CMD_SAVE));
        nvRow.addView(nvsButton("Load", FlightControlPacket.CMD_LOAD));
        nvRow.addView(nvsButton("Defaults", FlightControlPacket.CMD_DEFAULTS));
        panel.addView(nvRow, matchWrap());

        // Bench motor test (PROTOCOL.md §5) — disarmed only, watchdog-held by re-sends.
        // Thử động cơ trên bàn (PROTOCOL.md §5) — chỉ khi chưa armed, giữ bằng watchdog nhờ gửi lại.
        LinearLayout motorLpWrap = buildMotorTestPanel();
        LinearLayout.LayoutParams motorLp = matchWrap();
        motorLp.topMargin = dp(12);
        panel.addView(motorLpWrap, motorLp);

        // Parameter editors
        panel.addView(sectionLabel("Parameters"));
        for (ParamSpec spec : ParamSpec.ALL) {
            ParamEditor editor = new ParamEditor(this, spec, new ParamEditor.OnCommitListener() {
                @Override
                public void onCommit(ParamSpec spec, double value) {
                    if (mTuner != null && mTuner.isRunning()) {
                        Toast.makeText(TuneActivity.this,
                                "Auto-tune is running - it owns the gains right now", Toast.LENGTH_SHORT).show();
                        return;
                    }
                    if (mParam != null) {
                        mParam.setValue(spec.completeName, spec.toWire(value));
                    }
                }
            });
            mEditors.add(editor);
            LinearLayout.LayoutParams lp = matchWrap();
            lp.topMargin = dp(8);
            panel.addView(editor, lp);
        }
        return panel;
    }

    private Button nvsButton(String label, final int cmd) {
        Button b = new Button(this);
        b.setText(label);
        b.setAllCaps(false);
        if (cmd == FlightControlPacket.CMD_SAVE) {
            setButtonIcon(b, R.drawable.ic_btn_save, 0xFF212121);
        } else if (cmd == FlightControlPacket.CMD_LOAD) {
            setButtonIcon(b, R.drawable.ic_btn_open, 0xFF212121);
        }
        b.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                switch (cmd) {
                    case FlightControlPacket.CMD_SAVE:
                        sendFlightControl(FlightControlPacket.save());
                        mStatusLine.setText("Saved gains to flash.");
                        break;
                    case FlightControlPacket.CMD_LOAD:
                        sendFlightControl(FlightControlPacket.load());
                        mStatusLine.setText("Loading gains from flash...");
                        // The firmware swaps the gains; without a re-read the editors keep
                        // showing the old values and Load looks like it did nothing (the
                        // iOS client re-reads here - see PidTuneViewController.loadGains).
                        // Firmware thay đổi các hệ số; không đọc lại thì các ô vẫn hiện giá
                        // trị cũ và Load trông như không làm gì (client iOS có đọc lại -
                        // xem PidTuneViewController.loadGains).
                        rereadParamsAfterNvs();
                        break;
                    default:
                        sendFlightControl(FlightControlPacket.defaults());
                        mStatusLine.setText("Restoring firmware defaults...");
                        rereadParamsAfterNvs();
                        break;
                }
            }
        });
        LinearLayout.LayoutParams lp = matchWrap();
        lp.rightMargin = dp(6);
        return b;
    }

    private final Runnable mHoldFire = new Runnable() {
        @Override
        public void run() {
            onStartConfirmed();
        }
    };

    private final Runnable mHoldTick = new Runnable() {
        @Override
        public void run() {
            long held = SystemClock.elapsedRealtime() - mHoldStart;
            if (held < HOLD_TO_START_MS) {
                mStartBtn.setText(String.format(Locale.US, "Keep holding… %.1f s",
                        (HOLD_TO_START_MS - held) / 1000.0));
                mStartBtn.postDelayed(this, 100);
            }
        }
    };

    private void resetStartButton() {
        mStartBtn.setText("Hold 2 s to start - props spinning; secure the drone on a stand or hold it tethered");
    }

    // ------------------------------------------------------------- auto-tune control

    /** The hold-to-confirm completed; now the armed gate decides. */
    /** Việc giữ nút để xác nhận đã hoàn tất; giờ cổng kiểm tra armed quyết định. */
    private void onStartConfirmed() {
        resetStartButton();
        if (mTuner != null && mTuner.isRunning()) {
            setTunerStatus("Already running - one axis at a time.");
            return;
        }
        if (mMotorTestRunning) {
            // The test holds the outputs with the PID bypassed; arming mid-test would
            // cut it and hand control to the flight loop, so make the user stop it first.
            // Thử đang giữ output với PID bị bỏ qua; armed giữa chừng sẽ cắt thử và chuyển
            // quyền cho vòng bay, nên yêu cầu người dùng dừng trước.
            setTunerStatus("Stop the bench motor test before auto-tuning.");
            Toast.makeText(this, "Stop the bench motor test first", Toast.LENGTH_SHORT).show();
            return;
        }
        if (!mArmed) {
            new AlertDialog.Builder(this)
                    .setTitle("Drone not armed")
                    .setMessage("Auto-tune spins the props. The drone reports flight.armed = 0.\n\n"
                            + "Arm it now? Props will spin as soon as the first trial starts.")
                    .setPositiveButton("Arm", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            sendFlightControl(FlightControlPacket.arm());
                            mPendingStart = true;
                            setTunerStatus("Waiting for the drone to arm…");
                        }
                    })
                    .setNegativeButton("Cancel", null)
                    .show();
            return;
        }
        startTuner();
    }

    private void startTuner() {
        int trials = parseInt(mTrialsEdit, 40, 2, 200);
        double trialS = parseDouble(mTrialSecondsEdit, 6, 2, 30);
        AutoTuner.Method method = AutoTuner.Method.values()[mMethodSpinner.getSelectedItemPosition()];
        List<String> ids = new ArrayList<>();
        ids.add("pid." + mAxis + "_kp");
        ids.add("pid." + mAxis + "_ki");
        ids.add("pid." + mAxis + "_kd");
        AutoTuner.Config cfg = new AutoTuner.Config(method, ids, trials, trialS,
                0 /* no nudge: the drone sits on its stand */, method.defaultRange,
                /* không nudge: drone đặt trên giá đỡ */
                false /* no balance-angle trim */, false /* no cross-run history */);
                /* không chỉnh góc balance */
                /* không dùng lịch sử giữa các lần chạy */
        // One trial log per axis: rows from a roll run seed the next roll run and are
        // never matched against a pitch one.
        // Một file log lần thử cho mỗi trục: các hàng từ lần chạy roll khởi tạo cho
        // lần chạy roll kế tiếp và không bao giờ bị ghép với dữ liệu của pitch.
        mTuner = new AutoTuner(cfg, mParam,
                new java.io.File(getFilesDir(), "pid-tune-" + mAxis + ".csv"), mTunerListener);
        mTuner.start();
        mAxisSpinner.setEnabled(false);
        mMethodSpinner.setEnabled(false);
        mTrialsEdit.setEnabled(false);
        mTrialSecondsEdit.setEnabled(false);
        setTunerStatus("Starting: tuning " + mAxis + " with " + method.label);
    }

    private final AutoTuner.Listener mTunerListener = new AutoTuner.Listener() {
        @Override
        public void onStatus(String text) {
            if (!isFinishingOrDestroyed()) {
                setTunerStatus(text);
            }
        }

        @Override
        public void onTrial(int trialNo, float[] candidates, double cost) {
            if (isFinishingOrDestroyed()) {
                return;
            }
            StringBuilder sb = new StringBuilder();
            sb.append(String.format(Locale.US, "Trial %d - cost %.2f\n", trialNo, cost));
            if (candidates != null && candidates.length >= 3) {
                sb.append(String.format(Locale.US, "kp=%.4g  ki=%.4g  kd=%.4g",
                        candidates[0], candidates[1], candidates[2]));
            }
            mTunerStatus.setText(sb.toString());
        }

        @Override
        public void onModel(AutoTuner.Report report) {
            if (isFinishingOrDestroyed() || report == null || report.bestPred == null) {
                return;
            }
            // ModelView is not built yet; surface what the model believes as text.
            // ModelView chưa được xây dựng; hiển thị điều mô hình dự đoán dưới dạng văn bản.
            setTunerStatus(mTunerStatus.getText() + String.format(Locale.US,
                    "\nbest predicted cost %.2f", report.bestPred));
        }

        @Override
        public void onDone(AutoTuner.Report report) {
            mTuner = null;
            if (isFinishingOrDestroyed()) {
                return;
            }
            mAxisSpinner.setEnabled(true);
            mMethodSpinner.setEnabled(true);
            mTrialsEdit.setEnabled(true);
            mTrialSecondsEdit.setEnabled(true);
            StringBuilder sb = new StringBuilder();
            sb.append("Done (").append(report.reason).append(") after ")
                    .append(report.trialsRun).append(" trials\n");
            if (report.bestGains != null && report.bestGains.length >= 3) {
                sb.append(String.format(Locale.US, "best cost %.2f\nkp=%.4g  ki=%.4g  kd=%.4g",
                        report.bestCost, report.bestGains[0], report.bestGains[1], report.bestGains[2]));
            }
            setTunerStatus(sb.toString());
        }
    };

    private void panic() {
        mPendingStart = false;
        if (mTuner != null) {
            mTuner.stop();
        }
        // Halting the tuner is not enough: cut the motors now. EMERGENCY is always honoured.
        // Dừng bộ tinh chỉnh là chưa đủ: cắt động cơ ngay. EMERGENCY luôn được chấp hành.
        sendFlightControl(FlightControlPacket.emergency());
        setTunerStatus("CUT MOTORS sent - motors off, tuner halted.");
    }

    private void sendFlightControl(CrtpPacket packet) {
        if (mCrazyflie != null) {
            mCrazyflie.sendPacket(packet);
        }
    }

    /**
     * Re-read every parameter after a LOAD/DEFAULTS so the editors show what the firmware
     * actually holds. Delayed because the command rides the send path while param reads
     * ride the param-updater queue thread; a read overtaking LOAD would show stale values.
     * Đọc lại mọi tham số sau LOAD/DEFAULTS để các ô hiện đúng giá trị firmware đang giữ.
     * Trì hoãn vì lệnh đi theo đường gửi còn các lần đọc đi theo luồng hàng đợi tham số;
     * một lần đọc vượt lên trước LOAD sẽ hiện giá trị cũ.
     */
    private void rereadParamsAfterNvs() {
        mMain.removeCallbacks(mNvsReread);
        mMain.postDelayed(mNvsReread, NVS_REREAD_DELAY_MS);
    }

    private static final int NVS_REREAD_DELAY_MS = 300;

    private final Runnable mNvsReread = new Runnable() {
        @Override
        public void run() {
            if (mParam != null && !isFinishingOrDestroyed()) {
                mParam.requestUpdateOfAllParams();
            }
        }
    };

    private void setTunerStatus(String text) {
        mTunerStatus.setText(text);
    }

    private boolean isFinishingOrDestroyed() {
        return isFinishing() || isDestroyed();
    }

    // ------------------------------------------------------------- logging

    private void setupLogging() {
        mLogAdapter = new LogAdapter() {
            @Override
            public void logDataReceived(LogConfig logConfig, Map<String, Number> data, int timestamp) {
                // This listener sees every block on the link (the main screen's battery
                // block too); only ours are handled here.
                // Listener này thấy mọi khối trên liên kết (cả khối pin của màn hình chính);
                // chỉ các khối của chúng ta được xử lý tại đây.
                if (STATUS_LOG_NAME.equals(logConfig.getName())) {
                    handleStatusData(data);
                } else if (logConfig == mTuneLogConfig) {
                    handleTuneData(data);
                }
            }
        };
        mLogg.addLogListener(mLogAdapter);

        mStatusLogConfig = new LogConfig(STATUS_LOG_NAME, TUNE_PERIOD_MS);
        mStatusLogConfig.addVariable("pm.vbat", VariableType.FLOAT);
        mStatusLogConfig.addVariable("flight.mode", VariableType.UINT8_T);
        mStatusLogConfig.addVariable("flight.armed", VariableType.UINT8_T);
        mStatusLogConfig.addVariable("flight.auto_state", VariableType.UINT8_T);
        mStatusLogConfig.addVariable("stabilizer.roll", VariableType.FLOAT);
        mStatusLogConfig.addVariable("stabilizer.pitch", VariableType.FLOAT);
        startLogConfig(mStatusLogConfig);
        switchTuneBlock(mAxis);
    }

    /**
     * Logg's own config-id counter never increments (every addConfig() would hand out id
     * 1 and the blocks would collide on the wire), so the id is pinned by hand here, above
     * anything already registered (e.g. the main screen's "Standard" block).
     * Bộ đếm config-id của chính Logg không bao giờ tăng (mọi addConfig() sẽ phát ra id
     * 1 và các khối sẽ đụng nhau trên đường truyền), nên id được ghim thủ công tại đây, cao
     * hơn mọi thứ đã đăng ký (ví dụ khối "Standard" của màn hình chính).
     */
    private void startLogConfig(LogConfig config) {
        mLogg.addConfig(config);
        int id = 1;
        for (LogConfig other : mLogg.getLogConfigs()) {
            if (other != config && other.getId() >= id) {
                id = other.getId() + 1;
            }
        }
        config.setId(id);
        mLogg.start(config);
    }

    private void switchTuneBlock(String axis) {
        if (mTuneLogConfig != null) {
            mLogg.stop(mTuneLogConfig);
            mLogg.delete(mTuneLogConfig);
            mTuneLogConfig = null;
        }
        mAxis = axis;
        mPrevAngleValid = false;
        mAngleRing.clear();
        mRateRing.clear();
        mMotorRing.clear();
        mControlRing.clear();

        LogConfig cfg = new LogConfig("Tune" + capitalize(axis), TUNE_PERIOD_MS);
        cfg.addVariable("pid." + axis + "_p", VariableType.FLOAT);
        cfg.addVariable("pid." + axis + "_i", VariableType.FLOAT);
        cfg.addVariable("pid." + axis + "_d", VariableType.FLOAT);
        cfg.addVariable("stabilizer." + axis, VariableType.FLOAT);
        cfg.addVariable("controller." + axis, VariableType.FLOAT);
        // motor.m1 rides along on every axis block (22 B of the 25 B budget): the cost
        // function needs the motor RMS and the saturation share no matter which axis is
        // being tuned, and the brief's yaw block left it out.
        // motor.m1 đi kèm mọi khối theo trục (22 B trong ngân sách 25 B): hàm chi phí
        // cần motor RMS và tỷ lệ bão hòa bất kể trục nào đang được tinh chỉnh, còn khối
        // yaw trong đặc tả đã bỏ sót nó.
        cfg.addVariable("motor.m1", VariableType.UINT16_T);
        mTuneLogConfig = cfg;
        startLogConfig(cfg);
        showAxisCharts(axisIndex(axis));
    }

    private void handleStatusData(Map<String, Number> data) {
        float vbat = fval(data, "pm.vbat");
        int mode = (int) fval(data, "flight.mode");
        int armed = (int) fval(data, "flight.armed");
        int autoState = (int) fval(data, "flight.auto_state");
        mLastRoll = fval(data, "stabilizer.roll");
        mLastPitch = fval(data, "stabilizer.pitch");
        mLastVbat = vbat;
        mLastMode = mode;
        mLastAutoState = autoState;
        boolean wasArmed = mArmed;
        mArmed = armed == 1;
        final String line = String.format(Locale.US,
                "armed %s   mode %s   auto %d   vbat %.2f V   tune: %s",
                mArmed ? "yes" : "NO",
                (mode >= 0 && mode < MODE_NAMES.length) ? MODE_NAMES[mode] : "?",
                autoState, vbat, mAxis);
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (!isFinishingOrDestroyed()) {
                    mStatusLine.setText(line);
                }
            }
        });
        if (mPendingStart && mArmed && !wasArmed) {
            mPendingStart = false;
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    if (!isFinishingOrDestroyed()) {
                        startTuner();
                    }
                }
            });
        }
    }

    private void handleTuneData(Map<String, Number> data) {
        final int idx = axisIndex(mAxis);
        float angle = fval(data, "stabilizer." + mAxis);
        float ctrl = fval(data, "controller." + mAxis);
        float p = fval(data, "pid." + mAxis + "_p");
        float i = fval(data, "pid." + mAxis + "_i");
        float d = fval(data, "pid." + mAxis + "_d");
        float m1 = fval(data, "motor.m1");

        // The drone logs no roll/pitch gyro rates (only stabilizer.yaw is a rate), so the
        // rate is finite-differenced from consecutive angle samples at the 20 Hz sample
        // rate. A deliberate, honest substitution - not a bug.
        // Drone không log tốc độ góc roll/pitch từ gyro (chỉ stabilizer.yaw là một tốc độ
        // góc), nên tốc độ góc được sai phân hữu hạn từ các mẫu góc liên tiếp ở tần số lấy
        // mẫu 20 Hz. Một sự thay thế có chủ đích và trung thực - không phải lỗi.
        float rate;
        if ("yaw".equals(mAxis)) {
            rate = angle;
        } else {
            rate = mPrevAngleValid ? (angle - mPrevAngle) * (1000f / TUNE_PERIOD_MS) : 0f;
        }
        mPrevAngle = angle;
        mPrevAngleValid = true;

        mAngleRing.add(angle);
        mRateRing.add(rate);
        mMotorRing.add(m1);
        mControlRing.add(ctrl);
        if (mTuner != null) {
            mTuner.onTelemetry(new AutoTuner.Telemetry(angle, rate, ctrl, m1, mLastRoll, mLastPitch));
        }

        final float[] angleSample = {angle, ctrl};
        final float[] pidSample = {p, i, d};
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (isFinishingOrDestroyed()) {
                    return;
                }
                mAngleCharts[idx].add(angleSample);
                mPidCharts[idx].add(pidSample);
            }
        });
    }

    private static float fval(Map<String, Number> data, String name) {
        Number n = data.get(name);
        return n != null ? n.floatValue() : 0f;
    }

    // ------------------------------------------------------------- parameters

    private void setupParamListeners() {
        for (final ParamSpec spec : ParamSpec.ALL) {
            String[] parts = spec.completeName.split("\\.", 2);
            ParamListener listener = new ParamListener(parts[0], parts[1]) {
                @Override
                public void updated(String name, Number value) {
                    // Echo for the tuner first (it polls on its own thread), then the UI.
                    // Echo cho bộ tinh chỉnh trước (nó poll trên luồng riêng), sau đó là UI.
                    if (mTuner != null) {
                        mTuner.onParam(name, value.floatValue());
                    }
                    final Number v = value;
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            if (isFinishingOrDestroyed()) {
                                return;
                            }
                            for (ParamEditor editor : mEditors) {
                                if (editor.getSpec().completeName.equals(spec.completeName)) {
                                    editor.onParamValue(v);
                                    break;
                                }
                            }
                        }
                    });
                }
            };
            // Param keeps only ONE listener per complete name, so the activity owns a
            // single dispatcher per parameter and fans out from there.
            // Param chỉ giữ MỘT listener cho mỗi complete name, nên activity sở hữu một
            // bộ phân phối duy nhất cho từng tham số rồi lan ra từ đó.
            mParamListeners.add(listener);
            mParam.addParamListener(listener);
            mParam.requestParamUpdate(spec.completeName);
        }
    }

    // ------------------------------------------------------------- statistics

    private final Runnable mStatsTick = new Runnable() {
        @Override
        public void run() {
            updateStats();
            mMain.postDelayed(this, STATS_PERIOD_MS);
        }
    };

    private void updateStats() {
        if (isFinishingOrDestroyed()) {
            return;
        }
        float[] buf = new float[ChartView.CAPACITY];
        StringBuilder sb = new StringBuilder();
        for (int a = 0; a < AXES.length; a++) {
            appendChartStats(sb, mAngleCharts[a], buf);
            appendChartStats(sb, mPidCharts[a], buf);
        }

        // Cost components of the active axis over the same visible window.
        // Các thành phần chi phí của trục đang hoạt động trên cùng cửa sổ hiển thị.
        float[] angle = new float[ChartView.CAPACITY];
        float[] rate = new float[ChartView.CAPACITY];
        float[] motor = new float[ChartView.CAPACITY];
        float[] control = new float[ChartView.CAPACITY];
        int n = mAngleRing.copyTo(angle);
        int nRate = mRateRing.copyTo(rate);
        int nMotor = mMotorRing.copyTo(motor);
        int nControl = mControlRing.copyTo(control);
        int nCost = Math.min(n, Math.min(nRate, Math.min(nMotor, nControl)));
        if (nCost > 0) {
            AutoTuner.Metrics m = AutoTuner.costOf(angle, rate, motor, control, nCost);
            sb.append(String.format(Locale.US,
                    "\nCost (%s): angle sd %.3f + 0.1x rate rms %.3f + 0.2x motor rms %.3f\n"
                            + "        + 0.005x ctrl %.3f + 20x sat %.3f  =  %.3f",
                    mAxis, m.angleStd, m.rateRms, m.motorRms, m.controlEffort, m.saturatedShare, m.cost));
        } else {
            sb.append("\nCost (").append(mAxis).append("): waiting for telemetry");
        }
        mStatsView.setText(sb.toString());
    }

    private void appendChartStats(StringBuilder sb, ChartView chart, float[] buf) {
        List<ChartView.Series> series = chart.getSeries();
        for (int s = 0; s < series.size(); s++) {
            int n = chart.copySamples(s, buf);
            if (n <= 0) {
                continue;
            }
            Stats st = Stats.of(buf, n);
            sb.append(String.format(Locale.US, "%-14s min %8.2f  max %8.2f  mean %8.2f  sd %7.2f  rms %7.2f\n",
                    series.get(s).name, st.min, st.max, st.mean, st.stdDev, st.rms));
        }
    }

    // ------------------------------------------------------------- helpers

    private static int axisIndex(String axis) {
        for (int i = 0; i < AXES.length; i++) {
            if (AXES[i].equals(axis)) {
                return i;
            }
        }
        return 0;
    }

    private static String capitalize(String s) {
        return Character.toUpperCase(s.charAt(0)) + s.substring(1);
    }

    private TextView sectionLabel(String text) {
        TextView tv = new TextView(this);
        tv.setText(text);
        tv.setTextSize(TypedValue.COMPLEX_UNIT_SP, 12);
        tv.setTypeface(Typeface.DEFAULT_BOLD);
        tv.setTextColor(0xFF202428);
        LinearLayout.LayoutParams lp = matchWrap();
        lp.topMargin = dp(10);
        tv.setLayoutParams(lp);
        return tv;
    }

    private TextView caption(String text) {
        TextView tv = new TextView(this);
        tv.setText(text);
        tv.setTextSize(TypedValue.COMPLEX_UNIT_SP, 12);
        tv.setTextColor(0xFF3A4048);
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        lp.rightMargin = dp(6);
        tv.setLayoutParams(lp);
        return tv;
    }

    private EditText numberField(String initial) {
        EditText et = new EditText(this);
        et.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL);
        et.setSingleLine(true);
        et.setText(initial);
        et.setTextSize(TypedValue.COMPLEX_UNIT_SP, 13);
        return et;
    }

    private LinearLayout.LayoutParams spinnerParams() {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        lp.leftMargin = dp(4);
        return lp;
    }

    private LinearLayout.LayoutParams smallFieldParams() {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        lp.leftMargin = dp(4);
        lp.rightMargin = dp(8);
        return lp;
    }

    private LinearLayout.LayoutParams matchWrap() {
        return new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
    }

    // ------------------------------------------------------------- bench motor test

    /** Slider i maps 0..1000 onto a 1000..2000 µs thrust; read on the UI thread. */
    /** Slider i đổi 0..1000 thành lực đẩy 1000..2000 µs; đọc trên UI thread. */
    private int motorThrust(int i) {
        SeekBar bar = mMotorSliders[i];
        int offset = (bar == null) ? 0 : bar.getProgress();
        return MOTOR_THRUST_FLOOR + Math.max(0, Math.min(MOTOR_THRUST_RANGE, offset));
    }

    /** Four thrust sliders, a guarded start button and a live status line. */
    /** Bốn thanh trượt lực đẩy, nút bắt đầu có kiểm tra và dòng trạng thái trực tiếp. */
    private LinearLayout buildMotorTestPanel() {
        LinearLayout panel = new LinearLayout(this);
        panel.setOrientation(LinearLayout.VERTICAL);
        panel.addView(sectionLabel("Bench motor test"));

        TextView note = new TextView(this);
        note.setText("Props off or restrained. Spins m1..m4 directly with the PID bypassed, "
                + "disarmed only. The link re-sends every 200 ms — closing this screen or "
                + "losing the link stops the motors.");
        note.setTextSize(TypedValue.COMPLEX_UNIT_SP, 11);
        note.setTextColor(0xFF606870);
        panel.addView(note, matchWrap());

        String[] labels = {"M1", "M2", "M3", "M4"};
        for (int i = 0; i < 4; i++) {
            LinearLayout row = new LinearLayout(this);
            row.setOrientation(LinearLayout.HORIZONTAL);
            row.setGravity(Gravity.CENTER_VERTICAL);

            TextView label = new TextView(this);
            label.setText(labels[i]);
            label.setTypeface(Typeface.MONOSPACE);
            label.setWidth(dp(32));
            row.addView(label);

            SeekBar bar = new SeekBar(this);
            bar.setMax(MOTOR_THRUST_RANGE);
            bar.setProgress(0);
            final int idx = i;
            bar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override
                public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                    mMotorValues[idx].setText(motorThrust(idx) + " µs");
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) { }
                @Override public void onStopTrackingTouch(SeekBar seekBar) { }
            });
            mMotorSliders[i] = bar;
            row.addView(bar, matchWrap());

            TextView value = new TextView(this);
            value.setTypeface(Typeface.MONOSPACE);
            value.setWidth(dp(64));
            value.setGravity(Gravity.END);
            value.setText(motorThrust(i) + " µs");
            mMotorValues[i] = value;
            row.addView(value);

            LinearLayout.LayoutParams rowLp = matchWrap();
            rowLp.topMargin = dp(4);
            panel.addView(row, rowLp);
        }

        mMotorStartBtn = new Button(this);
        mMotorStartBtn.setText("Start test");
        mMotorStartBtn.setAllCaps(false);
        mMotorStartBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                toggleMotorTest();
            }
        });
        LinearLayout.LayoutParams btnLp = matchWrap();
        btnLp.topMargin = dp(8);
        panel.addView(mMotorStartBtn, btnLp);

        mMotorStatus = new TextView(this);
        mMotorStatus.setTextSize(TypedValue.COMPLEX_UNIT_SP, 11);
        mMotorStatus.setTypeface(Typeface.MONOSPACE);
        mMotorStatus.setTextColor(0xFF202428);
        mMotorStatus.setText("Idle");
        panel.addView(mMotorStatus, matchWrap());

        return panel;
    }

    /** Start (refused while armed) or stop the test. Stopping sends enable=0 once. */
    /** Bắt đầu (từ chối khi đang armed) hoặc dừng thử. Dừng gửi enable=0 một lần. */
    private void toggleMotorTest() {
        if (mMotorTestRunning) {
            stopMotorTest("Stopped.");
            return;
        }
        if (mArmed) {
            Toast.makeText(this, "Drone is armed - disarm before a bench motor test",
                    Toast.LENGTH_SHORT).show();
            mMotorStatus.setText("Refused: armed. Disarm first.");
            return;
        }
        mMotorTestRunning = true;
        mMotorStartBtn.setText("Stop test");
        mMotorStatus.setText("Running — re-sending every " + MOTOR_TEST_PERIOD_MS + " ms");
        mMotorTestTick.run();
    }

    /** Halt the re-send loop and tell the firmware to stop (enable=0). */
    /** Dừng vòng gửi lại và báo firmware dừng (enable=0). */
    private void stopMotorTest(String reason) {
        boolean wasRunning = mMotorTestRunning;
        mMotorTestRunning = false;
        mMain.removeCallbacks(mMotorTestTick);
        if (wasRunning) {
            sendFlightControl(FlightControlPacket.motorTestStop());
        }
        if (mMotorStartBtn != null) {
            mMotorStartBtn.setText("Start test");
        }
        if (mMotorStatus != null) {
            mMotorStatus.setText(reason);
        }
    }

    /** Put an icon at the start of a text button, tinted to match its text. */
    /** Đặt icon ở đầu nút có chữ, tô màu khớp với màu chữ của nút. */
    private void setButtonIcon(Button b, int resId, int tint) {
        Drawable d = ContextCompat.getDrawable(this, resId);
        if (d == null) {
            return;
        }
        d = d.mutate();
        d.setTint(tint);
        int px = dp(18);
        d.setBounds(0, 0, px, px);
        b.setCompoundDrawablesRelative(d, null, null, null);
        b.setCompoundDrawablePadding(dp(6));
    }

    /** Icon-only button: the label lives in the content description and a long-press tooltip. */
    /** Nút chỉ có icon: nhãn nằm trong content description và tooltip khi nhấn giữ. */
    private ImageButton iconButton(int resId, String label) {
        ImageButton b = new ImageButton(this);
        b.setImageResource(resId);
        b.setContentDescription(label);
        TooltipCompat.setTooltipText(b, label);
        return b;
    }

    private int dp(int v) {
        return (int) (v * getResources().getDisplayMetrics().density + 0.5f);
    }

    private int parseInt(EditText et, int fallback, int min, int max) {
        try {
            int v = Integer.parseInt(et.getText().toString().trim());
            return Math.max(min, Math.min(max, v));
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private double parseDouble(EditText et, double fallback, double min, double max) {
        try {
            double v = Double.parseDouble(et.getText().toString().trim());
            return Math.max(min, Math.min(max, v));
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    // ------------------------------------------------------------- lifecycle

    @Override
    protected void onResume() {
        super.onResume();
        if (mHeader != null) {
            mHeader.start();
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mHeader != null) {
            mHeader.stop();
        }
        // Never leave a bench test re-sending while we are in the background.
        // Không bao giờ để thử trên bàn tiếp tục gửi lại khi app ở nền.
        stopMotorTest("Stopped (screen backgrounded).");
    }

    @Override
    protected void onDestroy() {
        stopMotorTest("Stopped.");
        mMain.removeCallbacks(mStatsTick);
        mMain.removeCallbacks(mTocWait);
        mMain.removeCallbacks(mNvsReread);
        // The activity can close while the TOC is still loading, before buildUi() has
        // run; the button-backed callbacks do not exist yet then.
        // Activity có thể đóng trong lúc TOC còn đang tải, trước khi buildUi() chạy;
        // khi đó các callback gắn vào nút còn chưa tồn tại.
        if (mStartBtn != null) {
            mStartBtn.removeCallbacks(mHoldFire);
            mStartBtn.removeCallbacks(mHoldTick);
        }
        if (mTuner != null) {
            if (mTuner.isRunning()) {
                // Leaving a self-tuning, spinning drone unattended is not an option.
                // Không thể bỏ mặc drone đang tự tinh chỉnh với cánh quạt đang quay.
                Log.w(LOG_TAG, "auto-tune still running at exit - emergency stop");
                mTuner.stop();
                sendFlightControl(FlightControlPacket.emergency());
            }
            mTuner = null;
        }
        if (mLogg != null) {
            if (mTuneLogConfig != null) {
                mLogg.stop(mTuneLogConfig);
                mLogg.delete(mTuneLogConfig);
                mTuneLogConfig = null;
            }
            if (mStatusLogConfig != null) {
                mLogg.stop(mStatusLogConfig);
                mLogg.delete(mStatusLogConfig);
                mStatusLogConfig = null;
            }
            if (mLogAdapter != null) {
                mLogg.removeLogListener(mLogAdapter);
            }
        }
        if (mParam != null) {
            for (ParamListener listener : mParamListeners) {
                mParam.removeParamListeners(listener.getGroup(), listener.getName());
            }
            mParamListeners.clear();
        }
        super.onDestroy();
    }
}
