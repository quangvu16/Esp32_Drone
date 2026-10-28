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
 *
 * <p>Telemetry comes from two log blocks at 20 Hz (PROTOCOL.md section 4): a Status block
 * that is always running and an axis tune block that follows whichever axis is selected.
 * The charts of a non-selected axis therefore show "Waiting for telemetry" until their
 * block is streaming.
 *
 * <p>The connection is shared through {@link DroneSession}; nothing here owns the link.
 */
public class TuneActivity extends AppCompatActivity {

    private static final String LOG_TAG = "Crazyflie-TuneActivity";

    private static final int TUNE_PERIOD_MS = 50;          // 20 Hz charts
    private static final int STATS_PERIOD_MS = 500;
    private static final int HOLD_TO_START_MS = 2000;
    private static final String STATUS_LOG_NAME = "TuneStatus";

    private static final String[] AXES = {"roll", "pitch", "yaw"};
    private static final String[] AXIS_UNITS = {" deg", " deg", " deg/s"};
    private static final String[] MODE_NAMES = {"MANUAL", "HOLD", "AUTO", "FOLLOW", "RTH"};

    // Colour-blind tolerant series colours (hard-coded on purpose: no res/values edits).
    private static final int COLOR_BLUE = 0xFF2196F3;
    private static final int COLOR_RED = 0xFFF44336;
    private static final int COLOR_GREEN = 0xFF4CAF50;
    private static final int COLOR_ORANGE = 0xFFFF9800;
    private static final int COLOR_PURPLE = 0xFF9C27B0;

    /** Public entry point; no extras needed, the link comes from {@link DroneSession}. */
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
    private final Stats.Ring mAngleRing = new Stats.Ring(ChartView.CAPACITY);
    private final Stats.Ring mRateRing = new Stats.Ring(ChartView.CAPACITY);
    private final Stats.Ring mMotorRing = new Stats.Ring(ChartView.CAPACITY);
    private final Stats.Ring mControlRing = new Stats.Ring(ChartView.CAPACITY);

    private final Handler mMain = new Handler(Looper.getMainLooper());
    private long mHoldStart;

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
        if (mParam == null || mLogg == null || mLogg.getToc() == null) {
            Toast.makeText(this, "Drone TOCs not ready yet - try again in a moment", Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        setContentView(buildUi());
        setupParamListeners();
        setupLogging();
        mMain.post(mStatsTick);
    }

    // ------------------------------------------------------------- UI construction

    private View buildUi() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(Color.parseColor("#EFF1F3"));

        root.addView(buildTopBar(), new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

        LinearLayout body = new LinearLayout(this);
        body.setOrientation(LinearLayout.HORIZONTAL);
        body.addView(buildChartsColumn(), new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.MATCH_PARENT, 1f));
        ScrollView panelScroll = new ScrollView(this);
        panelScroll.setFillViewport(true);
        panelScroll.addView(buildPanel(), new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));
        LinearLayout.LayoutParams panelLp = new LinearLayout.LayoutParams(
                dp(370), LinearLayout.LayoutParams.MATCH_PARENT);
        panelLp.leftMargin = dp(4);
        body.addView(panelScroll, panelLp);

        LinearLayout.LayoutParams bodyLp = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f);
        root.addView(body, bodyLp);
        return root;
    }

    /** Shared header (sections, status, CUT MOTORS) with the tuner's status line under it. */
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

    private View buildChartsColumn() {
        LinearLayout col = new LinearLayout(this);
        col.setOrientation(LinearLayout.VERTICAL);
        col.setPadding(dp(6), dp(4), dp(2), dp(12));
        for (int a = 0; a < AXES.length; a++) {
            String axis = AXES[a];
            col.addView(sectionLabel(capitalize(axis) + " - angle vs controller output"));
            mAngleCharts[a] = buildAngleChart(axis);
            col.addView(mAngleCharts[a], matchWrap());

            col.addView(sectionLabel(capitalize(axis) + " - PID terms (p / i / d)"));
            mPidCharts[a] = buildPidChart(axis);
            col.addView(mPidCharts[a], matchWrap());
        }
        return col;
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

        // Pause / clear
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        // Chart tools are routine and reversible, so they are icon-only (with a
        // content description + long-press tooltip). Flight commands keep their text.
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
        row.addView(mPauseBtn, new LinearLayout.LayoutParams(0, dp(48), 1f));
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
        LinearLayout.LayoutParams clearLp = new LinearLayout.LayoutParams(0, dp(48), 1f);
        clearLp.leftMargin = dp(6);
        row.addView(clear, clearLp);
        panel.addView(row, matchWrap());

        // Statistics
        panel.addView(sectionLabel("Statistics (visible window)"));
        mStatsView = new TextView(this);
        mStatsView.setTextSize(TypedValue.COMPLEX_UNIT_SP, 10);
        mStatsView.setTypeface(Typeface.MONOSPACE);
        mStatsView.setTextColor(0xFF202428);
        mStatsView.setText("Waiting for telemetry");
        panel.addView(mStatsView, matchWrap());

        // Auto-tune
        panel.addView(sectionLabel("Auto-tune (one axis at a time)"));

        LinearLayout axisRow = new LinearLayout(this);
        axisRow.setOrientation(LinearLayout.HORIZONTAL);
        axisRow.setGravity(Gravity.CENTER_VERTICAL);
        axisRow.addView(caption("Axis"));
        mAxisSpinner = new Spinner(this);
        ArrayAdapter<String> axisAdapter = new ArrayAdapter<>(this,
                android.R.layout.simple_spinner_item,
                new String[] {"Roll", "Pitch", "Yaw"});
        axisAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        mAxisSpinner.setAdapter(axisAdapter);
        axisRow.addView(mAxisSpinner, spinnerParams());
        panel.addView(axisRow, matchWrap());
        mAxisSpinner.setOnItemSelectedListener(new android.widget.AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(android.widget.AdapterView<?> parent, View view, int position, long id) {
                String axis = AXES[position];
                if (axis.equals(mAxis)) {
                    return;
                }
                if (mTuner != null && mTuner.isRunning()) {
                    // one axis at a time: switching mid-run is blocked
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
        LinearLayout nvRow = new LinearLayout(this);
        nvRow.setOrientation(LinearLayout.HORIZONTAL);
        nvRow.addView(nvsButton("Save", FlightControlPacket.CMD_SAVE));
        nvRow.addView(nvsButton("Load", FlightControlPacket.CMD_LOAD));
        nvRow.addView(nvsButton("Defaults", FlightControlPacket.CMD_DEFAULTS));
        panel.addView(nvRow, matchWrap());

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
                        break;
                    case FlightControlPacket.CMD_LOAD:
                        sendFlightControl(FlightControlPacket.load());
                        break;
                    default:
                        sendFlightControl(FlightControlPacket.defaults());
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
    private void onStartConfirmed() {
        resetStartButton();
        if (mTuner != null && mTuner.isRunning()) {
            setTunerStatus("Already running - one axis at a time.");
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
                false /* no balance-angle trim */, false /* no cross-run history */);
        // One trial log per axis: rows from a roll run seed the next roll run and are
        // never matched against a pitch one.
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
        sendFlightControl(FlightControlPacket.emergency());
        setTunerStatus("CUT MOTORS sent - motors off, tuner halted.");
    }

    private void sendFlightControl(CrtpPacket packet) {
        if (mCrazyflie != null) {
            mCrazyflie.sendPacket(packet);
        }
    }

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
        cfg.addVariable("motor.m1", VariableType.UINT16_T);
        mTuneLogConfig = cfg;
        startLogConfig(cfg);
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

    /** Put an icon at the start of a text button, tinted to match its text. */
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
    }

    @Override
    protected void onDestroy() {
        mMain.removeCallbacks(mStatsTick);
        mStartBtn.removeCallbacks(mHoldFire);
        mStartBtn.removeCallbacks(mHoldTick);
        if (mTuner != null) {
            if (mTuner.isRunning()) {
                // Leaving a self-tuning, spinning drone unattended is not an option.
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
