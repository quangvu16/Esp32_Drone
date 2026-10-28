package se.bitcraze.crazyfliecontrol2;

import android.content.Context;
import android.content.res.ColorStateList;
import android.graphics.Typeface;
import android.os.Handler;
import android.os.Looper;
import android.util.AttributeSet;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.core.content.ContextCompat;

import com.espressif.espdrone.android.R;
import com.google.android.material.button.MaterialButton;

import java.util.Locale;

import se.bitcraze.crazyflie.lib.crtp.FlightControlPacket;

/**
 * Centre column of the Fly screen (redesign stage 2, PROTOCOL.md section 5).
 *
 * <p>Connected, it shows the flight mode, ONE main action that follows the situation
 * (Takeoff on the ground, Land when armed), four live numbers and a More button.
 * Not connected, it shows what to do next and a Connect button.
 *
 * <p>CUT MOTORS is not here: it lives in the screen header so it sits in the same
 * place on every screen. Hover thrust, tilt control and Arm / Disarm live in the More
 * panel. MainActivity owns both.
 * Cột giữa của màn hình Fly (redesign stage 2, PROTOCOL.md mục 5).
 *
 * <p>Khi đã kết nối, nó hiển thị chế độ bay, MỘT hành động chính theo tình huống
 * (Takeoff khi đang ở đất, Land khi đã armed), bốn số theo thời gian thực và nút More.
 * Khi chưa kết nối, nó hiển thị bước cần làm tiếp và nút Connect.
 *
 * <p>CUT MOTORS không nằm ở đây: nó nằm ở header màn hình để luôn ở cùng một chỗ
 * trên mọi màn hình. Hover lực đẩy, tilt control và Arm / Disarm nằm trong panel More.
 * MainActivity sở hữu cả hai.
 */
public class FlightModePanel extends LinearLayout {

    private static final String[] SEQUENCE = {"idle", "takeoff", "hold", "land"};

    public interface Listener {
        void onSetMode(int mode);
        void onTakeoff(int hoverThrust);
        void onLand();
        void onMore();
        void onConnect();
    }

    private Listener mListener;
    private int mHoverThrust = 30000;
    private boolean mArmed;
    private boolean mStatusSeen;
    private long mArmedSinceMs;

    private View mControls;
    private View mDisconnected;
    private MaterialButton[] mModeButtons;
    private MaterialButton mPrimary;
    private TextView mCaption;
    private TextView mRoll;
    private TextView mPitch;
    private TextView mSequence;
    private TextView mFlightTime;
    private TextView mSteps;
    private TextView mLinkCaption;

    private final Handler mHandler = new Handler(Looper.getMainLooper());
    private final Runnable mTick = new Runnable() {
        @Override
        public void run() {
            updateFlightTime();
            if (mArmed) {
                mHandler.postDelayed(this, 1000);
            }
        }
    };

    public FlightModePanel(Context context, AttributeSet attrs) {
        super(context, attrs);
        setOrientation(LinearLayout.VERTICAL);
        LayoutInflater inflater = (LayoutInflater) context.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        inflater.inflate(R.layout.view_flight_mode, this, true);
        bindViews();
    }

    public void setListener(Listener listener) {
        this.mListener = listener;
    }

    private void bindViews() {
        mControls = findViewById(R.id.fly_controls);
        mDisconnected = findViewById(R.id.fly_disconnected);
        mModeButtons = new MaterialButton[]{
                (MaterialButton) findViewById(R.id.fly_mode_manual),
                (MaterialButton) findViewById(R.id.fly_mode_hold),
                (MaterialButton) findViewById(R.id.fly_mode_auto),
        };
        mPrimary = (MaterialButton) findViewById(R.id.button_primary);
        mCaption = (TextView) findViewById(R.id.text_primary_caption);
        mRoll = (TextView) findViewById(R.id.fly_value_roll);
        mPitch = (TextView) findViewById(R.id.fly_value_pitch);
        mSequence = (TextView) findViewById(R.id.fly_value_sequence);
        mFlightTime = (TextView) findViewById(R.id.fly_value_time);
        mSteps = (TextView) findViewById(R.id.text_connect_steps);
        mLinkCaption = (TextView) findViewById(R.id.text_link_caption);

        for (int i = 0; i < mModeButtons.length; i++) {
            final int mode = i;
            mModeButtons[i].setOnClickListener(new OnClickListener() {
                @Override
                public void onClick(View v) {
                    // Show the choice at once; the drone's next status report confirms it.
                    // Hiển thị lựa chọn ngay; báo cáo trạng thái kế tiếp của drone sẽ xác nhận.
                    selectMode(mode);
                    if (mListener != null) {
                        mListener.onSetMode(mode);
                    }
                }
            });
        }
        mPrimary.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mListener == null) {
                    return;
                }
                if (mArmed) {
                    mListener.onLand();
                } else {
                    mListener.onTakeoff(mHoverThrust);
                }
            }
        });
        findViewById(R.id.button_more).setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mListener != null) {
                    mListener.onMore();
                }
            }
        });
        findViewById(R.id.button_connect_now).setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mListener != null) {
                    mListener.onConnect();
                }
            }
        });

        selectMode(0);
        showPrimary(false);
        setConnected(false, null);
    }

    /** Swap between the fly controls and the "not connected" card. */
    /** Chuyển đổi giữa phần điều khiển bay và thẻ "chưa kết nối". */
    public void setConnected(boolean connected, String linkName) {
        mControls.setVisibility(connected ? VISIBLE : GONE);
        mDisconnected.setVisibility(connected ? GONE : VISIBLE);
        if (!connected) {
            mStatusSeen = false;
            setArmed(false);
            mRoll.setText("—");
            mPitch.setText("—");
            mSequence.setText("—");
        }
        boolean wifi = linkName == null || linkName.toLowerCase(Locale.US).contains("wi");
        mSteps.setText(wifi
                ? "1. Power the drone on.\n2. Join the Wi-Fi network “ESP-Drone” in phone settings.\n3. Come back here and tap Connect."
                : "1. Power the drone on.\n2. Keep this phone's Bluetooth on.\n3. Tap Connect.");
        mLinkCaption.setText("Link: " + (linkName == null ? "Wi-Fi hotspot" : linkName) + " · change it in Setup");
    }

    public void setHoverThrust(int hoverThrust) {
        mHoverThrust = Math.max(FlightControlPacket.HOVER_THRUST_MIN,
                Math.min(FlightControlPacket.HOVER_THRUST_MAX, hoverThrust));
        if (!mArmed) {
            showPrimary(false);
        }
    }

    public int getHoverThrust() {
        return mHoverThrust;
    }

    /** One status report from the drone's log block. */
    /** Một báo cáo trạng thái từ log block của drone. */
    public void setFlightStatus(int mode, boolean armed, int autoState, float roll, float pitch) {
        mStatusSeen = true;
        selectMode(mode);
        setArmed(armed);
        mRoll.setText(String.format(Locale.US, "%.1f°", roll));
        mPitch.setText(String.format(Locale.US, "%.1f°", pitch));
        mSequence.setText(SEQUENCE[Math.max(0, Math.min(SEQUENCE.length - 1, autoState))]);
        findViewById(R.id.fly_cell_roll).setContentDescription("Roll " + mRoll.getText());
        findViewById(R.id.fly_cell_pitch).setContentDescription("Pitch " + mPitch.getText());
        findViewById(R.id.fly_cell_sequence).setContentDescription("Sequence " + mSequence.getText());
    }

    /** Follow and RTH are driven from the Mission screen, so none of the three is selected then. */
    /** Follow và RTH được điều khiển từ màn hình Mission, nên khi đó không nút nào trong ba nút được chọn. */
    private void selectMode(int mode) {
        int surface = ContextCompat.getColor(getContext(), R.color.fly_surface);
        int ink = ContextCompat.getColor(getContext(), R.color.fly_ink);
        int muted = ContextCompat.getColor(getContext(), R.color.fly_muted);
        for (int i = 0; i < mModeButtons.length; i++) {
            boolean selected = i == mode;
            MaterialButton b = mModeButtons[i];
            b.setBackgroundTintList(ColorStateList.valueOf(selected ? surface : 0x00000000));
            b.setTextColor(selected ? ink : muted);
            b.setTypeface(null, selected ? Typeface.BOLD : Typeface.NORMAL);
            b.setSelected(selected);
        }
    }

    private void setArmed(boolean armed) {
        boolean changed = armed != mArmed;
        mArmed = armed;
        if (!changed && mStatusSeen) {
            return;
        }
        showPrimary(armed);
        mHandler.removeCallbacks(mTick);
        if (armed) {
            mArmedSinceMs = System.currentTimeMillis();
            mHandler.post(mTick);
        } else {
            mArmedSinceMs = 0;
            updateFlightTime();
        }
    }

    private void showPrimary(boolean armed) {
        if (armed) {
            mPrimary.setText("Land");
            mPrimary.setIconResource(R.drawable.ic_btn_land);
            mCaption.setText("Ramps thrust down to the ground, then disarms.");
        } else {
            mPrimary.setText("Takeoff");
            mPrimary.setIconResource(R.drawable.ic_btn_takeoff);
            mCaption.setText("Arms, climbs to hover thrust " + mHoverThrust
                    + ", then holds level. Or push throttle up from zero to fly by hand.");
        }
    }

    private void updateFlightTime() {
        long seconds = mArmedSinceMs == 0 ? 0 : (System.currentTimeMillis() - mArmedSinceMs) / 1000;
        mFlightTime.setText(String.format(Locale.US, "%02d:%02d", seconds / 60, seconds % 60));
        findViewById(R.id.fly_cell_time).setContentDescription("Flight time " + mFlightTime.getText());
    }

    @Override
    protected void onDetachedFromWindow() {
        mHandler.removeCallbacks(mTick);
        super.onDetachedFromWindow();
    }
}
