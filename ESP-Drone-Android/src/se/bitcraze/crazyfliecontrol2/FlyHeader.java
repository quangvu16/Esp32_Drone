package se.bitcraze.crazyfliecontrol2;

import android.app.Activity;
import android.content.Intent;
import android.content.res.ColorStateList;
import android.graphics.Typeface;
import android.os.Handler;
import android.os.Looper;
import android.view.HapticFeedbackConstants;
import android.view.View;
import android.widget.TextView;

import androidx.core.content.ContextCompat;

import com.espressif.espdrone.android.R;
import com.google.android.material.button.MaterialButton;

import java.util.Locale;

import se.bitcraze.crazyfliecontrol.prefs.PreferencesActivity;
import se.bitcraze.crazyfliecontrol2.gpsnav.GpsNavActivity;
import se.bitcraze.crazyfliecontrol2.tune.TuneActivity;

/**
 * Binds the shared screen header (res/layout/view_fly_header.xml) on the Mission and
 * Tune screens: sections, link, battery, mode, ARMED and CUT MOTORS, in the same
 * place as on the Fly screen (redesign stage 3).
 *
 * <p>Mission and Tune are started from the Fly screen (MainActivity), which keeps the
 * link up while they are open. So "Fly" simply finishes this screen, and moving to a
 * sibling section starts it and finishes this one: Back always returns to Fly.
 *
 * <p>Status is read from {@link DroneSession}, which MainPresenter's log block keeps
 * current, twice a second while the screen is visible.
 * Gắn header màn hình dùng chung (res/layout/view_fly_header.xml) cho các màn hình
 * Mission và Tune: các mục, link, pin, mode, ARMED và CUT MOTORS, ở đúng vị trí
 * như trên màn hình Fly (redesign stage 3).
 *
 * <p>Mission và Tune được mở từ màn hình Fly (MainActivity), nơi giữ link kết nối
 * trong khi các màn hình đó mở. Vì vậy "Fly" chỉ đơn giản finish màn hình này, còn
 * chuyển sang mục khác sẽ mở mục đó và finish màn hình này: Back luôn quay về Fly.
 *
 * <p>Trạng thái được đọc từ {@link DroneSession}, nơi log block của MainPresenter
 * cập nhật, hai lần mỗi giây khi màn hình đang hiển thị.
 */
public final class FlyHeader {

    public static final int FLY = 0;
    public static final int MISSION = 1;
    public static final int TUNE = 2;
    public static final int SETUP = 3;

    private static final String[] MODE_NAMES = {"MANUAL", "HOLD", "AUTO", "FOLLOW", "RTH"};
    private static final long POLL_MS = 500;

    /** Lets a screen refuse to be left, e.g. while its mission is flying. */
    /** Cho phép một màn hình từ chối bị rời đi, ví dụ khi sứ mệnh của nó đang bay. */
    public interface LeaveGuard {
        /** A message saying why this screen must stay open, or null to allow leaving. */
        /** Thông điệp giải thích vì sao màn hình này phải mở, hoặc null để cho phép rời đi. */
        String blockReason();
    }

    private final Activity mActivity;
    private LeaveGuard mLeaveGuard;
    private final TextView mLink;
    private final TextView mBattery;
    private final TextView mModeChip;
    private final TextView mArmedChip;
    private final MaterialButton mCut;
    private final Handler mHandler = new Handler(Looper.getMainLooper());
    private final Runnable mPoll = new Runnable() {
        @Override
        public void run() {
            refresh();
            mHandler.postDelayed(this, POLL_MS);
        }
    };

    /**
     * @param header   the inflated view_fly_header layout
     * @param selected which section this screen is
     * @param onCut    what CUT MOTORS does on this screen (it fires on one tap here, as
     *                 these screens are used with the drone at a distance)
     * @param header   layout view_fly_header đã inflate
     * @param selected màn hình này thuộc mục nào
     * @param onCut    CUT MOTORS làm gì trên màn hình này (chạy ngay khi chạm một lần,
     *                 vì các màn hình này được dùng khi drone ở xa)
     */
    public FlyHeader(Activity activity, View header, int selected, final Runnable onCut) {
        mActivity = activity;
        mLink = (TextView) header.findViewById(R.id.hdr_link_text);
        mBattery = (TextView) header.findViewById(R.id.hdr_battery_text);
        mModeChip = (TextView) header.findViewById(R.id.hdr_mode_chip);
        mArmedChip = (TextView) header.findViewById(R.id.hdr_armed_chip);
        mCut = (MaterialButton) header.findViewById(R.id.hdr_cut_motors);

        int[] ids = {R.id.hdr_tab_fly, R.id.hdr_tab_mission, R.id.hdr_tab_tune, R.id.hdr_tab_setup};
        int surface = color(R.color.fly_surface);
        for (int i = 0; i < ids.length; i++) {
            final int section = i;
            MaterialButton tab = (MaterialButton) header.findViewById(ids[i]);
            boolean isSelected = i == selected;
            if (isSelected) {
                tab.setBackgroundTintList(ColorStateList.valueOf(surface));
                tab.setTextColor(color(R.color.fly_ink));
                tab.setTypeface(null, Typeface.BOLD);
                tab.setSelected(true);
            }
            if (!isSelected) {
                tab.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        go(section);
                    }
                });
            }
        }

        mCut.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mCut.performHapticFeedback(HapticFeedbackConstants.LONG_PRESS);
                onCut.run();
            }
        });
        refresh();
    }

    public void setLeaveGuard(LeaveGuard guard) {
        mLeaveGuard = guard;
    }

    /** Call from onResume. */
    /** Gọi từ onResume. */
    public void start() {
        mHandler.removeCallbacks(mPoll);
        mHandler.post(mPoll);
    }

    /** Call from onPause. */
    /** Gọi từ onPause. */
    public void stop() {
        mHandler.removeCallbacks(mPoll);
    }

    private void go(int section) {
        // Setup opens on top and comes back here, so it is always allowed.
        // Setup mở đè lên trên rồi quay lại đây, nên luôn được phép.
        String reason = (section == SETUP || mLeaveGuard == null) ? null : mLeaveGuard.blockReason();
        if (reason != null) {
            android.widget.Toast.makeText(mActivity, reason, android.widget.Toast.LENGTH_LONG).show();
            return;
        }
        switch (section) {
            case FLY:
                mActivity.finish();
                break;
            case MISSION:
                mActivity.startActivity(new Intent(mActivity, GpsNavActivity.class));
                mActivity.finish();
                break;
            case TUNE:
                mActivity.startActivity(TuneActivity.newIntent(mActivity));
                mActivity.finish();
                break;
            case SETUP:
                mActivity.startActivity(new Intent(mActivity, PreferencesActivity.class));
                break;
            default:
                break;
        }
    }

    private void refresh() {
        boolean connected = DroneSession.get() != null;
        String quality = DroneSession.linkQuality();
        mLink.setText(connected ? (quality == null ? "Connected" : "Link " + quality) : "Not connected");
        mLink.setTextColor(color(connected ? R.color.fly_ok : R.color.fly_muted));

        float vbat = connected ? DroneSession.vbat() : -1f;
        if (vbat < 0f) {
            mBattery.setText("— V");
            mBattery.setTextColor(color(R.color.fly_muted));
        } else {
            mBattery.setText(String.format(Locale.US, "%.2f V", vbat));
            mBattery.setTextColor(color(vbat < 3.3f ? R.color.fly_danger
                    : (vbat < 3.5f ? R.color.fly_warn : R.color.fly_ink)));
        }

        int mode = connected ? DroneSession.mode() : -1;
        if (mode < 0) {
            mModeChip.setVisibility(View.GONE);
            mArmedChip.setVisibility(View.GONE);
        } else {
            mModeChip.setText(MODE_NAMES[Math.min(mode, MODE_NAMES.length - 1)]);
            mModeChip.setVisibility(View.VISIBLE);
            boolean armed = DroneSession.armed();
            mArmedChip.setText(armed ? "● ARMED" : "DISARMED");
            mArmedChip.setBackgroundResource(armed ? R.drawable.bg_fly_chip_armed : R.drawable.bg_fly_chip_disarmed);
            mArmedChip.setTextColor(color(armed ? R.color.fly_ink : R.color.fly_muted));
            mArmedChip.setContentDescription(armed ? "Armed" : "Disarmed");
            mArmedChip.setVisibility(View.VISIBLE);
        }

        mCut.setEnabled(connected);
        int fill = color(connected ? R.color.fly_danger : R.color.fly_line);
        int text = color(connected ? R.color.fly_surface : R.color.fly_muted);
        mCut.setBackgroundTintList(ColorStateList.valueOf(fill));
        mCut.setTextColor(text);
        mCut.setIconTint(ColorStateList.valueOf(text));
    }

    private int color(int res) {
        return ContextCompat.getColor(mActivity, res);
    }
}
