/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright (C) 2015 Bitcraze AB
 *
 * Crazyflie Nano Quadcopter Client
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

package se.bitcraze.crazyfliecontrol2;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.util.AttributeSet;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.espressif.espdrone.android.R;

import se.bitcraze.crazyflie.lib.crtp.FlightControlPacket;

/**
 * Flight-mode controls for the ESP32 firmware's port 7 command set (PROTOCOL.md section 5).
 *
 * <p>Emergency is deliberately a two-step press: a single mis-tap while the props are
 * spinning must not cut the motors. The second press has to arrive within the window,
 * and the button relabels itself so the pilot can see it is armed.
 */
public class FlightModePanel extends LinearLayout {

    private static final String LOG_TAG = "FlightModePanel";

    /** Window in which the second Emergency press counts, ms. */
    private static final long EMERGENCY_CONFIRM_WINDOW_MS = 3000;

    private static final String[] MODE_LABELS = {
            "Manual", "Hold", "Auto", "Follow", "RTH"
    };

    public interface Listener {
        void onSetMode(int mode);
        void onTakeoff(int hoverThrust);
        void onLand();
        void onArm();
        void onDisarm();
        void onEmergency();
        void onSetHoverThrust(int hoverThrust);
    }

    private Listener mListener;
    private TiltToggleListener mTiltToggleListener;
    private int mHoverThrust = 30000;

    private Spinner mModeSpinner;
    private TextView mStateText;
    private TextView mHoverText;
    private SeekBar mHoverSeek;
    private Button mEmergencyButton;
    private TextView mTiltStatus;

    /** When non-zero, the next Emergency press is the confirming one. */
    private long mEmergencyArmedAtMs;

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

    /** Separate from {@link Listener} because one anonymous class cannot implement two interfaces. */
    public void setTiltToggleListener(TiltToggleListener listener) {
        this.mTiltToggleListener = listener;
    }

    private void bindViews() {
        mModeSpinner = (Spinner) findViewById(R.id.spinner_flight_mode);
        mStateText = (TextView) findViewById(R.id.text_flight_state);
        mHoverText = (TextView) findViewById(R.id.text_hover_thrust);
        mHoverSeek = (SeekBar) findViewById(R.id.seek_hover_thrust);
        mEmergencyButton = (Button) findViewById(R.id.button_emergency);
        mTiltStatus = (TextView) findViewById(R.id.text_tilt_status);

        ArrayAdapter<String> adapter = new ArrayAdapter<String>(getContext(),
                android.R.layout.simple_spinner_item, MODE_LABELS);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        mModeSpinner.setAdapter(adapter);
        mModeSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (mListener != null) {
                    mListener.onSetMode(position);
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });

        findViewById(R.id.button_takeoff).setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mListener != null) {
                    mListener.onTakeoff(mHoverThrust);
                }
            }
        });
        findViewById(R.id.button_land).setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mListener != null) {
                    mListener.onLand();
                }
            }
        });
        findViewById(R.id.button_arm).setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mListener != null) {
                    mListener.onArm();
                }
            }
        });
        findViewById(R.id.button_disarm).setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mListener != null) {
                    mListener.onDisarm();
                }
            }
        });

        // Two-step: first press arms the confirmation, second press fires. Anything
        // else — a third press, a timeout, leaving the screen — disarms it again.
        mEmergencyButton.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                long now = System.currentTimeMillis();
                if (mEmergencyArmedAtMs != 0 && now - mEmergencyArmedAtMs < EMERGENCY_CONFIRM_WINDOW_MS) {
                    mEmergencyArmedAtMs = 0;
                    mEmergencyButton.setText("EMERGENCY");
                    if (mListener != null) {
                        mListener.onEmergency();
                    }
                } else {
                    mEmergencyArmedAtMs = now;
                    mEmergencyButton.setText("Confirm EMERGENCY");
                    Toast.makeText(getContext(), "Press again within 3 s to cut the motors",
                            Toast.LENGTH_SHORT).show();
                }
            }
        });

        // SeekBar is 0..47000 mapped onto the firmware's 8000..55000 range.
        mHoverSeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                mHoverThrust = FlightControlPacket.HOVER_THRUST_MIN + progress;
                mHoverText.setText(String.valueOf(mHoverThrust));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                if (mListener != null) {
                    mListener.onSetHoverThrust(mHoverThrust);
                }
            }
        });
        setHoverThrust(mHoverThrust);

        ToggleButton tiltToggle = (ToggleButton) findViewById(R.id.toggle_tilt_control);
        tiltToggle.setOnCheckedChangeListener(new android.widget.CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(android.widget.CompoundButton buttonView, boolean isChecked) {
                // The toggle itself is the loud affordance: it must be impossible to
                // engage tilt-to-fly without a deliberate press on the labelled switch.
                mTiltStatus.setText(isChecked
                        ? "TILT CONTROL ACTIVE — phone attitude is driving the drone"
                        : "Phone tilt is inert until you switch it on.");
                if (mTiltToggleListener != null) {
                    mTiltToggleListener.onTiltControlToggled(isChecked);
                }
            }
        });
    }

    /** Optional extension so MainActivity can hear the tilt toggle without another callback. */
    public interface TiltToggleListener {
        void onTiltControlToggled(boolean enabled);
    }

    public void setHoverThrust(int hoverThrust) {
        mHoverThrust = Math.max(FlightControlPacket.HOVER_THRUST_MIN,
                Math.min(FlightControlPacket.HOVER_THRUST_MAX, hoverThrust));
        mHoverSeek.setProgress(mHoverThrust - FlightControlPacket.HOVER_THRUST_MIN);
        mHoverText.setText(String.valueOf(mHoverThrust));
    }

    public int getHoverThrust() {
        return mHoverThrust;
    }

    /** Reflects the drone-reported mode; does not re-fire the listener. */
    public void setMode(int mode) {
        if (mode >= 0 && mode < MODE_LABELS.length && mModeSpinner.getSelectedItemPosition() != mode) {
            mModeSpinner.setSelection(mode, false);
        }
    }

    public void setFlightState(String state) {
        mStateText.setText(state);
    }

    /** Show the tilt-to-fly readout (target roll/pitch, or why it is idle). */
    public void setTiltStatus(String status) {
        mTiltStatus.setText(status);
    }

    /**
     * Clear any half-armed Emergency confirmation. Called when the activity pauses,
     * so a stale "Confirm EMERGENCY" cannot be completed by an accidental tap later.
     */
    public void disarmEmergencyConfirm() {
        mEmergencyArmedAtMs = 0;
        mEmergencyButton.setText("EMERGENCY");
    }
}
