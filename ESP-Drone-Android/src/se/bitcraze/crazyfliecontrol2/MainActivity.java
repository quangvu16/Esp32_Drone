/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright (C) 2013 Bitcraze AB
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

import java.io.File;
import java.util.List;
import java.util.Locale;

import se.bitcraze.crazyflie.lib.crazyradio.Crazyradio;
import se.bitcraze.crazyflie.lib.crtp.FlightControlPacket;
import se.bitcraze.crazyfliecontrol.controller.Controls;
import se.bitcraze.crazyfliecontrol.controller.GamepadController;
import se.bitcraze.crazyfliecontrol.controller.GyroscopeController;
import se.bitcraze.crazyfliecontrol.controller.IController;
import se.bitcraze.crazyfliecontrol.controller.TouchController;
import se.bitcraze.crazyfliecontrol.prefs.PreferencesActivity;

import android.Manifest;
import android.annotation.TargetApi;
import android.app.AlertDialog;
import android.content.BroadcastReceiver;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.content.res.ColorStateList;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.location.LocationManager;
import android.media.AudioManager;
import android.media.SoundPool;
import android.media.SoundPool.OnLoadCompleteListener;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.preference.PreferenceManager;
import android.provider.Settings;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import android.util.Log;
import android.view.ContextMenu;
import android.view.HapticFeedbackConstants;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ImageButton;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.google.android.material.button.MaterialButton;

import com.MobileAnarchy.Android.Widgets.Joystick.JoystickView;
import com.espressif.espdrone.android.R;

public class MainActivity extends EspActivity {

    private static final String LOG_TAG = "CrazyflieControl";
    private static final int MY_PERMISSIONS_REQUEST_LOCATION = 42;

    private JoystickView mJoystickViewLeft;
    private JoystickView mJoystickViewRight;
    private FlightDataView mFlightDataView;
    private ImageButton mJoystickLeftHLock;

    private ScrollView mConsoleScrollView;
    private TextView mConsoleTextView;

    private SharedPreferences mPreferences;

    private IController mController;
    private GamepadController mGamepadController;

    private String mRadioChannelDefaultValue;
    private String mRadioDatarateDefaultValue;

    private boolean mDoubleBackToExitPressedOnce = false;
    private Controls mControls;
    private SoundPool mSoundPool;
    private boolean mLoaded;
    private int mSoundConnect;
    private int mSoundDisconnect;

    private ImageButton mToggleConnectButton;
    private ImageButton mRingEffectButton;
    private ImageButton mHeadlightButton;
    private ImageButton mBuzzerSoundButton;
    private File mCacheDir;

    private TextView mTextView_battery;
    private TextView mTextView_linkQuality;
    private MainPresenter mPresenter;

    private FlightModePanel mFlightModePanel;

    // Fly screen (redesign stage 2): header status, CUT MOTORS, More panel.
    private static final String[] MODE_NAMES = {"MANUAL", "HOLD", "AUTO", "FOLLOW", "RTH"};
    /** Window in which the second CUT MOTORS press counts, ms. */
    private static final long CUT_CONFIRM_WINDOW_MS = 3000;
    private MaterialButton mCutMotorsButton;
    private long mCutPrimedAtMs;
    private TextView mModeChip;
    private TextView mArmedChip;
    private View mMorePanel;
    private SeekBar mHoverSeek;
    private TextView mHoverText;
    private TextView mTiltStatus;
    private boolean mIsConnected;
    private final Runnable mCutExpire = new Runnable() {
        @Override
        public void run() {
            resetCutMotors();
        }
    };
    private TiltController mTiltController;
    /** Set just before starting a child screen so onPause() leaves the link up. */
    private boolean mSuppressDisconnect;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mPresenter = new MainPresenter(this);

        setDefaultPreferenceValues();

        mTextView_battery = (TextView) findViewById(R.id.battery_text);
        mTextView_linkQuality = (TextView) findViewById(R.id.linkQuality_text);

        setBatteryLevel(-1.0f);
        setLinkQualityText("N/A");

        mControls = new Controls(this, mPreferences);
        mControls.setDefaultPreferenceValues(getResources());

        //Default controller
        mJoystickViewLeft = (JoystickView) findViewById(R.id.joystick_left);
        mJoystickViewRight = (JoystickView) findViewById(R.id.joystick_right);
        mJoystickViewRight.setLeft(false);
        mController = new TouchController(mControls, this, mJoystickViewLeft, mJoystickViewRight);
        mJoystickLeftHLock = findViewById(R.id.joystick_left_hlock);
        mJoystickLeftHLock.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {
                boolean targetState = !mJoystickViewLeft.isHorizontalLocked();
                mJoystickViewLeft.setHorizontalLocked(targetState);
                mJoystickLeftHLock.setBackgroundResource(targetState ? R.drawable.custom_button :
                        R.drawable.custom_button_seledted);
            }
        });

        //initialize gamepad controller
        mGamepadController = new GamepadController(mControls, this, mPreferences);
        mGamepadController.setDefaultPreferenceValues(getResources());

        //initialize buttons
        mToggleConnectButton = (ImageButton) findViewById(R.id.imageButton_connect);
        initializeMenuButtons();

        mFlightDataView = (FlightDataView) findViewById(R.id.flightdataview);

        // Fly screen (redesign stage 2): header, centre panel, More panel.
        mTiltController = new TiltController(this);
        mFlightModePanel = (FlightModePanel) findViewById(R.id.flightmodepanel);
        mMorePanel = findViewById(R.id.fly_more_panel);
        mModeChip = (TextView) findViewById(R.id.fly_mode_chip);
        mArmedChip = (TextView) findViewById(R.id.fly_armed_chip);
        mCutMotorsButton = (MaterialButton) findViewById(R.id.button_emergency);
        mHoverSeek = (SeekBar) findViewById(R.id.seek_hover_thrust);
        mHoverText = (TextView) findViewById(R.id.text_hover_thrust);
        mTiltStatus = (TextView) findViewById(R.id.text_tilt_status);

        mFlightModePanel.setListener(new FlightModePanel.Listener() {
            @Override
            public void onSetMode(int mode) {
                mPresenter.sendFlightControl(FlightControlPacket.setMode(mode));
            }

            @Override
            public void onTakeoff(int hoverThrust) {
                mPresenter.sendFlightControl(FlightControlPacket.takeoff(hoverThrust));
            }

            @Override
            public void onLand() {
                mPresenter.sendFlightControl(FlightControlPacket.land());
            }

            @Override
            public void onMore() {
                mMorePanel.setVisibility(mMorePanel.getVisibility() == View.VISIBLE ? View.GONE : View.VISIBLE);
            }

            @Override
            public void onConnect() {
                // Same path as the old header button, so every link type keeps its checks.
                mToggleConnectButton.performClick();
            }
        });

        // Header sections. Fly is this screen.
        findViewById(R.id.tab_mission).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                launchChild(new Intent(MainActivity.this,
                        se.bitcraze.crazyfliecontrol2.gpsnav.GpsNavActivity.class));
            }
        });
        findViewById(R.id.tab_tune).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                launchChild(se.bitcraze.crazyfliecontrol2.tune.TuneActivity.newIntent(MainActivity.this));
            }
        });
        findViewById(R.id.tab_setup).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startActivity(new Intent(MainActivity.this, PreferencesActivity.class));
            }
        });

        // CUT MOTORS: header, every screen, tap twice within 3 s.
        mCutMotorsButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onCutMotorsTapped();
            }
        });

        // More panel
        findViewById(R.id.fly_more_close).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mMorePanel.setVisibility(View.GONE);
            }
        });
        findViewById(R.id.fly_disconnect).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mMorePanel.setVisibility(View.GONE);
                if (mIsConnected) {
                    mToggleConnectButton.performClick();
                }
            }
        });
        findViewById(R.id.button_arm).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mPresenter.sendFlightControl(FlightControlPacket.arm());
            }
        });
        findViewById(R.id.button_disarm).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mPresenter.sendFlightControl(FlightControlPacket.disarm());
            }
        });
        // SeekBar is 0..47000 mapped onto the firmware's 8000..55000 range.
        mHoverSeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                mHoverText.setText(String.valueOf(FlightControlPacket.HOVER_THRUST_MIN + progress));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                int hover = FlightControlPacket.HOVER_THRUST_MIN + seekBar.getProgress();
                mFlightModePanel.setHoverThrust(hover);
                mPresenter.sendFlightControl(FlightControlPacket.setHover(hover));
            }
        });
        mHoverSeek.setProgress(mFlightModePanel.getHoverThrust() - FlightControlPacket.HOVER_THRUST_MIN);
        ToggleButton tiltToggle = (ToggleButton) findViewById(R.id.toggle_tilt_control);
        tiltToggle.setOnCheckedChangeListener(new android.widget.CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(android.widget.CompoundButton buttonView, boolean isChecked) {
                // The toggle itself is the loud affordance: tilt-to-fly can only be
                // engaged by a deliberate press on the labelled switch.
                mTiltController.setEnabled(isChecked);
                mTiltStatus.setText(isChecked
                        ? "TILT CONTROL ACTIVE — phone attitude is driving the drone"
                        : mTiltController.describe());
            }
        });

        onLinkStateChanged(false);

        mConsoleScrollView = (ScrollView) findViewById(R.id.console_scrollView);
        mConsoleTextView = (TextView) findViewById(R.id.console_textView);
        registerForContextMenu(mConsoleTextView);

        //action buttons
        mRingEffectButton = (ImageButton) findViewById(R.id.button_ledRing);
        mHeadlightButton = (ImageButton) findViewById(R.id.button_headLight);
        mBuzzerSoundButton = (ImageButton) findViewById(R.id.button_buzzerSound);

        IntentFilter filter = new IntentFilter();
        filter.addAction(this.getPackageName()+".USB_PERMISSION");
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        registerReceiver(mUsbReceiver, filter);

        initializeSounds();

        setCacheDir();
    }

    private void initializeSounds() {
        this.setVolumeControlStream(AudioManager.STREAM_MUSIC);
        // Load sounds
        mSoundPool = new SoundPool(10, AudioManager.STREAM_MUSIC, 0);
        mSoundPool.setOnLoadCompleteListener(new OnLoadCompleteListener() {
            @Override
            public void onLoadComplete(SoundPool soundPool, int sampleId, int status) {
                mLoaded = true;
            }
        });
        mSoundConnect = mSoundPool.load(this, R.raw.proxima, 1);
        mSoundDisconnect = mSoundPool.load(this, R.raw.tejat, 1);
    }

    private void setCacheDir() {
        if (isExternalStorageWriteable()) {
            Log.d(LOG_TAG, "External storage is writeable.");
            if (mCacheDir == null) {
                File appDir = getApplicationContext().getExternalFilesDir(null);
                mCacheDir = new File(appDir, "TOC_cache");
                mCacheDir.mkdirs();
            }
        } else {
            Log.d(LOG_TAG, "External storage is not writeable.");
            mCacheDir = null;
        }
    }

    private boolean isExternalStorageWriteable() {
        String state = Environment.getExternalStorageState();
        return Environment.MEDIA_MOUNTED.equals(state);
    }

    private void setDefaultPreferenceValues(){
        // Set default preference values
        PreferenceManager.setDefaultValues(this, R.xml.preferences, false);
        // Initialize preferences
        mPreferences = PreferenceManager.getDefaultSharedPreferences(this);

        mRadioChannelDefaultValue = getString(R.string.preferences_radio_channel_defaultValue);
        mRadioDatarateDefaultValue = getString(R.string.preferences_radio_datarate_defaultValue);
    }

    private void checkScreenLock() {
        boolean isScreenLock = mPreferences.getBoolean(PreferencesActivity.KEY_PREF_SCREEN_ROTATION_LOCK_BOOL, false);
        if(isScreenLock){
            this.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        } else{
            this.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_SENSOR_LANDSCAPE);
        }
    }

    private void checkConsole() {
        boolean showConsole = mPreferences.getBoolean(PreferencesActivity.KEY_PREF_SHOW_CONSOLE_BOOL, false);
        if (showConsole) {
            mConsoleScrollView.setVisibility(View.VISIBLE);
        } else {
            mConsoleScrollView.setVisibility(View.GONE);
        }
    }

    private void initializeMenuButtons() {
        mToggleConnectButton.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {

                try {
                    if (mPresenter != null && mPresenter.getCrazyflie() != null && mPresenter.getCrazyflie().isConnected()) {
                        mPresenter.disconnect();
                    } else {
                        // Only BLE needs the radio/permission pre-checks; WiFi and SPP
                        // do not scan, so gating them on BLE support would block them.
                        String method = mPreferences.getString(PreferencesActivity.KEY_PREF_LINK_METHOD,
                                PreferencesActivity.LINK_METHOD_SOFTAP);
                        if (isCrazyradioAvailable(MainActivity.this)) {
                            connectCrazyradio();
                        } else if (PreferencesActivity.LINK_METHOD_BLE.equals(method)) {
                            connectBlePreChecks();
                        } else {
                            connect();
                        }
                    }
                } catch (IllegalStateException e) {
                    Toast.makeText(MainActivity.this, e.getMessage(), Toast.LENGTH_SHORT).show();
                }
            }
        });

        ImageButton settingsButton = (ImageButton) findViewById(R.id.imageButton_settings);
        settingsButton.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
              Intent intent = new Intent(MainActivity.this, PreferencesActivity.class);
              startActivity(intent);
            }
        });
    }

    private void connectCrazyradio() {
        int radioChannel = Integer.parseInt(mPreferences.getString(PreferencesActivity.KEY_PREF_RADIO_CHANNEL, mRadioChannelDefaultValue));
        int radioDatarate = Integer.parseInt(mPreferences.getString(PreferencesActivity.KEY_PREF_RADIO_DATARATE, mRadioDatarateDefaultValue));
        mPresenter.connectCrazyradio(radioChannel, radioDatarate, mCacheDir);
    }

    private void connectBlePreChecks() {
        // Check if Bluetooth LE is supported by the Android version
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.KITKAT) {
            Log.e(LOG_TAG, Build.VERSION.SDK_INT + "does not support Bluetooth LE.");
            Toast.makeText(this, Build.VERSION.SDK_INT + "does not support Bluetooth LE. Please use a Crazyradio to connect to the Crazyflie instead.", Toast.LENGTH_LONG).show();
            return;
        }
        // Check if Bluetooth LE is supported by the hardware
        if (!getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
            Log.e(LOG_TAG, "Device does not support Bluetooth LE.");
            Toast.makeText(this,  "Device does not support Bluetooth LE. Please use a Crazyradio to connect to the Crazyflie instead.", Toast.LENGTH_LONG).show();
            return;
        }
        // Since Android version 6, ACCESS_COARSE_LOCATION is required for Bluetooth scanning
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            Log.e(LOG_TAG, "Android version >= 6 requires ACCESS_COARSE_LOCATION permissions for Bluetooth scanning.");
            requestPermissions(Manifest.permission.ACCESS_FINE_LOCATION, MY_PERMISSIONS_REQUEST_LOCATION);
        } else {
            connect();
        }
    }

    private void connect() {
        // Dispatch on the transport the pilot picked. SoftAP is the default because
        // it needs no infrastructure: the drone hosts the network itself.
        String method = mPreferences.getString(PreferencesActivity.KEY_PREF_LINK_METHOD,
                PreferencesActivity.LINK_METHOD_SOFTAP);
        if (PreferencesActivity.LINK_METHOD_STATION.equals(method)
                || PreferencesActivity.LINK_METHOD_SOFTAP.equals(method)) {
            connectUDP();
        } else if (PreferencesActivity.LINK_METHOD_BT_SPP.equals(method)) {
            connectBtSpp();
        } else if (PreferencesActivity.LINK_METHOD_BLE.equals(method)) {
            connectBle();
        } else {
            connectUDP();
        }
    }

    private void connectBle() {
        boolean writeWithResponse = mPreferences.getBoolean(PreferencesActivity.KEY_PREF_BLATENCY_BOOL, false);
        Log.d(LOG_TAG, "Using bluetooth write with response - " + writeWithResponse);
        mPresenter.connectBle(writeWithResponse, mCacheDir);
    }

    private void connectBtSpp() {
        mPresenter.connectBtSpp(mCacheDir);
    }

    private void connectUDP() {
        String host = mPreferences.getString(PreferencesActivity.KEY_PREF_LINK_HOST, "192.168.43.42");
        int devicePort = parsePort(mPreferences.getString(PreferencesActivity.KEY_PREF_LINK_DEVICE_PORT, "2390"), 2390);
        int appPort = parsePort(mPreferences.getString(PreferencesActivity.KEY_PREF_LINK_APP_PORT, "2399"), 2399);
        mPresenter.connectUDP(mCacheDir, host, devicePort, appPort);
    }

    private static int parsePort(String value, int fallback) {
        try {
            int port = Integer.parseInt(value.trim());
            return (port > 0 && port <= 65535) ? port : fallback;
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private void checkLocationSettings() {
        LocationManager service = (LocationManager) getSystemService(LOCATION_SERVICE);
        boolean isEnabled = service.isProviderEnabled(LocationManager.GPS_PROVIDER);
        if (!isEnabled) {
            final AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setTitle("Location Access");
            builder.setMessage("The app needs location access for Bluetooth scanning. Please enable it in the settings menu.");
            builder.setPositiveButton(android.R.string.yes, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialogInterface, int i) {
                    Intent intent = new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS);
                    startActivity(intent);
                }
            });
            builder.setNegativeButton(android.R.string.no, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialogInterface, int i) {
                    Log.d(LOG_TAG, "Location access request has been denied.");
                }
            });
            builder.show();
        } else {
            connect();
        }

    }

    private void requestPermissions(String permission, int request) {
        if (ContextCompat.checkSelfPermission(MainActivity.this, permission) != PackageManager.PERMISSION_GRANTED) {
            // Permission is not granted. Should we show an explanation?
            if (ActivityCompat.shouldShowRequestPermissionRationale(MainActivity.this, permission)) {
                // Show an explanation to the user *asynchronously* -- don't block this thread waiting for the user's response!
                // After the user sees the explanation, try again to request the permission.
                Log.d(LOG_TAG, "ACCESS_COARSE_LOCATION permission request has been denied.");
                //Toast.makeText(this,  "Android version >= 6 requires ACCESS_COARSE_LOCATION permissions for Bluetooth scanning.", Toast.LENGTH_LONG).show();
                ActivityCompat.requestPermissions(MainActivity.this, new String[]{permission}, request);
            } else {
                ActivityCompat.requestPermissions(MainActivity.this, new String[]{permission}, request);
            }
        } else {
            // Permission has already been granted
            checkLocationSettings();
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch (requestCode) {
            case MY_PERMISSIONS_REQUEST_LOCATION: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // permission was granted, yay! Do the contacts-related task you need to do.
                    checkLocationSettings();
                } else {
                    // permission denied, boo! Disable the functionality that depends on this permission.
                    Log.d(LOG_TAG, "ACCESS_COARSE_LOCATION permission request has been denied.");
                    Toast.makeText(this,  "Android version >= 6 requires ACCESS_COARSE_LOCATION permissions for Bluetooth scanning.", Toast.LENGTH_LONG).show();
                }
            }
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        //TODO: improve
        PreferencesActivity.setDefaultJoystickSize(this);
        mJoystickViewLeft.setPreferences(mPreferences);
        mJoystickViewRight.setPreferences(mPreferences);
        mControls.setControlConfig();
        mGamepadController.setControlConfig();
        resetInputMethod();
        checkScreenLock();
        checkConsole();
        //disable action buttons
        mRingEffectButton.setEnabled(false);
        mHeadlightButton.setEnabled(false);
        mBuzzerSoundButton.setEnabled(false);
        if (mPreferences.getBoolean(PreferencesActivity.KEY_PREF_IMMERSIVE_MODE_BOOL, false)) {
            setHideyBar();
        }
        //mJoystickViewLeft.requestLayout();

        // Returning from a child screen: drop the guard and refresh the tilt readout
        // so a stale "TILT CONTROL ACTIVE" cannot outlive the sensor registration.
        mSuppressDisconnect = false;
        if (mTiltStatus != null && mTiltController != null) {
            mTiltStatus.setText(mTiltController.describe());
        }
    }

    @Override
    protected void onRestart() {
        super.onRestart();
    }

    @Override
    protected void onPause() {
        super.onPause();
        Log.d(LOG_TAG, "onPause()");
        if (mCutMotorsButton != null) {
            // A half-armed "TAP AGAIN: CUT" must not survive a pause.
            resetCutMotors();
        }
        if (mControls != null) {
            mControls.resetAxisValues();
        }
        if (mController != null) {
            mController.disable();
        }
        updateFlightData();
        // Child screens (PID Tune, GPS Nav) share the one radio link. Disconnecting
        // here would trip the firmware's 500 ms failsafe and cut the motors mid-air.
        if (!mSuppressDisconnect) {
            mPresenter.disconnect();
        }
    }

    /** Starts a child screen without tearing down the live CRTP link. */
    private void launchChild(Intent intent) {
        mSuppressDisconnect = true;
        startActivity(intent);
    }

    /** Used by {@link MainPresenter} so the 50 Hz commander thread can read phone tilt. */
    public TiltController getTiltController() {
        return mTiltController;
    }

    @Override
    protected void onDestroy() {
        Log.d(LOG_TAG, "onDestroy()");
        unregisterReceiver(mUsbReceiver);
        mSoundPool.release();
        mSoundPool = null;
        mPresenter.onDestroy();
        super.onDestroy();
    }

    @Override
    public void onBackPressed() {
        if (mDoubleBackToExitPressedOnce) {
            super.onBackPressed();
            return;
        }
        this.mDoubleBackToExitPressedOnce = true;
        Toast.makeText(this, "Please click BACK again to exit", Toast.LENGTH_SHORT).show();
        new Handler().postDelayed(new Runnable() {

            @Override
            public void run() {
                mDoubleBackToExitPressedOnce = false;

            }
        }, 2000);
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if(mPreferences.getBoolean(PreferencesActivity.KEY_PREF_IMMERSIVE_MODE_BOOL, false) && hasFocus){
            new Handler().postDelayed(new Runnable() {
                @Override
                public void run() {
                    setHideyBar();
                }
            }, 2000);
        }
    }

    @Override
    public void onCreateContextMenu(ContextMenu menu, View v, ContextMenu.ContextMenuInfo menuInfo) {
        if (v.getId() == R.id.console_textView) {
            AdapterView.AdapterContextMenuInfo info = (AdapterView.AdapterContextMenuInfo) menuInfo;
            menu.add(Menu.NONE, 0, 0, "Copy to clipboard");
            menu.add(Menu.NONE, 1, 1, "Clear console");
        }
    }

    @Override
    public boolean onContextItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case 0:
                ClipboardManager cm = (ClipboardManager) getApplicationContext().getSystemService(Context.CLIPBOARD_SERVICE);
                ClipData clipData = ClipData.newPlainText("console text", mConsoleTextView.getText());
                cm.setPrimaryClip(clipData);
                showToastie("Copied to clipboard");
                break;
            case 1:
                mConsoleTextView.setText("");
                showToastie("Console cleared");
                break;
            default:
                break;
        }
        return true;
    }

    @TargetApi(Build.VERSION_CODES.KITKAT)
    private void setHideyBar() {
        Log.i(LOG_TAG, "Activating immersive mode");
        int uiOptions = getWindow().getDecorView().getSystemUiVisibility();
        int newUiOptions = uiOptions;

        if(Build.VERSION.SDK_INT >= 14){
            newUiOptions |= View.SYSTEM_UI_FLAG_HIDE_NAVIGATION;
        }
        if(Build.VERSION.SDK_INT >= 16){
            newUiOptions |= View.SYSTEM_UI_FLAG_FULLSCREEN;
        }
        if(Build.VERSION.SDK_INT >= 18){
            newUiOptions |= View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY;
        }
        getWindow().getDecorView().setSystemUiVisibility(newUiOptions);
    }

    //TODO: fix indirection
    public void updateFlightData(){
        mFlightDataView.updateFlightData(mController.getPitch(), mController.getRoll(), mController.getThrust(), mController.getYaw());
    }

    public void appendToConsole(String text) {
        final String ftext = text;
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mConsoleTextView.append("\n" + ftext);
                mConsoleScrollView.post(new Runnable() {
                    @Override
                    public void run() {
                        mConsoleScrollView.fullScroll(ScrollView.FOCUS_DOWN);
                    }
                });
            }
        });
    }

    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent event) {
        // Check that the event came from a joystick since a generic motion event could be almost anything.
        if ((event.getSource() & InputDevice.SOURCE_CLASS_JOYSTICK) != 0 && event.getAction() == MotionEvent.ACTION_MOVE && mController instanceof GamepadController) {
            mGamepadController.dealWithMotionEvent(event);
            updateFlightData();
            return true;
        } else {
            return super.dispatchGenericMotionEvent(event);
        }
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent event) {
        // do not call super if key event comes from a gamepad, otherwise the buttons can quit the app
        if (isJoystickButton(event.getKeyCode()) && mController instanceof GamepadController) {
            mGamepadController.dealWithKeyEvent(event);
            // exception for OUYA controllers
            if (!Build.MODEL.toUpperCase(Locale.getDefault()).contains("OUYA")) {
                return true;
            }
        }
        return super.dispatchKeyEvent(event);
    }

    // this workaround is necessary because DPad buttons are not considered to be "Gamepad buttons"
    private static boolean isJoystickButton(int keyCode) {
        switch (keyCode) {
            case KeyEvent.KEYCODE_DPAD_CENTER:
            case KeyEvent.KEYCODE_DPAD_LEFT:
            case KeyEvent.KEYCODE_DPAD_RIGHT:
            case KeyEvent.KEYCODE_DPAD_DOWN:
                return true;
            default:
                return KeyEvent.isGamepadButton(keyCode);
        }
    }

    private void resetInputMethod() {
        mController.disable();
        updateFlightData();
        switch (mControls.getControllerType()) {
            case 0:
                // Use GyroscopeController if activated in the preferences
                if (mControls.isUseGyro()) {
                    mController = new GyroscopeController(mControls, this, mJoystickViewLeft, mJoystickViewRight);
                } else {
                    // TODO: reuse existing touch controller?
                    mController = new TouchController(mControls, this, mJoystickViewLeft, mJoystickViewRight);
                }
                break;
            case 1:
                    // TODO: show warning if no game pad is found?
                    mController = mGamepadController;
                break;
            default:
                break;

        }
        mController.enable();
        Toast.makeText(this, "Using " + mController.getControllerName(), Toast.LENGTH_SHORT).show();
    }

    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {

        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            Log.d(LOG_TAG, "mUsbReceiver action: " + action);
            if ((MainActivity.this.getPackageName()+".USB_PERMISSION").equals(action)) {
                //reached only when USB permission on physical connect was canceled and "Connect" or "Radio Scan" is clicked
                synchronized (this) {
                    UsbDevice device = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                        if (device != null) {
                            Toast.makeText(MainActivity.this, "Crazyradio attached", Toast.LENGTH_SHORT).show();
                        }
                    } else {
                        Log.d(LOG_TAG, "permission denied for device " + device);
                    }
                }
            }
            if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action)) {
                UsbDevice device = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                if (device != null && UsbLinkAndroid.isUsbDevice(device, Crazyradio.CRADIO_VID, Crazyradio.CRADIO_PID)) {
                    Log.d(LOG_TAG, "Crazyradio detached");
                    Toast.makeText(MainActivity.this, "Crazyradio detached", Toast.LENGTH_SHORT).show();
                    playSound(mSoundDisconnect);
                    if (mPresenter != null && mPresenter.getCrazyflie() != null) {
                        Log.d(LOG_TAG, "linkDisconnect()");
                        mPresenter.disconnect();
                    }
                }
            }
            if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action)) {
                UsbDevice device = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                if (device != null && UsbLinkAndroid.isUsbDevice(device, Crazyradio.CRADIO_VID, Crazyradio.CRADIO_PID)) {
                    Log.d(LOG_TAG, "Crazyradio attached");
                    Toast.makeText(MainActivity.this, "Crazyradio attached", Toast.LENGTH_SHORT).show();
                    playSound(mSoundConnect);
                }
            }
        }
    };

    private void playSound(int sound){
        if (mLoaded) {
            float volume = 1.0f;
            mSoundPool.play(sound, volume, volume, 1, 0, 1f);
        }
    }

    // extra method for onClick attribute in XML
    public void switchLedRingEffect(View view) {
        if (mPresenter != null) {
            mPresenter.runAltAction("ring.effect");
        }
    }

    // extra method for onClick attribute in XML
    public void toggleHeadlight(View view) {
        if (mPresenter != null) {
            mPresenter.runAltAction("ring.headlightEnable");
        }
    }

    // extra method for onClick attribute in XML
    public void playBuzzerSound(View view) {
        if (mPresenter != null) {
            mPresenter.runAltAction("sound.effect:10");
        }
    }

    public MainPresenter getPresenter() {
        return mPresenter;
    }

    public IController getController(){
        return mController;
    }

    public Controls getControls(){
        return mControls;
    }

    public static boolean isCrazyradioAvailable(Context context) {
        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        if (usbManager == null) {
            throw new IllegalArgumentException("UsbManager == null!");
        }
        List<UsbDevice> usbDeviceList = UsbLinkAndroid.findUsbDevices(usbManager, (short) Crazyradio.CRADIO_VID, (short) Crazyradio.CRADIO_PID);
        return !usbDeviceList.isEmpty();
    }

    /**
     * Battery as volts. The old (V - 3.0) x 100 "percentage" was not a charge level and
     * read 92% on a 1S pack that was nearly flat under load.
     */
    public void setBatteryLevel(final float battery) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (battery < 0f) {
                    mTextView_battery.setText("— V");
                    mTextView_battery.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.fly_muted));
                    return;
                }
                mTextView_battery.setText(String.format(Locale.US, "%.2f V", battery));
                int color = battery < 3.3f ? R.color.fly_danger : (battery < 3.5f ? R.color.fly_warn : R.color.fly_ink);
                mTextView_battery.setTextColor(ContextCompat.getColor(MainActivity.this, color));
            }
        });
    }

    public void setLinkQualityText(final String quality){
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mTextView_linkQuality.setText(format(R.string.linkQuality_text, quality));
            }
        });
    }

    private String format(int identifier, Object o){
        return String.format(getResources().getString(identifier), o);
    }

    public void showToastie(final String message) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), message, Toast.LENGTH_SHORT).show();
            }
        });
    }

    public void setConnectionButtonConnected() {
        setConnectionButtonBackground(R.drawable.custom_button_connected);
    }

    public void setConnectionButtonConnectedBle() {
        setConnectionButtonBackground(R.drawable.custom_button_connected_ble);
    }

    public void setConnectionButtonDisconnected() {
        setConnectionButtonBackground(R.drawable.custom_button);
    }

    public void setConnectionButtonBackground(final int drawable) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mToggleConnectButton.setBackgroundDrawable(getResources().getDrawable(drawable));
                onLinkStateChanged(drawable != R.drawable.custom_button);
            }
        });
    }

    public void setBuzzerSoundButtonEnablement(final boolean enabled) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mBuzzerSoundButton.setEnabled(enabled);
                // Only shown when the drone reports the hardware (this board has none).
                mBuzzerSoundButton.setVisibility(enabled ? View.VISIBLE : View.GONE);
            }
        });
    }

    public void setRingEffectButtonEnablement(final boolean enabled) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mRingEffectButton.setEnabled(enabled);
                // Only shown when the drone reports the hardware (this board has none).
                mRingEffectButton.setVisibility(enabled ? View.VISIBLE : View.GONE);
            }
        });
    }

    public void setHeadlightButtonEnablement(final boolean enabled) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mHeadlightButton.setEnabled(enabled);
                // Only shown when the drone reports the hardware (this board has none).
                mHeadlightButton.setVisibility(enabled ? View.VISIBLE : View.GONE);
            }
        });
    }

    public void toggleHeadlightButtonColor(final boolean toggle) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mHeadlightButton.setColorFilter(toggle ? Color.parseColor("#00FF00") : Color.BLACK);
            }
        });
    }

    // ---- Fly screen (redesign stage 2) ---------------------------------------------

    private String linkName() {
        String method = mPreferences == null ? PreferencesActivity.LINK_METHOD_SOFTAP
                : mPreferences.getString(PreferencesActivity.KEY_PREF_LINK_METHOD,
                        PreferencesActivity.LINK_METHOD_SOFTAP);
        if (PreferencesActivity.LINK_METHOD_STATION.equals(method)) {
            return "Wi-Fi network";
        } else if (PreferencesActivity.LINK_METHOD_BT_SPP.equals(method)) {
            return "Bluetooth Classic";
        } else if (PreferencesActivity.LINK_METHOD_BLE.equals(method)) {
            return "Bluetooth LE";
        }
        return "Wi-Fi hotspot";
    }

    /** Must run on the UI thread. */
    private void onLinkStateChanged(boolean connected) {
        mIsConnected = connected;
        mFlightModePanel.setConnected(connected, linkName());
        mCutMotorsButton.setEnabled(connected);
        resetCutMotors();
        if (!connected) {
            mModeChip.setVisibility(View.GONE);
            mArmedChip.setVisibility(View.GONE);
            mMorePanel.setVisibility(View.GONE);
            mJoystickViewLeft.setAlpha(1f);
            mJoystickViewRight.setAlpha(1f);
        }
    }

    private void onCutMotorsTapped() {
        if (!mIsConnected) {
            return;
        }
        long now = System.currentTimeMillis();
        if (mCutPrimedAtMs != 0 && now - mCutPrimedAtMs < CUT_CONFIRM_WINDOW_MS) {
            resetCutMotors();
            mPresenter.sendFlightControl(FlightControlPacket.emergency());
            mCutMotorsButton.performHapticFeedback(HapticFeedbackConstants.LONG_PRESS);
        } else {
            mCutPrimedAtMs = now;
            mCutMotorsButton.setText("TAP AGAIN: CUT");
            mCutMotorsButton.setBackgroundTintList(ColorStateList.valueOf(0xFFE65100));
            mCutMotorsButton.removeCallbacks(mCutExpire);
            mCutMotorsButton.postDelayed(mCutExpire, CUT_CONFIRM_WINDOW_MS);
        }
    }

    private void resetCutMotors() {
        mCutPrimedAtMs = 0;
        mCutMotorsButton.removeCallbacks(mCutExpire);
        mCutMotorsButton.setText("CUT MOTORS");
        boolean enabled = mCutMotorsButton.isEnabled();
        int fill = ContextCompat.getColor(this, enabled ? R.color.fly_danger : R.color.fly_line);
        int text = ContextCompat.getColor(this, enabled ? R.color.fly_surface : R.color.fly_muted);
        mCutMotorsButton.setBackgroundTintList(ColorStateList.valueOf(fill));
        mCutMotorsButton.setTextColor(text);
        mCutMotorsButton.setIconTint(ColorStateList.valueOf(text));
    }

    /** One status report from the drone (MainPresenter's "Standard" log block). */
    public void setFlightStatus(final int mode, final boolean armed, final int autoState,
                                final float roll, final float pitch) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (!mIsConnected) {
                    return;
                }
                mModeChip.setText(MODE_NAMES[Math.max(0, Math.min(MODE_NAMES.length - 1, mode))]);
                mModeChip.setVisibility(View.VISIBLE);
                if (armed) {
                    mArmedChip.setText("\u25CF ARMED");
                    mArmedChip.setBackgroundResource(R.drawable.bg_fly_chip_armed);
                    mArmedChip.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.fly_ink));
                    mArmedChip.setContentDescription("Armed");
                } else {
                    mArmedChip.setText("DISARMED");
                    mArmedChip.setBackgroundResource(R.drawable.bg_fly_chip_disarmed);
                    mArmedChip.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.fly_muted));
                    mArmedChip.setContentDescription("Disarmed");
                }
                mArmedChip.setVisibility(View.VISIBLE);
                mFlightModePanel.setFlightStatus(mode, armed, autoState, roll, pitch);
                // The sticks only steer in Manual; dim them otherwise so that is visible.
                float alpha = mode == 0 ? 1f : 0.45f;
                mJoystickViewLeft.setAlpha(alpha);
                mJoystickViewRight.setAlpha(alpha);
            }
        });
    }

    public void disableButtonsAndResetBatteryLevel() {
        setRingEffectButtonEnablement(false);
        setHeadlightButtonEnablement(false);
        setBuzzerSoundButtonEnablement(false);
        setBatteryLevel(-1.0f);
    }
}
