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
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import se.bitcraze.crazyflie.lib.crazyradio.Crazyradio;
import se.bitcraze.crazyflie.lib.crtp.FlightControlPacket;
import se.bitcraze.crazyfliecontrol.controller.Controls;
import se.bitcraze.crazyfliecontrol.controller.GamepadController;
import se.bitcraze.crazyfliecontrol.controller.GyroscopeController;
import se.bitcraze.crazyfliecontrol.controller.IController;
import se.bitcraze.crazyfliecontrol.ble.BleLink;
import se.bitcraze.crazyfliecontrol.controller.TouchController;
import se.bitcraze.crazyfliecontrol.prefs.PreferencesActivity;

import android.Manifest;
import android.annotation.TargetApi;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.content.ActivityNotFoundException;
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
    private static final int MY_PERMISSIONS_REQUEST_BT = 43;
    // Continuation for an in-flight permission request (see requestPermissions).
    // Phần việc tiếp theo cho một yêu cầu quyền đang chờ (xem requestPermissions).
    private Runnable mPendingPermissionAction;

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
    // Màn hình Fly (giai đoạn thiết kế lại 2): trạng thái trên header, CUT MOTORS, bảng More.
    private static final String[] MODE_NAMES = {"MANUAL", "HOLD", "AUTO", "FOLLOW", "RTH"};
    /** Window in which the second CUT MOTORS press counts, ms. */
    /** Khoảng thời gian mà lần bấm CUT MOTORS thứ hai còn được tính, theo mili giây. */
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
    /** Được đặt ngay trước khi mở màn hình con để onPause() không ngắt liên kết. */
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
        // Màn hình Fly (giai đoạn thiết kế lại 2): header, bảng giữa, bảng More.
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
                // Đi cùng một đường với nút header cũ, để mọi loại liên kết giữ nguyên bước kiểm tra.
                mToggleConnectButton.performClick();
            }
        });

        // Header sections. Fly is this screen.
        // Các mục trên header. Fly chính là màn hình này.
        findViewById(R.id.tab_plan).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                launchChild(new Intent(MainActivity.this,
                        se.bitcraze.crazyfliecontrol2.gpsnav.PlannerActivity.class));
            }
        });
        findViewById(R.id.tab_mission).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                launchChild(new Intent(MainActivity.this,
                        se.bitcraze.crazyfliecontrol2.gpsnav.GpsNavActivity.class));
            }
        });
        findViewById(R.id.tab_indoor).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                launchChild(new Intent(MainActivity.this,
                        se.bitcraze.crazyfliecontrol2.gpsnav.IndoorMapActivity.class));
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
        // CUT MOTORS: trên header, mọi màn hình, phải bấm hai lần trong vòng 3 giây.
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
        // SeekBar nhận giá trị 0..47000, ánh xạ vào dải 8000..55000 của firmware.
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
                // Bản thân công tắc là điểm tương tác rõ ràng: chế độ nghiêng để bay chỉ
                // được kích hoạt bằng một cú bấm có chủ đích vào công tắc có nhãn.
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
                        // Chỉ BLE cần kiểm tra radio/quyền trước; WiFi và SPP không quét
                        // thiết bị, nên chặn chúng bằng điều kiện hỗ trợ BLE sẽ làm chúng bị chặn oan.
                        String method = mPreferences.getString(PreferencesActivity.KEY_PREF_LINK_METHOD,
                                PreferencesActivity.LINK_METHOD_SOFTAP);
                        if (isCrazyradioAvailable(MainActivity.this)) {
                            connectCrazyradio();
                        } else if (PreferencesActivity.LINK_METHOD_BLE.equals(method)) {
                            connectBlePreChecks();
                        } else if (PreferencesActivity.LINK_METHOD_BT_SPP.equals(method)
                                || PreferencesActivity.LINK_METHOD_BT_DUAL.equals(method)) {
                            connectBtSppPreChecks();
                        } else {
                            connect();
                        }
                    }
                    // SecurityException: a Bluetooth permission the runtime
                    // check missed (or a revoked one) — say so instead of
                    // crashing in the adapter code below.
                    // SecurityException: quyền Bluetooth mà bước kiểm tra runtime
                    // đã lỡ (hoặc bị thu hồi) — báo rõ thay vì crash trong đoạn
                    // adapter phía dưới.
                } catch (IllegalStateException | SecurityException e) {
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
        // Kiểm tra phiên bản Android có hỗ trợ Bluetooth LE không
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.KITKAT) {
            Log.e(LOG_TAG, Build.VERSION.SDK_INT + "does not support Bluetooth LE.");
            Toast.makeText(this, Build.VERSION.SDK_INT + "does not support Bluetooth LE. Please use a Crazyradio to connect to the Crazyflie instead.", Toast.LENGTH_LONG).show();
            return;
        }
        // Check if Bluetooth LE is supported by the hardware
        // Kiểm tra phần cứng có hỗ trợ Bluetooth LE không
        if (!getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
            Log.e(LOG_TAG, "Device does not support Bluetooth LE.");
            Toast.makeText(this,  "Device does not support Bluetooth LE. Please use a Crazyradio to connect to the Crazyflie instead.", Toast.LENGTH_LONG).show();
            return;
        }
        // Android 12+ (API 31): BLUETOOTH_SCAN + BLUETOOTH_CONNECT are the
        // runtime pair. With neverForLocation on SCAN no location permission
        // is needed. Android 6..11 need the location permission for scanning.
        // Android 12+ (API 31): BLUETOOTH_SCAN + BLUETOOTH_CONNECT là cặp quyền
        // chạy. Với neverForLocation trên SCAN thì không cần quyền vị trí.
        // Android 6..11 cần quyền vị trí để quét.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            // Permission first: even reading the radio state can throw without
            // BLUETOOTH_CONNECT on Android 12+.
            // Quyền trước: ngay cả việc đọc trạng thái radio cũng có thể ném lỗi
            // khi thiếu BLUETOOTH_CONNECT trên Android 12+.
            requestPermissions(new String[]{
                    Manifest.permission.BLUETOOTH_SCAN,
                    Manifest.permission.BLUETOOTH_CONNECT},
                MY_PERMISSIONS_REQUEST_BT,
                new Runnable() {
                    @Override
                    public void run() {
                        if (isBluetoothEnabled()) {
                            connect();
                        }
                    }
                });
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            if (isBluetoothEnabled()) {
                Log.e(LOG_TAG, "Android version >= 6 requires ACCESS_COARSE_LOCATION permissions for Bluetooth scanning.");
                requestPermissions(Manifest.permission.ACCESS_FINE_LOCATION, MY_PERMISSIONS_REQUEST_LOCATION);
            }
        } else {
            if (isBluetoothEnabled()) {
                connect();
            }
        }
    }

    // SPP connects to a known adapter address and never scans, so it only
    // needs BLUETOOTH_CONNECT on Android 12+ (and the radio on).
    // SPP kết nối tới địa chỉ adapter đã biết và không quét, nên chỉ cần
    // BLUETOOTH_CONNECT trên Android 12+ (và radio phải bật).
    private void connectBtSppPreChecks() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            // Permission first — see connectBlePreChecks.
            // Quyền trước — xem connectBlePreChecks.
            requestPermissions(new String[]{Manifest.permission.BLUETOOTH_CONNECT},
                MY_PERMISSIONS_REQUEST_BT,
                new Runnable() {
                    @Override
                    public void run() {
                        if (isBluetoothEnabled()) {
                            connect();
                        }
                    }
                });
        } else {
            if (isBluetoothEnabled()) {
                connect();
            }
        }
    }

    // Shared radio check for both Bluetooth transports. A missing adapter is
    // permanent; a disabled one is actionable, so offer the settings screen.
    // Kiểm tra radio dùng chung cho cả hai tầng Bluetooth. Adapter không có là
    // lỗi vĩnh viễn; adapter tắt thì sửa được, nên mở màn hình cài đặt.
    private boolean isBluetoothEnabled() {
        BluetoothAdapter adapter = getBluetoothAdapter();
        if (adapter == null) {
            Toast.makeText(this, "This device has no Bluetooth adapter.", Toast.LENGTH_LONG).show();
            return false;
        }
        if (!adapter.isEnabled()) {
            Toast.makeText(this, "Bluetooth is off — turn it on and connect again.", Toast.LENGTH_LONG).show();
            try {
                startActivity(new Intent(Settings.ACTION_BLUETOOTH_SETTINGS));
            } catch (ActivityNotFoundException e) {
                Log.w(LOG_TAG, "No Bluetooth settings screen: " + e.getMessage());
            }
            return false;
        }
        return true;
    }

    // Context#getSystemService instead of the deprecated
    // BluetoothAdapter.getDefaultAdapter() (API 31 removed the warnings; the
    // old path also has no clean "no adapter" answer).
    // Context#getSystemService thay cho BluetoothAdapter.getDefaultAdapter()
    // đã lỗi thời (API 31 bỏ cảnh báo; đường cũ cũng không trả lời gọn khi
    // không có adapter).
    private BluetoothAdapter getBluetoothAdapter() {
        BluetoothManager manager = (BluetoothManager) getSystemService(BLUETOOTH_SERVICE);
        return manager != null ? manager.getAdapter() : null;
    }

    private void connect() {
        // Dispatch on the transport the pilot picked. SoftAP is the default because
        // it needs no infrastructure: the drone hosts the network itself. The
        // coexistence profiles (4/5/6) pick the side this app can use: WiFi UDP
        // for 4/5, SPP for 6.
        // Chuyển tiếp theo tầng truyền mà người lái chọn. SoftAP là mặc định vì
        // không cần hạ tầng sẵn có: chính drone tự phát mạng. Các hồ sơ chung
        // sống (4/5/6) chọn phía mà app này dùng được: WiFi UDP cho 4/5, SPP
        // cho 6.
        String method = mPreferences.getString(PreferencesActivity.KEY_PREF_LINK_METHOD,
                PreferencesActivity.LINK_METHOD_SOFTAP);
        if (PreferencesActivity.LINK_METHOD_STATION.equals(method)
                || PreferencesActivity.LINK_METHOD_SOFTAP.equals(method)
                || PreferencesActivity.LINK_METHOD_WIFI_AP_BLE.equals(method)
                || PreferencesActivity.LINK_METHOD_WIFI_STA_BLE.equals(method)) {
            connectUDP();
        } else if (PreferencesActivity.LINK_METHOD_BT_SPP.equals(method)
                || PreferencesActivity.LINK_METHOD_BT_DUAL.equals(method)) {
            connectBtSpp();
        } else if (PreferencesActivity.LINK_METHOD_BLE.equals(method)) {
            connectBle();
        } else {
            connectUDP();
        }
    }

    // ------------------------------------------------------------ device picker
    // The transport alone does not say WHERE to connect: the pilot picks the
    // drone from what the radio can see (BLE) or has paired (SPP), and the app
    // connects to that exact device — no name scan, so two drones advertising
    // the same name stay apart.
    // Tầng truyền không nói được KẾT NỐI Ở ĐÂU: người lái chọn drone từ những
    // gì radio thấy được (BLE) hoặc đã ghép đôi (SPP), rồi app kết nối đúng
    // thiết bị đó — không quét tên, nên hai drone trùng tên quảng bá vẫn
    // phân biệt được.

    private ScanCallback mPickerScan;

    private void connectBle() {
        final boolean writeWithResponse = mPreferences.getBoolean(PreferencesActivity.KEY_PREF_BLATENCY_BOOL, false);
        Log.d(LOG_TAG, "Using bluetooth write with response - " + writeWithResponse);
        // A short unfiltered scan feeds the list; the old filtered name scan
        // would hide exactly the drones whose name the pilot changed.
        // Một lượt quét ngắn không lọc cấp dữ liệu cho danh sách; phép quét lọc
        // theo tên ngày trước lại giấu đúng những drone mà người lái đã đổi tên.
        final Map<String, ScanResult> found = new LinkedHashMap<String, ScanResult>();
        BluetoothAdapter adapter = getBluetoothAdapter();
        BluetoothLeScanner scanner = adapter != null ? adapter.getBluetoothLeScanner() : null;
        if (scanner == null) {
            Toast.makeText(this, "Bluetooth is off — turn it on and connect again.", Toast.LENGTH_LONG).show();
            return;
        }
        Toast.makeText(this, "Scanning for Bluetooth devices…", Toast.LENGTH_SHORT).show();
        mPickerScan = new ScanCallback() {
            @Override
            public void onScanResult(int callbackType, ScanResult result) {
                if (result != null && result.getDevice() != null) {
                    synchronized (found) {
                        found.put(result.getDevice().getAddress(), result);
                    }
                }
            }
        };
        scanner.startScan(mPickerScan);
        new Handler().postDelayed(new Runnable() {
            @Override
            public void run() {
                stopPickerScan();
                List<ScanResult> results;
                synchronized (found) {
                    results = new ArrayList<ScanResult>(found.values());
                }
                showBleDeviceList(results, writeWithResponse);
            }
        }, 4000);
    }

    private void showBleDeviceList(List<ScanResult> results, final boolean writeWithResponse) {
        if (results.isEmpty()) {
            Toast.makeText(this, "No Bluetooth devices found.", Toast.LENGTH_LONG).show();
            return;
        }
        // Drone-like names first, then strongest signal.
        // Tên giống drone lên trước, sau đó là tín hiệu mạnh nhất.
        Collections.sort(results, new Comparator<ScanResult>() {
            @Override
            public int compare(ScanResult a, ScanResult b) {
                boolean da = BleLink.isDroneDeviceName(a.getDevice().getName());
                boolean db = BleLink.isDroneDeviceName(b.getDevice().getName());
                if (da != db) {
                    return da ? -1 : 1;
                }
                return b.getRssi() - a.getRssi();
            }
        });
        final List<ScanResult> sorted = results;
        CharSequence[] labels = new CharSequence[sorted.size()];
        for (int i = 0; i < sorted.size(); i++) {
            ScanResult r = sorted.get(i);
            String name = r.getDevice().getName();
            labels[i] = (name != null ? name : "(no name)") + "   " + r.getDevice().getAddress()
                    + "   " + r.getRssi() + " dBm";
        }
        new AlertDialog.Builder(this)
                .setTitle("Bluetooth devices")
                .setItems(labels, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        mPresenter.connectBle(writeWithResponse, mCacheDir, sorted.get(which).getDevice());
                    }
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void connectBtSpp() {
        // SPP has no scan — the list is the already-paired devices.
        // SPP không có quét — danh sách là các thiết bị đã ghép đôi.
        Set<BluetoothDevice> bonded = getBluetoothAdapter().getBondedDevices();
        if (bonded == null || bonded.isEmpty()) {
            Toast.makeText(this, "No paired Bluetooth devices — pair the drone in system settings first.", Toast.LENGTH_LONG).show();
            return;
        }
        final List<BluetoothDevice> all = new ArrayList<BluetoothDevice>(bonded);
        Collections.sort(all, new Comparator<BluetoothDevice>() {
            @Override
            public int compare(BluetoothDevice a, BluetoothDevice b) {
                boolean da = BleLink.isDroneDeviceName(a.getName());
                boolean db = BleLink.isDroneDeviceName(b.getName());
                if (da != db) {
                    return da ? -1 : 1;
                }
                return String.valueOf(a.getName()).compareTo(String.valueOf(b.getName()));
            }
        });
        CharSequence[] labels = new CharSequence[all.size()];
        for (int i = 0; i < all.size(); i++) {
            BluetoothDevice d = all.get(i);
            labels[i] = (d.getName() != null ? d.getName() : "(no name)") + "   " + d.getAddress();
        }
        new AlertDialog.Builder(this)
                .setTitle("Paired Bluetooth devices")
                .setItems(labels, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        mPresenter.connectBtSpp(mCacheDir, all.get(which));
                    }
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void stopPickerScan() {
        if (mPickerScan == null) {
            return;
        }
        BluetoothAdapter adapter = getBluetoothAdapter();
        BluetoothLeScanner scanner = adapter != null ? adapter.getBluetoothLeScanner() : null;
        try {
            if (scanner != null) {
                scanner.stopScan(mPickerScan);
            }
        } catch (IllegalStateException ise) {
            Log.w(LOG_TAG, "stopPickerScan: " + ise.getMessage());
        }
        mPickerScan = null;
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
        // The legacy location path keeps its "then check location settings" step.
        // Đường vị trí cũ giữ bước "kiểm tra cài đặt vị trí" sau khi cấp quyền.
        requestPermissions(new String[]{permission}, request, new Runnable() {
            @Override
            public void run() {
                checkLocationSettings();
            }
        });
    }

    // Requests every permission still missing; runs onGranted immediately when
    // all are already granted, or on the next onRequestPermissionsResult.
    // Yêu cầu mọi quyền còn thiếu; chạy onGranted ngay khi tất cả đã được cấp,
    // hoặc trong lần onRequestPermissionsResult kế tiếp.
    private void requestPermissions(final String[] wanted, final int request, final Runnable onGranted) {
        java.util.List<String> missing = new java.util.ArrayList<String>();
        for (String permission : wanted) {
            if (ContextCompat.checkSelfPermission(MainActivity.this, permission) != PackageManager.PERMISSION_GRANTED) {
                missing.add(permission);
            }
        }
        if (missing.isEmpty()) {
            onGranted.run();
            return;
        }
        // Keep the callback runnable for the result handler.
        // Giữ lại runnable cho hàm xử lý kết quả.
        mPendingPermissionAction = onGranted;
        ActivityCompat.requestPermissions(MainActivity.this,
                missing.toArray(new String[0]), request);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        boolean granted = grantResults.length > 0;
        for (int result : grantResults) {
            if (result != PackageManager.PERMISSION_GRANTED) {
                granted = false;
                break;
            }
        }
        if (requestCode == MY_PERMISSIONS_REQUEST_LOCATION
                || requestCode == MY_PERMISSIONS_REQUEST_BT) {
            if (granted) {
                // Run whatever the pre-check wanted to do once the grant lands.
                // Chạy việc mà bước kiểm tra trước muốn làm sau khi quyền được cấp.
                Runnable action = mPendingPermissionAction;
                mPendingPermissionAction = null;
                if (action != null) {
                    action.run();
                } else if (requestCode == MY_PERMISSIONS_REQUEST_LOCATION) {
                    checkLocationSettings();
                } else {
                    connect();
                }
            } else {
                // Permission denied, boo! Disable the functionality that depends on this permission.
                // Quyền bị từ chối! Tắt các chức năng phụ thuộc vào quyền này.
                Log.d(LOG_TAG, "Bluetooth permission request has been denied.");
                Toast.makeText(this, requestCode == MY_PERMISSIONS_REQUEST_BT
                        ? "Bluetooth permission is required to connect."
                        : "Android version >= 6 requires ACCESS_COARSE_LOCATION permissions for Bluetooth scanning.",
                        Toast.LENGTH_LONG).show();
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
        // Quay lại từ màn hình con: bỏ cờ chặn và làm mới phần hiển thị nghiêng
        // để chuỗi "TILT CONTROL ACTIVE" cũ không tồn tại lâu hơn việc đăng ký cảm biến.
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
            // Trạng thái nửa sẵng sàng "TAP AGAIN: CUT" không được tồn tại qua lần pause.
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
        // Các màn hình con (PID Tune, GPS Nav) dùng chung một liên kết radio. Ngắt
        // kết nối tại đây sẽ kích hoạt failsafe 500 ms của firmware và cắt động cơ giữa trời.
        if (!mSuppressDisconnect) {
            mPresenter.disconnect();
        }
    }

    /** Starts a child screen without tearing down the live CRTP link. */
    /** Mở một màn hình con mà không gỡ liên kết CRTP đang hoạt động. */
    private void launchChild(Intent intent) {
        mSuppressDisconnect = true;
        startActivity(intent);
    }

    /** Used by {@link MainPresenter} so the 50 Hz commander thread can read phone tilt. */
    /** Được {@link MainPresenter} dùng để luồng điều khiển 50 Hz đọc độ nghiêng của điện thoại. */
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
        // Kiểm tra sự kiện có đến từ joystick không, vì sự kiện motion tổng quát có thể là bất cứ thứ gì.
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
        // không gọi super nếu sự kiện phím đến từ gamepad, nếu không các nút có thể thoát ứng dụng
        if (isJoystickButton(event.getKeyCode()) && mController instanceof GamepadController) {
            mGamepadController.dealWithKeyEvent(event);
            // exception for OUYA controllers
            // ngoại lệ dành cho tay cầm OUYA
            if (!Build.MODEL.toUpperCase(Locale.getDefault()).contains("OUYA")) {
                return true;
            }
        }
        return super.dispatchKeyEvent(event);
    }

    // this workaround is necessary because DPad buttons are not considered to be "Gamepad buttons"
    // cách vá này là cần thiết vì các nút DPad không được coi là "nút Gamepad"
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
                // Dùng GyroscopeController nếu được bật trong thiết lập
                if (mControls.isUseGyro()) {
                    mController = new GyroscopeController(mControls, this, mJoystickViewLeft, mJoystickViewRight);
                } else {
                    // TODO: reuse existing touch controller?
                    // TODO: tái sử dụng touch controller hiện có?
                    mController = new TouchController(mControls, this, mJoystickViewLeft, mJoystickViewRight);
                }
                break;
            case 1:
                    // TODO: show warning if no game pad is found?
                    // TODO: hiện cảnh báo nếu không tìm thấy gamepad?
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
                // chỉ đến đây khi quyền USB khi cắm vật lý bị huỷ và bấm "Connect" hoặc "Radio Scan"
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
    // phương thức phụ cho thuộc tính onClick trong XML
    public void switchLedRingEffect(View view) {
        if (mPresenter != null) {
            mPresenter.runAltAction("ring.effect");
        }
    }

    // extra method for onClick attribute in XML
    // phương thức phụ cho thuộc tính onClick trong XML
    public void toggleHeadlight(View view) {
        if (mPresenter != null) {
            mPresenter.runAltAction("ring.headlightEnable");
        }
    }

    // extra method for onClick attribute in XML
    // phương thức phụ cho thuộc tính onClick trong XML
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
     * Pin hiển thị theo vôn. Cách cũ (V - 3.0) x 100 dưới dạng "phần trăm" không phải mức
     * sạc thật và từng báo 92% cho pin 1S gần hết dưới tải.
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
                // Chỉ hiện khi drone báo có phần cứng này (bo này không có).
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
                // Chỉ hiện khi drone báo có phần cứng này (bo này không có).
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
                // Chỉ hiện khi drone báo có phần cứng này (bo này không có).
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
        } else if (PreferencesActivity.LINK_METHOD_WIFI_AP_BLE.equals(method)) {
            return "Wi-Fi hotspot + BLE";
        } else if (PreferencesActivity.LINK_METHOD_WIFI_STA_BLE.equals(method)) {
            return "Wi-Fi network + BLE";
        } else if (PreferencesActivity.LINK_METHOD_BT_DUAL.equals(method)) {
            return "Bluetooth SPP + BLE";
        }
        return "Wi-Fi hotspot";
    }

    /** Must run on the UI thread. */
    /** Phải chạy trên luồng UI. */
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
    /** Một báo cáo trạng thái từ drone (khối log "Standard" của MainPresenter). */
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
                // Cần chỉ lái được ở Manual; các chế độ khác làm mờ cần để thể hiện điều đó.
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
