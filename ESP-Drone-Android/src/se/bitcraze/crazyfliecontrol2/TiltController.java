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

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

import java.util.Locale;

/**
 * Tilt-to-fly from the phone's rotation vector (PROTOCOL.md section 9).
 * Nghiêng điện thoại để bay, dùng vector quay của điện thoại (PROTOCOL.md mục 9).
 *
 * <p>Landscape, phone flat = level drone. The rotation vector gives an orientation that
 * does not drift the way a pure gyro integration would, and needs no calibration beyond
 * the sensor's own. Roll and pitch are the phone's attitude; yaw is the rate of turn
 * about the vertical, scaled to deg/s — the firmware treats the yaw field as a rate.
 * <p>Để ngang, điện thoại nằm phẳng = drone thăng bằng. Vector quay cho một định hướng
 * không trôi như cách tích phân con quay thuần túy, và không cần hiệu chuẩn gì thêm ngoài
 * hiệu chuẩn của chính cảm biến. Roll và pitch là tư thế của điện thoại; yaw là tốc độ
 * quay quanh phương thẳng đứng, quy đổi sang deg/s — firmware coi trường yaw là một
 * tốc độ.
 *
 * <p>The mapping is deliberately inert until {@link #setEnabled} is turned on from the
 * labelled toggle, so picking up the phone cannot engage it by accident.
 * <p>Phép ánh xạ cố tình bất hoạt cho đến khi {@link #setEnabled} được bật từ công tắc
 * có nhãn, nên việc nhấc điện thoại lên không thể kích hoạt nhầm.
 */
public class TiltController implements SensorEventListener {

    private static final String LOG_TAG = "TiltController";

    /** Phone attitude inside this many degrees is treated as level. */
    /** Tư thế điện thoại trong khoảng này (đơn vị độ) được coi là thăng bằng. */
    private static final float DEADZONE_DEG = 3.0f;

    /** Phone tilt at or beyond this maps to the full commanded angle. */
    /** Điện thoại nghiêng từ mức này trở lên sẽ ánh xạ sang góc điều khiển đầy đủ. */
    private static final float MAX_TILT_DEG = 25.0f;

    /** The firmware rejects |roll|,|pitch| above this, so never command more. */
    /** Firmware từ chối |roll|,|pitch| lớn hơn mức này, nên không bao giờ điều khiển vượt quá. */
    private static final float FIRMWARE_ANGLE_LIMIT_DEG = 25.0f;

    /** Phone yaw rate at or beyond this maps to the full commanded rate. */
    /** Tốc độ yaw của điện thoại từ mức này trở lên sẽ ánh xạ sang tốc độ điều khiển đầy đủ. */
    private static final float MAX_YAW_RATE_DPS = 120.0f;

    private final SensorManager mSensorManager;
    private final Sensor mRotationSensor;

    private boolean mEnabled;
    private boolean mListening;

    private final float[] mRotationMatrix = new float[9];
    private final float[] mOrientation = new float[3];
    private final float[] mLastAccelerometer = new float[3];
    private final float[] mLastMagnetometer = new float[3];
    private boolean mHaveAccel;
    private boolean mHaveMagnet;

    private float mRollDeg;
    private float mPitchDeg;
    private float mYawRateDps;

    public TiltController(Context context) {
        mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        Sensor rotation = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        if (rotation == null) {
            rotation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        }
        mRotationSensor = rotation;
    }

    public boolean isAvailable() {
        return mRotationSensor != null;
    }

    /**
     * Tilt-to-fly must be deliberately armed from the UI toggle. Turning it off also
     * zeroes the outputs so the drone does not keep tracking a phone that is now idle.
     * Nghiêng điện thoại để bay phải được kích hoạt có chủ đích từ công tắc trên UI. Tắt nó
     * cũng đưa các đầu ra về 0 để drone không tiếp tục bám theo một chiếc điện thoại đang nghỉ.
     */
    public void setEnabled(boolean enabled) {
        mEnabled = enabled;
        if (!enabled) {
            mRollDeg = 0f;
            mPitchDeg = 0f;
            mYawRateDps = 0f;
            stop();
        } else {
            start();
        }
    }

    public boolean isEnabled() {
        return mEnabled;
    }

    public void start() {
        if (mListening || mRotationSensor == null) {
            return;
        }
        // Game rotation vector is the better choice (no magnetometer, so no magnetic
        // interference from the drone's motors), but falls back to the plain rotation
        // vector when the device lacks it.
        // Game rotation vector là lựa chọn tốt hơn (không dùng từ kế, nên không có nhiễu
        // từ của động cơ drone), nhưng sẽ dự phòng sang vector quay thường khi thiết bị
        // không có cảm biến này.
        Sensor sensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        if (sensor == null) {
            sensor = mRotationSensor;
        }
        mSensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_GAME);
        mListening = true;
    }

    public void stop() {
        if (mListening) {
            mSensorManager.unregisterListener(this);
            mListening = false;
        }
    }

    /** Commanded roll in degrees for a CommanderPacket. */
    /** Roll được điều khiển, đơn vị độ, dùng cho CommanderPacket. */
    public float getRollDeg() {
        return mRollDeg;
    }

    /**
     * Commanded pitch in degrees. {@link se.bitcraze.crazyflie.lib.crtp.CommanderPacket}
     * negates the UI pitch on the wire, so return the UI sense (nose-down positive) and
     * let the packet class do the sign, exactly as the stick path does.
     * Pitch được điều khiển, đơn vị độ. {@link se.bitcraze.crazyflie.lib.crtp.CommanderPacket}
     * đảo dấu pitch của UI trên đường truyền, nên trả về theo quy ước UI (mũi chúi xuống là
     * dương) và để lớp gói tin tự xử lý dấu, giống hệt đường xử lý từ cần gạt.
     */
    public float getPitchDeg() {
        return mPitchDeg;
    }

    /** Commanded yaw rate in deg/s. */
    /** Tốc độ yaw được điều khiển, đơn vị deg/s. */
    public float getYawRateDps() {
        return mYawRateDps;
    }

    public String describe() {
        if (!mEnabled) {
            return "Phone tilt is inert until you switch it on.";
        }
        return String.format(Locale.US, "tilt active — roll %+.1f°  pitch %+.1f°  yaw %+.0f°/s",
                mRollDeg, mPitchDeg, mYawRateDps);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (!mEnabled) {
            return;
        }
        if (event.sensor.getType() == Sensor.TYPE_GAME_ROTATION_VECTOR
                || event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            SensorManager.getRotationMatrixFromVector(mRotationMatrix, event.values);
            SensorManager.getOrientation(mRotationMatrix, mOrientation);
            // Orientation is radians: [azimuth, pitch, roll] about the device axes.
            // In landscape the device's roll axis is the drone's roll axis.
            // Orientation là radian: [azimuth, pitch, roll] theo các trục của thiết bị.
            // Ở chiều ngang, trục roll của thiết bị chính là trục roll của drone.
            float rollRad = mOrientation[2];
            float pitchRad = mOrientation[1];
            float yawRad = mOrientation[0];

            mRollDeg = applyDeadzone((float) Math.toDegrees(rollRad));
            mPitchDeg = applyDeadzone((float) Math.toDegrees(pitchRad));

            // Yaw is a rate here, not an angle: differentiate the azimuth so a held
            // angle does not command a continuous turn.
            // Ở đây yaw là tốc độ, không phải góc: vi phân azimuth để một góc giữ cố định
            // không điều khiển drone rẽ liên tục.
            float yawDeg = (float) Math.toDegrees(yawRad);
            float delta = wrapDeg(yawDeg - mLastYawDeg);
            mLastYawDeg = yawDeg;
            // SENSOR_DELAY_GAME is ~50 Hz; treat the sample interval as 20 ms.
            // SENSOR_DELAY_GAME khoảng 50 Hz; coi khoảng lấy mẫu là 20 ms.
            mYawRateDps = clamp(delta * 50f, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);
            if (Math.abs(mYawRateDps) < 2f) {
                mYawRateDps = 0f;
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // The rotation vector is unusable while it is still calibrating, but there is
        // nothing useful to do here beyond letting the sensor settle on its own.
        // Vector quay chưa dùng được trong khi còn đang hiệu chuẩn, nhưng không có việc
        // gì hữu ích cần làm ở đây ngoài để cảm biến tự ổn định.
    }

    private float mLastYawDeg;

    private float applyDeadzone(float deg) {
        if (Math.abs(deg) < DEADZONE_DEG) {
            return 0f;
        }
        float sign = deg < 0 ? -1f : 1f;
        float magnitude = Math.abs(deg) - DEADZONE_DEG;
        float scaled = magnitude / (MAX_TILT_DEG - DEADZONE_DEG);
        return sign * clamp(scaled, 0f, 1f) * FIRMWARE_ANGLE_LIMIT_DEG;
    }

    private static float clamp(float v, float lo, float hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static float wrapDeg(float deg) {
        while (deg > 180f) {
            deg -= 360f;
        }
        while (deg < -180f) {
            deg += 360f;
        }
        return deg;
    }
}
