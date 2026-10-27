package se.bitcraze.crazyfliecontrol2.tune;

import java.util.Locale;

/**
 * The 13 tunable parameters (PROTOCOL.md section 3, ids 3-15): what the editors show and
 * what the auto-tuner's search box is clamped to.
 *
 * <p>The min/max values are client-side guards, not firmware limits. The firmware clamps
 * hover_thrust itself (8000..55000) and the PID output to +-400; everything else is free,
 * so the editor limits exist to keep fat-fingered values (and the log-scaled search box)
 * inside something a drone can survive.
 */
public final class ParamSpec {

    public final String completeName;
    public final String label;
    public final String hint;
    public final int decimals;
    public final float min;
    public final float max;
    /** Fallback increment for the -10%/+10% buttons when 10% of the value is zero. */
    public final float step;
    /** True for the UINT16 params (hover_thrust, takeoff_ms, land_ms). */
    public final boolean integer;

    private ParamSpec(String completeName, String label, String hint, int decimals,
                      float min, float max, float step, boolean integer) {
        this.completeName = completeName;
        this.label = label;
        this.hint = hint;
        this.decimals = decimals;
        this.min = min;
        this.max = max;
        this.step = step;
        this.integer = integer;
    }

    // Hints follow the tuning guide's "too low / too high" table. The firmware's
    // pidCompute() is rate-mode: err = gyro_rate - setpoint, p = kp*err, i += ki*err
    // (clamped to +-400), d = kd*(err - last_err), output clamped to +-400.
    public static final ParamSpec[] ALL = {
            new ParamSpec("pid.roll_kp", "Roll rate Kp",
                    "Rate stiffness. Too low: slow, sluggish response. Too high: overshoot, oscillation, buzz.",
                    2, 0f, 100f, 0.05f, false),
            new ParamSpec("pid.roll_ki", "Roll rate Ki",
                    "Trims out steady-state rate error. Too low: drifts back, steady-state error. Too high: slow oscillation, windup twitch.",
                    3, 0f, 20f, 0.005f, false),
            new ParamSpec("pid.roll_kd", "Roll rate Kd",
                    "Damps rate change. Too low: overshoots and rings. Too high: amplifies noise, motor heat, buzz.",
                    1, 0f, 200f, 0.5f, false),
            new ParamSpec("pid.pitch_kp", "Pitch rate Kp",
                    "Rate stiffness. Too low: slow, sluggish response. Too high: overshoot, oscillation, buzz.",
                    2, 0f, 100f, 0.05f, false),
            new ParamSpec("pid.pitch_ki", "Pitch rate Ki",
                    "Trims out steady-state rate error. Too low: drifts back, steady-state error. Too high: slow oscillation, windup twitch.",
                    3, 0f, 20f, 0.005f, false),
            new ParamSpec("pid.pitch_kd", "Pitch rate Kd",
                    "Damps rate change. Too low: overshoots and rings. Too high: amplifies noise, motor heat, buzz.",
                    1, 0f, 200f, 0.5f, false),
            new ParamSpec("pid.yaw_kp", "Yaw rate Kp",
                    "Rate stiffness. Too low: slow, sluggish response. Too high: overshoot, oscillation, buzz.",
                    2, 0f, 100f, 0.05f, false),
            new ParamSpec("pid.yaw_ki", "Yaw rate Ki",
                    "Trims out steady-state rate error. Too low: drifts back, steady-state error. Too high: slow oscillation, windup twitch.",
                    3, 0f, 20f, 0.005f, false),
            new ParamSpec("pid.yaw_kd", "Yaw rate Kd",
                    "Damps rate change. Too low: overshoots and rings. Too high: amplifies noise, motor heat, buzz.",
                    1, 0f, 200f, 0.5f, false),
            new ParamSpec("flight.hover_thrust", "Hover thrust",
                    "Thrust level, not altitude - no barometer on this board. Too low: won't climb / sinks. Too high: jumps / climbs uncommanded.",
                    0, 8000f, 55000f, 100f, true),
            new ParamSpec("flight.takeoff_ms", "Takeoff ramp (ms)",
                    "Takeoff ramp duration. Too low: abrupt. Too high: sluggish.",
                    0, 100f, 20000f, 50f, true),
            new ParamSpec("flight.land_ms", "Land ramp (ms)",
                    "Land ramp duration. Too low: abrupt. Too high: sluggish.",
                    0, 100f, 20000f, 50f, true),
            new ParamSpec("nav.kp", "Nav Kp (tilt deg per m)",
                    "Tilt degrees per metre of position error. Too low: crawls. Too high: aggressive tilt, oscillation.",
                    2, 0f, 20f, 0.05f, false),
    };

    public static ParamSpec byName(String completeName) {
        for (ParamSpec spec : ALL) {
            if (spec.completeName.equals(completeName)) {
                return spec;
            }
        }
        throw new IllegalArgumentException("Unknown parameter " + completeName);
    }

    public float clamp(double value) {
        return (float) Math.max(min, Math.min(max, value));
    }

    public String format(double value) {
        return String.format(Locale.US, "%." + decimals + "f", value);
    }

    /** The wire representation: Integer for the UINT16 params, Float for the FLOAT ones. */
    public Number toWire(double value) {
        if (integer) {
            return Integer.valueOf((int) Math.round(clamp(value)));
        }
        return Float.valueOf(clamp(value));
    }
}
