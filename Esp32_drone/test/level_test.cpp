// Regression guard for the self-level sign in loop().
// Run with ./run.sh, which also checks the two setpoint lines still match the sketch.
//
// The rate loop is modelled as perfect (gyro rate == rate setpoint), which is the
// best case for it. Even then, adding the level term instead of subtracting it is
// positive feedback: any tilt commands MORE tilt the same way and the drone flips.
// The old formula is kept as a negative control so this cannot pass vacuously.

#include <cmath>
#include <cstdio>

#define LEVEL_SENSITIVITY 15.0f
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Simulate `seconds` of flight at 250 Hz from angle0 toward target sp_deg.
// sign = -1 is the current firmware, +1 the old one.
static float simulate(float angle0, float sp_deg, float sign, float seconds) {
  const float dt = 0.004f;
  float angle = angle0;
  for (int i = 0; i < (int)(seconds / dt); i++) {
    float sp_rate = 0.0f;
    float err = angle - sp_deg;
    float level_adjust = clampf(err * LEVEL_SENSITIVITY, -200.0f, 200.0f);
    float setpoint = sp_rate + sign * level_adjust;   // firmware: sp_roll_rate - roll_level_adjust
    float gyro = setpoint;                              // perfect rate loop
    angle += gyro * dt;
    if (fabsf(angle) > 180.0f) return angle;            // flipped
  }
  return angle;
}

static int fails = 0;
static void check(const char *what, bool ok, float got) {
  printf("%s %-50s (%+.3f deg)\n", ok ? "ok  " : "FAIL", what, got);
  if (!ok) fails++;
}

int main() {
  float a;
  a = simulate(+10.0f, 0.0f, -1.0f, 1.0f);
  check("tilted +10 deg, levels out within 1 s", fabsf(a) < 0.1f, a);
  a = simulate(-10.0f, 0.0f, -1.0f, 1.0f);
  check("tilted -10 deg, levels out within 1 s", fabsf(a) < 0.1f, a);
  a = simulate(0.0f, 15.0f, -1.0f, 1.0f);
  check("level, commanded +15 deg, reaches it", fabsf(a - 15.0f) < 0.1f, a);
  a = simulate(0.0f, -15.0f, -1.0f, 1.0f);
  check("level, commanded -15 deg, reaches it", fabsf(a + 15.0f) < 0.1f, a);

  printf("\n--- negative control: the old '+' formula ---\n");
  a = simulate(+2.0f, 0.0f, +1.0f, 1.0f);
  check("old formula diverges from a 2 deg tilt", fabsf(a) > 90.0f, a);

  if (fails) { printf("\n%d level test(s) FAILED\n", fails); return 1; }
  printf("\nall level tests passed\n");
  return 0;
}
