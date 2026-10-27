// Regression guard for the launch-frame rotation in crtpHandleNav.
// Run with ./run.sh, which also checks these formulas still match the sketch.
//
// navSetpoints() computes ex = tgt_x - pos_x, ey = tgt_y - pos_y and tilts
// toward (ex, ey). That is only correct if TARGET and the dead reckoning put a
// physical point in the SAME frame. If they disagree in one axis the drone
// turns away from any target that is not straight ahead.
//
// Stated convention, which navUpdate() also implements:
//   +x = where the nose pointed at ALIGN        (compass bearing alpha)
//   +y = 90 degrees to the LEFT of that         (compass bearing alpha - 90)
//
// The old tgt_y = -n*sin(a) + e*cos(a) projected onto STARBOARD instead. It is
// kept below as a negative control so this test cannot pass vacuously.

#include <cmath>
#include <cstdio>

#define DEG_TO_RAD 0.017453292519943295f

// Project a north/east offset onto the launch frame. Current firmware.
static void targetRot(float n, float e, float alpha_deg, float *tx, float *ty) {
  const float a = alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  *tx = n * ca + e * sa;
  *ty = n * sa - e * ca;
}

// The formula this test exists to catch.
static void targetRotBuggy(float n, float e, float alpha_deg, float *tx, float *ty) {
  const float a = alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  *tx =  n * ca + e * sa;
  *ty = -n * sa + e * ca;
}

// Decompose the same physical displacement into the body axes at yaw_rel = 0,
// which is exactly what navUpdate() integrates into pos_x / pos_y when the
// drone is still pointing the launch heading.
static void posRot(float n, float e, float alpha_deg, float *px, float *py) {
  const float a = alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  *px = n * ca + e * sa;         // along the nose
  *py = n * sa - e * ca;         // 90 deg left of it
}

static int fails = 0;

static void check(const char *what, float want, float got, float tol = 1e-3f) {
  if (fabsf(want - got) > tol) {
    printf("FAIL %-52s want %+.4f got %+.4f\n", what, want, got);
    fails++;
  } else {
    printf("ok   %-52s (%+.4f)\n", what, got);
  }
}

static void axes(const char *what, float n, float e, float alpha) {
  float tx, ty, px, py;
  targetRot(n, e, alpha, &tx, &ty);
  posRot(n, e, alpha, &px, &py);
  printf("  %-22s TARGET x=%+.4f y=%+.4f | pos x=%+.4f y=%+.4f\n", what, tx, ty, px, py);
  check("    x agrees", tx, px);
  check("    y agrees", ty, py);
}

int main() {
  const float alpha = 30.0f;      // nose at compass bearing 30 at launch
  const float a = alpha * DEG_TO_RAD, ca = cosf(a), sa = sinf(a);

  printf("launch nose at compass bearing %.0f deg\n\n", alpha);

  // The four cardinal directions relative to the launch nose. Each is placed by
  // its true compass bearing, so the expected launch-frame result is known.
  printf("--- a point 10 m along the launch nose (bearing %.0f) ---\n", alpha);
  axes("ahead", 10.0f * ca, 10.0f * sa, alpha);
  {
    float tx, ty;
    targetRot(10.0f * ca, 10.0f * sa, alpha, &tx, &ty);
    check("  x ~ +10 (straight ahead)", 10.0f, tx);
    check("  y ~ 0 (straight ahead)", 0.0f, ty);
  }

  printf("\n--- a point 10 m off the LEFT wing (bearing %.0f) ---\n", alpha - 90);
  {
    const float b = (alpha - 90.0f) * DEG_TO_RAD;
    float n = 10.0f * cosf(b), e = 10.0f * sinf(b);
    axes("left wing", n, e, alpha);
    float tx, ty;
    targetRot(n, e, alpha, &tx, &ty);
    check("  x ~ 0 (directly abeam)", 0.0f, tx);
    check("  y ~ +10 (LEFT is positive y)", 10.0f, ty);
  }

  printf("\n--- a point 10 m off the RIGHT wing (bearing %.0f) ---\n", alpha + 90);
  {
    const float b = (alpha + 90.0f) * DEG_TO_RAD;
    float n = 10.0f * cosf(b), e = 10.0f * sinf(b);
    axes("right wing", n, e, alpha);
    float tx, ty;
    targetRot(n, e, alpha, &tx, &ty);
    check("  x ~ 0 (directly abeam)", 0.0f, tx);
    check("  y ~ -10 (RIGHT is negative y)", -10.0f, ty);
  }

  printf("\n--- a point 10 m astern (bearing %.0f) ---\n", alpha + 180);
  {
    const float b = (alpha + 180.0f) * DEG_TO_RAD;
    float n = 10.0f * cosf(b), e = 10.0f * sinf(b);
    axes("astern", n, e, alpha);
    float tx, ty;
    targetRot(n, e, alpha, &tx, &ty);
    check("  x ~ -10 (behind)", -10.0f, tx);
    check("  y ~ 0", 0.0f, ty);
  }

  // Negative control: the old formula must FAIL the left-wing case, so a green
  // run here proves the test is actually checking something.
  printf("\n--- negative control: the pre-fix formula ---\n");
  {
    const float b = (alpha - 90.0f) * DEG_TO_RAD;
    float n = 10.0f * cosf(b), e = 10.0f * sinf(b);
    float tx, ty, px, py;
    targetRotBuggy(n, e, alpha, &tx, &ty);
    posRot(n, e, alpha, &px, &py);
    if (fabsf(ty - py) <= 1e-3f) {
      printf("FAIL the old formula now agrees -- this test is vacuous\n");
      return 1;
    }
    printf("ok   old formula puts the left-wing target at y=%+.4f, correct is %+.4f\n", ty, py);
    printf("     (it landed on the STARBOARD side, so the drone would fly away)\n");
  }

  printf("\n%s\n", fails ? "FRAME MISMATCH PRESENT" : "all frame tests passed");
  return fails ? 1 : 0;
}
