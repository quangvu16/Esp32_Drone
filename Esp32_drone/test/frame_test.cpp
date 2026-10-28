// Regression guard for the launch-frame rotation in crtpHandleNav.
// Run with ./run.sh, which also checks these formulas still match the sketch.
// Chốt chặn hồi quy cho phép quay khung cất cánh trong crtpHandleNav.
// Chạy bằng ./run.sh, công cụ này cũng kiểm tra các công thức còn khớp với sketch.
//
// navSetpoints() computes ex = tgt_x - pos_x, ey = tgt_y - pos_y and tilts
// toward (ex, ey). That is only correct if TARGET and the dead reckoning put a
// physical point in the SAME frame. If they disagree in one axis the drone
// turns away from any target that is not straight ahead.
// navSetpoints() tính ex = tgt_x - pos_x, ey = tgt_y - pos_y và nghiêng về
// (ex, ey). Điều đó chỉ đúng nếu TARGET và tự dẫn đường quán tính đặt một điểm
// vật lý trong CÙNG một hệ quy chiếu. Nếu chúng lệch nhau trên một trục, drone
// sẽ quay xa khỏi mọi mục tiêu không nằm thẳng phía trước.
//
// Stated convention, which navUpdate() also implements:
//   +x = where the nose pointed at ALIGN        (compass bearing alpha)
//   +y = 90 degrees to the LEFT of that         (compass bearing alpha - 90)
// Quy ước đã nêu, mà navUpdate() cũng thực hiện:
//   +x = hướng mũi chỉ tại ALIGN        (phương hướng la bàn alpha)
//   +y = lệch 90 độ sang TRÁI           (phương hướng la bàn alpha - 90)
//
// The old tgt_y = -n*sin(a) + e*cos(a) projected onto STARBOARD instead. It is
// kept below as a negative control so this test cannot pass vacuously.
// Công thức cũ tgt_y = -n*sin(a) + e*cos(a) lại chiếu sang mạn phải. Nó được
// giữ bên dưới làm đối chứng âm để bài kiểm tra này không thể pass rỗng tuếch.

#include <cmath>
#include <cstdio>

#define DEG_TO_RAD 0.017453292519943295f

// Project a north/east offset onto the launch frame. Current firmware.
// Chiếu độ lệch bắc/đông lên khung cất cánh. Firmware hiện tại.
static void targetRot(float n, float e, float alpha_deg, float *tx, float *ty) {
  const float a = alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  *tx = n * ca + e * sa;
  *ty = n * sa - e * ca;
}

// The formula this test exists to catch.
// Công thức mà bài kiểm tra này tồn tại để phát hiện.
static void targetRotBuggy(float n, float e, float alpha_deg, float *tx, float *ty) {
  const float a = alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  *tx =  n * ca + e * sa;
  *ty = -n * sa + e * ca;
}

// Decompose the same physical displacement into the body axes at yaw_rel = 0,
// which is exactly what navUpdate() integrates into pos_x / pos_y when the
// drone is still pointing the launch heading.
// Phân rã cùng một độ dịch vật lý vào các trục thân khi yaw_rel = 0, đúng như
// những gì navUpdate() tích phân vào pos_x / pos_y khi drone còn đang hướng theo
// hướng mũi khi cất cánh.
static void posRot(float n, float e, float alpha_deg, float *px, float *py) {
  const float a = alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  *px = n * ca + e * sa;         // along the nose
                                 // dọc theo mũi
  *py = n * sa - e * ca;         // 90 deg left of it
                                 // lệch 90 độ sang trái
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
                                  // mũi ở phương hướng la bàn 30 khi cất cánh
  const float a = alpha * DEG_TO_RAD, ca = cosf(a), sa = sinf(a);

  printf("launch nose at compass bearing %.0f deg\n\n", alpha);

  // The four cardinal directions relative to the launch nose. Each is placed by
  // its true compass bearing, so the expected launch-frame result is known.
  // Bốn hướng chính so với hướng mũi khi cất cánh. Mỗi điểm được đặt theo
  // phương hướng la bàn thật của nó, nên kết quả kỳ vọng trong hệ quy chiếu
  // cất cánh là đã biết trước.
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
  // Đối chứng âm: công thức cũ phải FAIL ở trường hợp cánh trái, nên một lượt
  // chạy xanh ở đây chứng minh bài kiểm tra thực sự đang kiểm tra điều gì đó.
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
