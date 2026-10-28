// Does the body<->world rotation in navUpdate()/navSetpoints() run the right
// way? Run with ./run.sh, which reads NAV_HEADING_SIGN out of the sketch and
// passes it here, so this checks the firmware's ACTUAL configuration.
// Phép quay thân <-> thế giới trong navUpdate()/navSetpoints() có chạy đúng
// chiều không? Chạy bằng ./run.sh, công cụ này đọc NAV_HEADING_SIGN từ sketch
// và truyền vào đây, nên bài kiểm tra này kiểm tra cấu hình THẬT của firmware.
//
// The property that matters: if the target sits straight along the drone's
// NOSE in the real world, navSetpoints() must command pure forward pitch and
// no roll, whatever yaw the drone is holding. Get the rotation backwards and
// it rolls toward the target's cross-axis instead -- up to 8.66 deg of spurious
// roll at 30 deg of yaw, spiralling away from anything not dead ahead.
// Tính chất quan trọng: nếu mục tiêu nằm thẳng dọc theo MŨI drone trong thế
// giới thực, navSetpoints() phải lệnh pitch thuần tiến tới và không roll, bất
// kể drone đang giữ yaw nào. Quay ngược chiều thì drone lại roll về phía trục
// ngang của mục tiêu -- tới 8,66 độ roll giả ở yaw 30 độ, xoắn ra xa khỏi mọi
// thứ không nằm thẳng phía trước.
//
// navUpdate() and navSetpoints() are transposes of each other so they are
// always mutually consistent. What this pins is the ABSOLUTE direction, which
// comes down to which way heading_deg turns. See the NAV_HEADING_SIGN comment
// in the sketch for why it has to be -1.
// navUpdate() và navSetpoints() là chuyển vị của nhau nên chúng luôn nhất quán
// với nhau. Điều bài test này chốt là chiều TUYỆT ĐỐI, xoay quanh việc
// heading_deg quay theo chiều nào. Xem chú thích NAV_HEADING_SIGN trong sketch
// để biết vì sao nó phải là -1.

#include <cmath>
#include <cstdio>

#ifndef HEADING_SIGN
#define HEADING_SIGN -1     // must match NAV_HEADING_SIGN in the sketch
                            // phải khớp với NAV_HEADING_SIGN trong sketch
#endif

#define DEG_TO_RAD 0.017453292519943295f

// ---- the rotation as the sketch applies it, with NAV_HEADING_SIGN folded in ----
// ---- phép quay như sketch áp dụng, với NAV_HEADING_SIGN đã được gộp vào ----
static void worldToBody(float ux, float uy, float yaw_rel_deg,
                        float *body_fwd, float *body_left) {
  const float yaw_rel = yaw_rel_deg * DEG_TO_RAD;
  const float cy = cosf(yaw_rel), sy = sinf(yaw_rel);
  *body_fwd  =  ux * cy + uy * sy;
  *body_left = -ux * sy + uy * cy;
}

static void bodyToWorld(float fwd, float left, float yaw_rel_deg,
                        float *wx, float *wy) {
  const float yaw_rel = yaw_rel_deg * DEG_TO_RAD;
  const float cy = cosf(yaw_rel), sy = sinf(yaw_rel);
  *wx = fwd * cy - left * sy;
  *wy = fwd * sy + left * cy;
}

static int fails = 0;

static void check(const char *what, float want, float got, float tol = 1e-2f) {
  if (fabsf(want - got) > tol) {
    printf("FAIL %-46s want %+.4f got %+.4f\n", what, want, got);
    fails++;
  } else {
    printf("ok   %-46s (%+.4f)\n", what, got);
  }
}

// Where the nose really points after a right turn of t degrees. This is a fact
// about the world and does not depend on how heading is labelled: +y is the
// drone's LEFT, so a right turn goes toward -y.
// Hướng mũi thật sự chỉ sau khi rẽ phải t độ. Đây là một sự thật về thế giới
// và không phụ thuộc cách heading được gán nhãn: +y là phía TRÁI của drone,
// nên rẽ phải là đi về phía -y.
static void noseAfterRightTurn(float t_deg, float *nx, float *ny) {
  const float t = t_deg * DEG_TO_RAD;
  *nx = cosf(t);
  *ny = -sinf(t);
}

// yaw_rel is NAV_HEADING_SIGN * (heading - heading_ref). A compass bearing
// rises to the right, so a right turn of t gives heading - heading_ref = +t.
// yaw_rel là NAV_HEADING_SIGN * (heading - heading_ref). Phương hướng la bàn
// tăng theo chiều phải, nên rẽ phải t độ cho heading - heading_ref = +t.
static float yawRelForRightTurn(float t_deg) {
  return (float)HEADING_SIGN * t_deg;
}

int main() {
  printf("NAV_HEADING_SIGN = %d\n", HEADING_SIGN);

  // A target dead along the nose must give pure pitch at every yaw.
  // Một mục tiêu nằm thẳng dọc mũi phải cho pitch thuần ở mọi yaw.
  for (int deg = -75; deg <= 75; deg += 25) {
    float nx, ny;
    noseAfterRightTurn((float)deg, &nx, &ny);

    float bf, bl;
    worldToBody(10.0f * nx, 10.0f * ny, yawRelForRightTurn((float)deg), &bf, &bl);

    char label[64];
    snprintf(label, sizeof label, "target on the nose, yawed %+4d: pitch", deg);
    check(label, 10.0f, bf);
    snprintf(label, sizeof label, "target on the nose, yawed %+4d: roll ", deg);
    check(label, 0.0f, bl);
  }

  // A target directly off the LEFT wing must give pure positive roll.
  // Một mục tiêu nằm ngay ngoài cánh TRÁI phải cho roll dương thuần.
  for (int deg = -75; deg <= 75; deg += 25) {
    float nx, ny;
    noseAfterRightTurn((float)deg, &nx, &ny);
    const float lx = -ny, ly = nx;      // 90 deg toward +y, i.e. the left wing
                                        // lệch 90 độ về phía +y, tức là cánh trái

    float bf, bl;
    worldToBody(10.0f * lx, 10.0f * ly, yawRelForRightTurn((float)deg), &bf, &bl);

    char label[64];
    snprintf(label, sizeof label, "target on the left wing, yawed %+4d: pitch", deg);
    check(label, 0.0f, bf);
    snprintf(label, sizeof label, "target on the left wing, yawed %+4d: roll ", deg);
    check(label, 10.0f, bl);
  }

  // The two rotations must stay exact inverses whatever the sign.
  // Hai phép quay phải luôn là nghịch đảo chính xác của nhau bất kể dấu.
  {
    float wx, wy, bf, bl;
    bodyToWorld(3.0f, -2.0f, 37.0f, &wx, &wy);
    worldToBody(wx, wy, 37.0f, &bf, &bl);
    printf("\n");
    check("round trip returns fwd", 3.0f, bf);
    check("round trip returns left", -2.0f, bl);
  }

  if (fails) {
    printf("\n%d failure(s). NAV_HEADING_SIGN should be %d -- flip it in the sketch.\n",
           fails, -HEADING_SIGN);
  } else {
    printf("\nrotation sense is correct for NAV_HEADING_SIGN = %d\n", HEADING_SIGN);
  }
  return fails ? 1 : 0;
}
