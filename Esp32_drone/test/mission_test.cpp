// Regression guard for the onboard mission engine (PROTOCOL.md §12).
// Run with ./run.sh, which also checks these formulas still match the sketch.
// Chốt chặn hồi quy cho động cơ nhiệm vụ trên bo (PROTOCOL.md §12).
// Chạy bằng ./run.sh, công cụ này cũng kiểm tra các công thức còn khớp với sketch.
//
// Two things are easy to get subtly wrong here, so both are pinned:
//
// 1. The flat-earth lat/lon <-> launch-frame conversion. missionToLocal() and
//    the NAV_CMD_TARGET rotation must put a physical waypoint in the SAME
//    frame navSetpoints() chases, and localToMission() (which feeds the
//    MAVLink telemetry) must be its exact inverse.
// 2. The DO_JUMP repeat accounting. ArduPilot gives p2 the meaning "perform
//    the jump p2 times in total, then fall through to the next item" — one
//    off and the runner either loops forever on the jump item or skips a lap.
//
// Hai thứ dễ sai tinh vi ở đây, nên cả hai đều được chốt:
//
// 1. Phép chuyển đổi đất phẳng lat/lon <-> khung cất cánh. missionToLocal() và
//    phép quay NAV_CMD_TARGET phải đặt một waypoint vật lý trong CÙNG hệ mà
//    navSetpoints() bám theo, và localToMission() (nuôi telemetry MAVLink) phải
//    là phép nghịch đảo chính xác của nó.
// 2. Cách đếm lần lặp của DO_JUMP. ArduPilot cho p2 nghĩa là "thực hiện nhảy
//    tổng cộng p2 lần, rồi đi tiếp sang mục sau" — sai một đơn vị là runner hoặc
//    lặp mãi ở mục nhảy, hoặc bỏ sót một vòng.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#define DEG_TO_RAD 0.017453292519943295f
#define MISSION_MAX_ITEMS    32
#define MISSION_STOP_RADIUS_M 2.5f
#define MISSION_RADIUS_MIN_M  0.5f

#define MAV_CMD_NAV_WAYPOINT 16
#define MAV_CMD_NAV_LAND     21
#define MAV_CMD_NAV_TAKEOFF  22
#define MAV_CMD_DO_JUMP      177

#define MAV_MISSION_ACCEPTED 0
#define MAV_MISSION_INVALID  5

#define AUTO_HOLD            2
#define AUTO_MISSION         4

// One mission item — the MAVLink MISSION_ITEM_INT packing (PROTOCOL.md §12).
// Một mục nhiệm vụ — dạng đóng gói MISSION_ITEM_INT của MAVLink (PROTOCOL.md §12).
typedef struct {
  uint16_t seq;
  uint16_t command;          // MAV_CMD
  float    p1, p2, p3, p4;
  int32_t  lat_e7, lon_e7;   // degrees x 1e7
  float    alt;              // metres — stored and reported, never flown
} MissionItem;

// State the copied functions read, named as in the sketch.
// Các biến trạng thái mà các hàm đã sao chép đọc, đặt tên như trong sketch.
static double   home_lat = 0.0, home_lon = 0.0;
static float    align_alpha_deg = 0.0f;
static MissionItem mission_items[MISSION_MAX_ITEMS];
static uint16_t mission_count = 0;
static int16_t  mission_current = -1;
static uint16_t jump_exec_count[MISSION_MAX_ITEMS];
static int      auto_state = AUTO_MISSION;

static int missionIndexForSeq(uint16_t seq) {
  for (uint16_t i = 0; i < mission_count; i++)
    if (mission_items[i].seq == seq) return (int)i;
  return -1;
}

static void missionSendCurrent() {
  // CRTP output — silent in this test. // Đầu ra CRTP — im lặng trong bài test.
}

// Copied verbatim from missionToLocal() / localToMission() in Esp32_drone.ino.
// Sao chép nguyên văn từ missionToLocal() / localToMission() trong Esp32_drone.ino.
static void missionToLocal(int32_t lat_e7, int32_t lon_e7, float *x, float *y) {
  const float a = align_alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  double n = ((double)lat_e7 * 1e-7 - home_lat) * 111319.49;
  double e = ((double)lon_e7 * 1e-7 - home_lon) * 111319.49 * cos(home_lat * DEG_TO_RAD);
  *x = (float)(n * ca + e * sa);
  *y = (float)(n * sa - e * ca);
}

static void localToMission(float x, float y, int32_t *lat_e7, int32_t *lon_e7) {
  const float a = align_alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  double n = (double)x * ca + (double)y * sa;
  double e = (double)x * sa - (double)y * ca;
  *lat_e7 = (int32_t)llround((home_lat + n / 111319.49) * 1e7);
  *lon_e7 = (int32_t)llround((home_lon + e / (111319.49 * cos(home_lat * DEG_TO_RAD))) * 1e7);
}

// Copied verbatim from missionValidate() in Esp32_drone.ino.
// Sao chép nguyên văn từ missionValidate() trong Esp32_drone.ino.
static int missionValidate(const MissionItem *items, uint16_t count) {
  for (uint16_t i = 0; i < count; i++) {
    if (!isfinite(items[i].p1) || !isfinite(items[i].p2) || !isfinite(items[i].alt))
      return MAV_MISSION_INVALID;
    if (items[i].command == MAV_CMD_DO_JUMP) {
      uint16_t target = (uint16_t)items[i].p1;
      if (target == items[i].seq) return MAV_MISSION_INVALID;   // self-jump
      bool found = false;
      for (uint16_t j = 0; j < count; j++)
        if (items[j].seq == target) { found = true; break; }
      if (!found) return MAV_MISSION_INVALID;
    }
  }
  return MAV_MISSION_ACCEPTED;
}

// Copied verbatim from missionAdvanceInstant() in Esp32_drone.ino.
// Sao chép nguyên văn từ missionAdvanceInstant() trong Esp32_drone.ino.
static bool missionAdvanceInstant() {
  for (int guard = 0; guard < MISSION_MAX_ITEMS; guard++) {
    if (mission_current < 0 || mission_current >= (int)mission_count) return true;
    MissionItem *it = &mission_items[mission_current];
    if (it->command == MAV_CMD_DO_JUMP) {
      int target = missionIndexForSeq((uint16_t)it->p1);
      if (target < 0 || target == mission_current) return false;
      uint16_t times = (uint16_t)it->p2;
      if (it->p2 > 0.0f && jump_exec_count[mission_current] >= times) {
        jump_exec_count[mission_current] = 0;
        mission_current++;
        continue;
      }
      jump_exec_count[mission_current]++;
      mission_current = (int16_t)target;
      missionSendCurrent();
      continue;
    }
    if (it->command == MAV_CMD_NAV_TAKEOFF &&
        (auto_state == AUTO_HOLD || auto_state == AUTO_MISSION)) {
      mission_current++;
      missionSendCurrent();
      continue;
    }
    break;
  }
  return true;
}

static int fails = 0;

static void checkInt(const char *what, long want, long got) {
  if (want != got) {
    printf("FAIL %-52s want %ld got %ld\n", what, want, got);
    fails++;
  } else {
    printf("ok   %-52s (%ld)\n", what, got);
  }
}

static void checkFloat(const char *what, float want, float got, float tol = 1e-2f) {
  if (fabsf(want - got) > tol) {
    printf("FAIL %-52s want %+.4f got %+.4f\n", what, want, got);
    fails++;
  } else {
    printf("ok   %-52s (%+.4f)\n", what, got);
  }
}

static void checkTrue(const char *what, bool ok) {
  if (!ok) {
    printf("FAIL %-52s\n", what);
    fails++;
  } else {
    printf("ok   %-52s\n", what);
  }
}

static void setMission(const MissionItem *items, uint16_t count) {
  memcpy(mission_items, items, sizeof(MissionItem) * count);
  mission_count = count;
  mission_current = 0;
  memset(jump_exec_count, 0, sizeof(jump_exec_count));
}

static MissionItem mk(uint16_t seq, uint16_t cmd, float p1, float p2,
                      int32_t lat_e7 = 0, int32_t lon_e7 = 0, float alt = 0.0f) {
  MissionItem it;
  memset(&it, 0, sizeof(it));
  it.seq = seq;
  it.command = cmd;
  it.p1 = p1;
  it.p2 = p2;
  it.lat_e7 = lat_e7;
  it.lon_e7 = lon_e7;
  it.alt = alt;
  return it;
}

// ---------------------------------------------------------------------------
//  Conversion: a physical point must land in the frame navSetpoints() chases.
//  Chuyển đổi: một điểm vật lý phải nằm trong hệ mà navSetpoints() bám theo.
// ---------------------------------------------------------------------------
static void testConversion() {
  // Ho Chi Minh City, nose at compass bearing 30 at launch.
  // TP.HCM, mũi ở phương la bàn 30 khi cất cánh.
  home_lat = 10.762622;
  home_lon = 106.660172;
  align_alpha_deg = 30.0f;
  const float a = align_alpha_deg * DEG_TO_RAD, ca = cosf(a), sa = sinf(a);

  printf("--- 10 m due north of the origin ---\n");
  {
    int32_t lat_e7 = (int32_t)llround((home_lat + 10.0 / 111319.49) * 1e7);
    int32_t lon_e7 = (int32_t)llround(home_lon * 1e7);
    float x = 0, y = 0;
    missionToLocal(lat_e7, lon_e7, &x, &y);
    checkFloat("  x = 10*cos(alpha)", 10.0f * ca, x);
    checkFloat("  y = 10*sin(alpha)", 10.0f * sa, y);
  }

  printf("--- 10 m due east of the origin ---\n");
  {
    int32_t lat_e7 = (int32_t)llround(home_lat * 1e7);
    int32_t lon_e7 = (int32_t)llround(
        (home_lon + 10.0 / (111319.49 * cos(home_lat * DEG_TO_RAD))) * 1e7);
    float x = 0, y = 0;
    missionToLocal(lat_e7, lon_e7, &x, &y);
    checkFloat("  x = 10*sin(alpha)", 10.0f * sa, x);
    checkFloat("  y = -10*cos(alpha) (east is -y at the nose)", -10.0f * ca, y);
  }

  printf("--- localToMission is the exact inverse ---\n");
  {
    const float pts[][2] = {
      { 10.0f, 0.0f }, { 0.0f, 10.0f }, { -4.5f, 6.75f },
      { 3.25f, -7.5f }, { 0.0f, 0.0f },
    };
    for (size_t k = 0; k < sizeof(pts) / sizeof(pts[0]); k++) {
      int32_t lat_e7 = 0, lon_e7 = 0;
      localToMission(pts[k][0], pts[k][1], &lat_e7, &lon_e7);
      float x = 0, y = 0;
      missionToLocal(lat_e7, lon_e7, &x, &y);
      char buf[64];
      snprintf(buf, sizeof(buf), "  round trip x (%+.2f)", pts[k][0]);
      checkFloat(buf, pts[k][0], x);
      snprintf(buf, sizeof(buf), "  round trip y (%+.2f)", pts[k][1]);
      checkFloat(buf, pts[k][1], y);
    }
  }
}

// ---------------------------------------------------------------------------
//  DO_JUMP: p2 jumps in total, then fall through — ArduPilot accounting.
//  DO_JUMP: tổng cộng p2 lần nhảy, rồi đi tiếp — cách đếm của ArduPilot.
// ---------------------------------------------------------------------------
static void testJumpCounted() {
  // TAKEOFF@0, WP@1, DO_JUMP to seq 1 three times @2, LAND@3.
  // TAKEOFF@0, WP@1, DO_JUMP tới seq 1 ba lần @2, LAND@3.
  MissionItem items[4] = {
    mk(0, MAV_CMD_NAV_TAKEOFF, 0, 0),
    mk(1, MAV_CMD_NAV_WAYPOINT, 0, 2.5f),
    mk(2, MAV_CMD_DO_JUMP, 1, 3),
    mk(3, MAV_CMD_NAV_LAND, 0, 0),
  };
  setMission(items, 4);
  auto_state = AUTO_MISSION;

  printf("--- DO_JUMP p2=3: three jumps, then LAND ---\n");
  checkTrue("  takeoff item skipped (ramp already flown)", missionAdvanceInstant());
  checkInt("  stopped on the WP", 1, mission_current);

  int jumps = 0;
  for (int lap = 0; lap < 8; lap++) {
    // The WP "arrives" -> missionTick advances to the jump item.
    // WP "đến nơi" -> missionTick đi tiếp tới mục nhảy.
    if (mission_items[mission_current].command == MAV_CMD_NAV_WAYPOINT) {
      mission_current++;
      missionSendCurrent();
    }
    int before = mission_current;
    missionAdvanceInstant();
    if (mission_items[before].command == MAV_CMD_DO_JUMP &&
        mission_items[mission_current].command == MAV_CMD_NAV_WAYPOINT) jumps++;
    if (mission_items[before].command == MAV_CMD_DO_JUMP &&
        mission_items[mission_current].command == MAV_CMD_NAV_LAND) break;
  }
  checkInt("  exactly 3 jumps before fall-through", 3, jumps);
  checkInt("  landed on the LAND item", 3, mission_current);
  checkInt("  jump count reset after fall-through", 0, jump_exec_count[2]);
}

static void testJumpForever() {
  // WP@0, DO_JUMP to seq 0 with p2 = 0 = forever.
  // WP@0, DO_JUMP tới seq 0 với p2 = 0 = vĩnh viễn.
  MissionItem items[2] = {
    mk(0, MAV_CMD_NAV_WAYPOINT, 0, 2.5f),
    mk(1, MAV_CMD_DO_JUMP, 0, 0),
  };
  setMission(items, 2);
  auto_state = AUTO_MISSION;

  printf("--- DO_JUMP p2=0: loops forever, never falls through ---\n");
  for (int lap = 0; lap < 5; lap++) {
    if (mission_items[mission_current].command == MAV_CMD_NAV_WAYPOINT) {
      mission_current++;
      missionSendCurrent();
    }
    missionAdvanceInstant();
    checkInt("  jumped back to the WP", 0, mission_current);
  }
  checkInt("  repeat counter grew (no fall-through)", 5, jump_exec_count[1]);
}

static void testJumpValidation() {
  printf("--- missionValidate: broken jump chains are refused ---\n");
  {
    MissionItem items[2] = {
      mk(0, MAV_CMD_NAV_WAYPOINT, 0, 2.5f),
      mk(1, MAV_CMD_DO_JUMP, 1, 0),          // jumps to itself // tự nhảy
    };
    checkInt("  self-jump INVALID", MAV_MISSION_INVALID, missionValidate(items, 2));
  }
  {
    MissionItem items[2] = {
      mk(0, MAV_CMD_NAV_WAYPOINT, 0, 2.5f),
      mk(1, MAV_CMD_DO_JUMP, 9, 0),          // no seq 9 // không có seq 9
    };
    checkInt("  missing target INVALID", MAV_MISSION_INVALID, missionValidate(items, 2));
  }
  {
    MissionItem items[2] = {
      mk(0, MAV_CMD_NAV_WAYPOINT, 0, 2.5f),
      mk(1, MAV_CMD_DO_JUMP, 0, 3),
    };
    checkInt("  real chain ACCEPTED", MAV_MISSION_ACCEPTED, missionValidate(items, 2));
  }

  // Belt and braces: even if a bad chain reached the runner, the instant-
  // advance guard refuses it instead of spinning the CPU.
  // An toàn hai lớp: kể cả chuỗi hỏng lọt vào runner, vòng lặp tức thời vẫn từ
  // chối thay vì xoay CPU.
  printf("--- runner guard: a jump to a missing seq stops the mission ---\n");
  {
    MissionItem items[1] = { mk(0, MAV_CMD_DO_JUMP, 9, 0) };
    setMission(items, 1);
    auto_state = AUTO_MISSION;
    checkTrue("  advanceInstant returns false (broken chain)", !missionAdvanceInstant());
  }
  {
    MissionItem items[1] = { mk(0, MAV_CMD_DO_JUMP, 0, 0) };
    setMission(items, 1);
    auto_state = AUTO_MISSION;
    checkTrue("  advanceInstant returns false (self-jump)", !missionAdvanceInstant());
  }
}

// ---------------------------------------------------------------------------
//  Radius rule: default 2.5 m, floor 0.5 m (PROTOCOL.md §12).
//  Quy tắc bán kính: mặc định 2.5 m, sàn 0.5 m (PROTOCOL.md §12).
// ---------------------------------------------------------------------------
static void testRadiusRule() {
  printf("--- waypoint acceptance radius ---\n");
  {
    float p2 = 0.0f;
    float rad = p2 > MISSION_RADIUS_MIN_M ? p2 : MISSION_STOP_RADIUS_M;
    checkFloat("  p2=0 falls back to the default", 2.5f, rad);
  }
  {
    float p2 = 0.3f;
    float rad = p2 > MISSION_RADIUS_MIN_M ? p2 : MISSION_STOP_RADIUS_M;
    checkFloat("  p2 below the floor falls back", 2.5f, rad);
  }
  {
    float p2 = 1.0f;
    float rad = p2 > MISSION_RADIUS_MIN_M ? p2 : MISSION_STOP_RADIUS_M;
    checkFloat("  p2=1.0 is honoured", 1.0f, rad);
  }
}

int main() {
  testConversion();
  printf("\n");
  testJumpCounted();
  printf("\n");
  testJumpForever();
  printf("\n");
  testJumpValidation();
  printf("\n");
  testRadiusRule();

  printf("\n%s\n", fails ? "MISSION TEST FAILURES" : "all mission tests passed");
  return fails ? 1 : 0;
}
