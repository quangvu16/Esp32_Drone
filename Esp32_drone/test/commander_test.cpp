// Regression guard for the CommanderPacket wire layout (PROTOCOL.md §5) and the
// CRTP arming sequence the phone sticks must satisfy before the motors run.
// Chốt chặn hồi quy cho định dạng gói CommanderPacket (PROTOCOL.md §5) và chuỗi
// kích hoạt CRTP mà cần gạt điện thoại phải thoả trước khi động cơ quay.
//
// The fly screens stream these bytes and the motors can still stay dead: the
// wire is right and the arming gate says no. So this test pins both halves.
// First, the bytes each client builds (Android CommanderPacket, iOS packed
// CommandPacket, the web page's commanderPacket) must decode exactly as
// crtpHandleCommander reads them. Second, the arming sequence must keep
// refusing until it has seen thrust at zero first AND a battery above the
// 3.30 V gate — the two things that look like "the sticks do nothing" from
// the outside.
// Màn hình bay phát các byte này mãi mà động cơ vẫn có thể đứng yên: gói tin
// đúng, còn cổng kích hoạt nói không. Vì vậy bài kiểm tra chốt cả hai nửa.
// Trước tiên, byte mỗi client dựng (CommanderPacket Android, CommandPacket
// đóng gói của iOS, commanderPacket trên trang web) phải giải mã đúng như
// crtpHandleCommander đọc. Sau đó, chuỗi kích hoạt phải tiếp tục từ chối cho
// tới khi đã thấy lực đẩy về 0 TRƯỚC ĐÃ và pin vượt cổng 3,30 V — hai thứ nhìn
// từ ngoài giống hệt "cần gạt không ăn".
//
// The formulas here are copied verbatim out of ../Esp32_drone.ino and the app
// sources; run.sh greps for those exact lines so this file cannot go stale.
// Các công thức ở đây chép nguyên văn từ ../Esp32_drone.ino và mã app; run.sh
// grep đúng các dòng đó để file này không thể lỗi thời lặng lẽ.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

using std::fabsf;
using std::isfinite;

static int fails = 0;

static void check(const char *what, float want, float got, float tol = 1e-3f) {
  if (fabsf(want - got) > tol) {
    printf("FAIL %-56s want %+.4f got %+.4f\n", what, want, got);
    fails++;
  } else {
    printf("ok   %-56s (%+.4f)\n", what, got);
  }
}

static void checkInt(const char *what, long want, long got) {
  if (want != got) {
    printf("FAIL %-56s want %ld got %ld\n", what, want, got);
    fails++;
  } else {
    printf("ok   %-56s (%ld)\n", what, got);
  }
}

static void checkTrue(const char *what, bool cond) {
  if (!cond) {
    printf("FAIL %-56s\n", what);
    fails++;
  } else {
    printf("ok   %-56s\n", what);
  }
}

// =============================================================================
//  The wire: what each client actually builds
//  Đường truyền: mỗi client thực sự dựng gì
// =============================================================================

// Android: CrtpPacket.toByteArray() writes the header byte then the payload with
// ByteOrder.LITTLE_ENDIAN (CrtpPacket.BYTE_ORDER), and
// CommanderPacket.serializeData() writes putFloat(roll); putFloat(-pitch);
// putFloat(yaw); putChar(thrust). putChar is a uint16.
// Android: CrtpPacket.toByteArray() ghi byte header rồi phần dữ liệu với
// ByteOrder.LITTLE_ENDIAN (CrtpPacket.BYTE_ORDER), còn
// CommanderPacket.serializeData() ghi putFloat(roll); putFloat(-pitch);
// putFloat(yaw); putChar(thrust). putChar là uint16.
// The host is little-endian like the wire, so memcpy matches the Java buffer.
// Máy chủ là little-endian như đường truyền, nên memcpy khớp bộ đệm Java.
static void androidPacket(uint8_t out[15], float roll, float pitch, float yaw,
                          uint16_t thrust) {
  out[0] = 0x30;                       // port 3, channel 0
  const float wire_pitch = -pitch;     // putFloat(-mPitch) — invert axis
                                       // đảo trục
  memcpy(out + 1,  &roll,  4);
  memcpy(out + 5,  &wire_pitch, 4);
  memcpy(out + 9,  &yaw,   4);
  memcpy(out + 13, &thrust, 2);        // putChar(mThrust), little-endian
}

// iOS: the packed struct in CommandPacket.h. SimpleCrazyFlieCommander already
// pre-negates pitch in pitch(from:) = control * -1 * pitchRate, so the struct
// carries the wire value directly.
// iOS: struct đóng gói trong CommandPacket.h. SimpleCrazyFlieCommander đã đảo
// dấu pitch trong pitch(from:) = control * -1 * pitchRate, nên struct mang giá
// trị đường truyền sẵn.
struct __attribute__((packed)) IosCommander {
  uint8_t  header;
  float    roll, pitch, yaw;
  uint16_t thrust;
};

static void iosPacket(uint8_t out[15], float roll, float pitch, float yaw,
                      uint16_t thrust) {
  IosCommander c;
  c.header = 0x30;
  c.roll   = roll;
  c.pitch  = -pitch;                   // what pitch(from:) produces
  c.yaw    = yaw;
  c.thrust = thrust;
  memcpy(out, &c, sizeof c);
}

// Web page: commanderPacket() fills a 14-byte payload that is all zeros except
// the u16 thrust little-endian at b[12..13], and pkt() wraps it as
// [0x30][payload][checksum]. It carries no attitude — the page's stand stream
// only holds a thrust level (Spin up).
// Trang web: commanderPacket() điền phần dữ liệu 14 byte toàn số 0 trừ u16
// thrust little-endian tại b[12..13], còn pkt() bọc thành
// [0x30][payload][checksum]. Không mang tư thế — luồng giá đỡ của trang chỉ
// giữ một mức lực đẩy (Spin up).
static void webPacket(uint8_t out[16], uint16_t thrust) {
  uint8_t b[14];                       /* roll/pitch/yaw float 0.0 + u16 thrust */
  memset(b, 0, sizeof b);
  b[12] = (uint8_t)(thrust & 255);
  b[13] = (uint8_t)((thrust >> 8) & 255);
  out[0] = 0x30;
  memcpy(out + 1, b, 14);
  // crtpChecksum(): sum & 0xFF over everything before the checksum byte.
  // crtpChecksum(): sum & 0xFF trên mọi byte trước byte checksum.
  uint8_t sum = 0;
  for (int i = 0; i < 15; i++) sum = (uint8_t)(sum + out[i]);
  out[15] = sum;
}

// Negative control: the same values with the float bytes reversed. That is what
// goes out if the Android buffer ever reverts to ByteBuffer's default
// BIG_ENDIAN (the line that stops it is CrtpPacket.BYTE_ORDER) — the firmware
// must NOT decode it as the intended setpoints.
// Đối chứng âm: cùng giá trị nhưng đảo byte của các số thực. Đó là thứ sẽ lọt
// ra nếu bộ đệm Android quay về BIG_ENDIAN mặc định của ByteBuffer (dòng ngăn
// chuyện đó là CrtpPacket.BYTE_ORDER) — firmware KHÔNG được giải mã nó thành
// các setpoint định nghĩa.
static void androidPacketBigEndian(uint8_t out[15], float roll, float pitch,
                                   float yaw, uint16_t thrust) {
  androidPacket(out, roll, pitch, yaw, thrust);
  for (int i = 1; i < 13; i += 4) {    // swap each 4-byte float in place
    uint8_t *f = out + i;
    uint8_t t0 = f[0], t1 = f[1];
    f[0] = f[3]; f[3] = t0;
    f[1] = f[2]; f[2] = t1;
  }
  out[13] = (uint8_t)(thrust >> 8);    // u16 also in the wrong byte order
  out[14] = (uint8_t)(thrust & 255);
}

// =============================================================================
//  The firmware parser, copied verbatim from crtpHandleCommander
//  Bộ giải mã firmware, chép nguyên văn từ crtpHandleCommander
// =============================================================================

static float    cmd_roll_deg, cmd_pitch_deg, cmd_yaw_dps;
static uint16_t cmd_thrust;
static bool     cmd_valid = false;
static uint32_t cmd_last_ms = 0;
static uint32_t now_ms = 0;
static uint32_t millis() { return now_ms; }

static void crtpHandleCommander(const uint8_t *p, size_t len) {
  // 1 header + 14 payload = 15 CRTP bytes (checksum handled by caller)
  // 1 header + 14 phần dữ liệu = 15 byte CRTP (checksum do hàm gọi xử lý)
  if (len < 15) return;
  float roll, pitch, yaw;
  uint16_t thrust;
  memcpy(&roll,  p + 1,  4);
  memcpy(&pitch, p + 5,  4);
  memcpy(&yaw,   p + 9,  4);
  memcpy(&thrust, p + 13, 2);

  // sanity: reject NaN/inf and absurd angles
  // kiểm tra hợp lệ: loại NaN/inf và các góc vô lý
  if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw)) return;
  if (fabsf(roll) > 180.0f || fabsf(pitch) > 180.0f || fabsf(yaw) > 2000.0f) return;

  cmd_roll_deg  = roll;
  cmd_pitch_deg = pitch;
  cmd_yaw_dps   = yaw;
  cmd_thrust    = thrust;
  cmd_valid     = true;
  cmd_last_ms   = millis();
}

// =============================================================================
//  The arming gate, copied verbatim from armChecksOk / armUpdate CRTP branch
//  Cổng kích hoạt, chép nguyên văn từ armChecksOk / nhánh CRTP của armUpdate
//
// Serial output becomes puts/printf (no Arduino on the host); every decision
// line is untouched. The SBUS half of armUpdate belongs to sbus_test and is
// not copied here.
// Đầu ra Serial thành puts/printf (không có Arduino trên máy chủ); mọi dòng
// quyết định giữ nguyên. Nửa SBUS của armUpdate thuộc sbus_test, không chép ở đây.
// =============================================================================

#define VBAT_CELLS        1
#define VBAT_CELL_CRIT    3.30f
#define VBAT_ARM_CHECK    1
#define MODE_MANUAL       0
#define MODE_HOLD         1

static float    vbat = 0.0f;
static bool     armed = false;
static bool     crtp_thrust_was_low = false;
static uint8_t  flight_mode = MODE_MANUAL;
static uint32_t arm_stick_ms = 0, disarm_stick_ms = 0;

static void pidReset() {}
static void motorStop() {}
static void crtpConsole(const char *) {}

static bool armChecksOk(const char *who) {
#if VBAT_ARM_CHECK
  if (vbat < VBAT_CELLS * VBAT_CELL_CRIT) {
    static uint32_t last_msg_ms = 0;
    uint32_t now = millis();
    if (now - last_msg_ms > 1000) {
      last_msg_ms = now;
      printf("[ARM] %s refused: battery %.2f V is below %.2f V\n",
             who, vbat, VBAT_CELLS * VBAT_CELL_CRIT);
      crtpConsole("arm refused: low battery");
    }
    return false;
  }
#endif
  (void)who;
  return true;
}

static void armUpdate(uint32_t now, bool use_crtp) {
  (void)now;
  if (use_crtp) {
    arm_stick_ms = 0;
    disarm_stick_ms = 0;

    // HOLD/AUTO arm and disarm from the flight-control commands, not from the
    // thrust stick — the stick is not the throttle in those modes. Letting the
    // implicit thrust-up arming run here would fight the auto sequence.
    // HOLD/AUTO kích hoạt và ngắt bằng các lệnh điều khiển bay, không cần gạt lực
    // đẩy — cần gạt không phải throttle trong các chế độ đó. Để việc kích hoạt ngầm
    // khi tăng lực đẩy chạy ở đây sẽ xung đột với chuỗi tự động.
    if (flight_mode != MODE_MANUAL) return;

    if (!armed) {
      // Arm on thrust-up, but only after the app has shown thrust at zero: a
      // phone whose throttle is already up when it connects must not spin the
      // props straight to that level.
      // Kích hoạt khi tăng lực đẩy, nhưng chỉ sau khi app đã báo lực đẩy về 0: một
      // điện thoại có throttle đang tăng khi kết nối không được làm quay cánh quạt
      // thẳng tới mức đó.
      if (cmd_thrust <= 500) crtp_thrust_was_low = true;
      if (cmd_thrust > 1000 && crtp_thrust_was_low && armChecksOk("CRTP thrust-up")) {
        armed = true;
        crtp_thrust_was_low = false;
        pidReset();
        puts("[ARM] Armed (CRTP thrust-up) — props will spin!");
      }
    } else {
      // disarm the moment the app drops thrust to (near) zero
      // ngắt kích hoạt ngay khi app hạ lực đẩy về (gần) 0
      if (cmd_thrust <= 500) {
        armed = false;
        motorStop();
        pidReset();
        puts("[ARM] Disarmed (CRTP thrust-down).");
      }
    }
    return;
  }
}

// =============================================================================
//  The app thrust scalings, copied verbatim from both clients
//  Cách hai app quy đổi lực đẩy, chép nguyên văn từ cả hai client
// =============================================================================

// Android: AbstractController.getThrust()/getThrustAbsolute() with the shipped
// defaults (res/values/strings.xml: minThrust 25, maxThrust 90, deadzone 0.2;
// MAX_THRUST 65000). getThrustFactor() is max(0, maxThrust - minThrust) = 65.
// Android: AbstractController.getThrust()/getThrustAbsolute() với giá trị mặc
// định (res/values/strings.xml: minThrust 25, maxThrust 90, deadzone 0,2;
// MAX_THRUST 65000). getThrustFactor() = max(0, maxThrust - minThrust) = 65.
static uint16_t androidWireThrust(float stick) {
  float thrust = stick;
  if (thrust > 0.2f) {
    thrust = 25.0f + (thrust * 65.0f);   // getThrust() — percent
  } else {
    thrust = 0;
  }
  if (thrust > 0) {
    thrust = thrust / 100 * 65000;       // getThrustAbsolute() — MAX_THRUST
    return (uint16_t)thrust;
  }
  return 0;
}

// iOS: SimpleCrazyFlieCommander.thrust(from:) — the LINEAR_THRUST branch with
// the slow preset's maxThrust 80 (DefaultPreferences.plist), clamped 0..65535.
// iOS: SimpleCrazyFlieCommander.thrust(from:) — nhánh LINEAR_THRUST với
// maxThrust 80 của preset chậm (DefaultPreferences.plist), kẹp 0..65535.
static uint16_t iosWireThrust(float control, float maxThrust) {
  float thrust = control * 65535 * (maxThrust / 100);
  if (thrust > 65535) thrust = 65535;
  if (thrust < 0) thrust = 0;
  return (uint16_t)thrust;
}

// =============================================================================
//  One stream tick, in the order loop() applies it
//  Một nhịp luồng, theo thứ tự loop() áp dụng
// =============================================================================

// loop() derives cmd_ok from cmd_valid and the 500 ms failsafe window, resets
// the zero-thrust witness on any gap, then runs armUpdate only for a fresh
// packet — and in MANUAL phone_ok is cmd_ok, so use_crtp follows the stream.
// loop() tính cmd_ok từ cmd_valid và cửa sổ failsafe 500 ms, xóa bằng chứng
// lực đẩy 0 khi có khoảng trống, rồi chạy armUpdate chỉ khi có gói mới — và ở
// MANUAL phone_ok là cmd_ok, nên use_crtp theo luồng.
static void tick(uint16_t thrust, bool cmd_ok) {
  cmd_thrust = thrust;
  cmd_valid  = cmd_ok;
  now_ms += 20;                        // the apps stream every 20–50 ms
                                       // các app phát mỗi 20–50 ms
  if (!cmd_ok) crtp_thrust_was_low = false;
  if (cmd_ok) armUpdate(now_ms, true); // if (rx_ok) armUpdate(now, use_crtp)
}

static void resetSim(float battery, uint8_t mode) {
  vbat = battery;
  armed = false;
  crtp_thrust_was_low = false;
  flight_mode = mode;
  cmd_valid = false;
  cmd_thrust = 0;
  cmd_roll_deg = cmd_pitch_deg = cmd_yaw_dps = 0;
}

int main() {
  // ---------------------------------------------------------------------------
  printf("--- what the three clients put on the wire ---\n");
  const float R = 12.5f, P = -7.25f, Y = 120.0f;   // app-side stick values
                                                   // giá trị cần gạt phía app
  const uint16_t T = 58500;                        // Android full stick
  uint8_t a[15], i15[15], w[16];
  androidPacket(a, R, P, Y, T);
  iosPacket(i15, R, P, Y, T);
  webPacket(w, 52428);

  checkTrue("android and iOS build identical CRTP bytes",
            memcmp(a, i15, 15) == 0);
  checkInt("the packed iOS struct is exactly 15 bytes", 15, (long)sizeof(IosCommander));
  checkInt("header byte is 0x30 (port 3, channel 0)", 0x30, a[0]);
  checkTrue("web page payload carries thrust at bytes 12..13",
            w[13] == 52428 % 256 && w[14] == 52428 / 256);
  {
    uint8_t sum = 0;
    for (int k = 0; k < 15; k++) sum = (uint8_t)(sum + w[k]);
    checkTrue("web wire = [0x30][payload 14][checksum] (16 bytes)",
              w[15] == sum);
  }

  printf("\n--- decoding what the firmware would read ---\n");
  crtpHandleCommander(a, 15);
  check("android: roll decodes as the app's roll", R, cmd_roll_deg);
  check("android: pitch decodes pre-negated (invert axis)", -P, cmd_pitch_deg);
  check("android: yaw decodes as the app's yaw", Y, cmd_yaw_dps);
  checkInt("android: thrust decodes as the app's u16", T, (long)cmd_thrust);

  crtpHandleCommander(i15, 15);
  check("iOS: roll decodes as the app's roll", R, cmd_roll_deg);
  check("iOS: pitch decodes pre-negated (invert axis)", -P, cmd_pitch_deg);
  checkInt("iOS: thrust decodes as the app's u16", T, (long)cmd_thrust);

  crtpHandleCommander(w, 15);          // crtpReceiveFramed() strips the checksum
                                       // crtpReceiveFramed() cắt checksum
  check("web: attitude decodes as 0 (it sends none)", 0.0f, cmd_roll_deg);
  checkInt("web: Spin up thrust decodes at payload 12..13", 52428, (long)cmd_thrust);

  printf("\n--- the parser refuses what it must refuse ---\n");
  {
    cmd_thrust = 58500;
    crtpHandleCommander(a, 14);        // one byte short: no thrust u16
    checkInt("a 14-byte frame is ignored (thrust unchanged)",
             58500, (long)cmd_thrust);
  }
  {
    uint8_t bad[15];
    androidPacket(bad, R, P, Y, T);
    float nan = NAN;
    memcpy(bad + 1, &nan, 4);
    cmd_thrust = 58500;
    crtpHandleCommander(bad, 15);
    checkInt("NaN roll is ignored (thrust unchanged)", 58500, (long)cmd_thrust);
  }
  {
    uint8_t bad[15];
    androidPacket(bad, 200.0f, P, Y, T);   // absurd angle
    cmd_thrust = 58500;
    crtpHandleCommander(bad, 15);
    checkInt("200 deg roll is ignored (thrust unchanged)", 58500, (long)cmd_thrust);
    androidPacket(bad, R, P, 3000.0f, T);  // absurd rate
    crtpHandleCommander(bad, 15);
    checkInt("3000 deg/s yaw is ignored (thrust unchanged)", 58500, (long)cmd_thrust);
  }
  {
    uint8_t be[15];
    androidPacketBigEndian(be, R, P, Y, T);
    crtpHandleCommander(be, 15);
    checkTrue("big-endian slip does NOT decode as the intended roll",
              fabsf(cmd_roll_deg - R) > 1.0f);
  }

  printf("\n--- the sticks' thrust must pass the arming gates ---\n");
  checkInt("android stick at rest sends 0 (<= 500: the zero witness)",
           0, (long)androidWireThrust(0.0f));
  checkInt("android stick at rest through the 0.2 deadzone sends 0",
           0, (long)androidWireThrust(0.2f));
  checkTrue("android just above the deadzone clears the 1000 gate",
            androidWireThrust(0.25f) > 1000);
  checkInt("android full stick sends 58500", 58500, (long)androidWireThrust(1.0f));
  checkInt("iOS stick at rest sends 0 (<= 500: the zero witness)",
           0, (long)iosWireThrust(0.0f, 80.0f));
  checkTrue("iOS full stick (slow preset) clears the 1000 gate",
            iosWireThrust(1.0f, 80.0f) > 1000);
  checkInt("iOS full stick (slow preset) sends 52428",
           52428, (long)iosWireThrust(1.0f, 80.0f));

  printf("\n--- arming: the fly screen, as the firmware sees it ---\n");
  printf("scenario: MANUAL, battery 3.80 V, Android stick rest then up\n");
  resetSim(3.80f, MODE_MANUAL);
  tick(0, true);
  tick(0, true);
  checkTrue("  stick at rest does not arm yet", !armed);
  tick(androidWireThrust(1.0f), true);
  checkTrue("  stick-up after the zero witness ARMS", armed);

  printf("scenario: the phone connects with the stick already up\n");
  resetSim(3.80f, MODE_MANUAL);
  tick(androidWireThrust(1.0f), true);
  tick(androidWireThrust(1.0f), true);
  checkTrue("  no zero witness: refused (props never jump to the stick)",
            !armed);

  printf("scenario: MANUAL, battery 3.20 V (the sag measured on this board)\n");
  resetSim(3.20f, MODE_MANUAL);
  tick(0, true);
  tick(androidWireThrust(1.0f), true);
  checkTrue("  below the 3.30 V gate: refused (charge the battery)", !armed);

  printf("scenario: armed, then the stick comes back to rest\n");
  resetSim(3.80f, MODE_MANUAL);
  tick(0, true);
  tick(androidWireThrust(1.0f), true);
  checkTrue("  armed on stick-up", armed);
  tick(0, true);
  checkTrue("  stick back to zero disarms", !armed);

  printf("scenario: the stream drops for longer than the 500 ms failsafe\n");
  resetSim(3.80f, MODE_MANUAL);
  tick(0, true);
  tick(0, false);                      // gap: the witness is reset
  tick(androidWireThrust(1.0f), true); // stream back, stick still up
  checkTrue("  after a gap the zero witness is gone: refused", !armed);
  tick(0, true);
  tick(androidWireThrust(1.0f), true);
  checkTrue("  zero again, then up: arms", armed);

  printf("scenario: HOLD mode, stick rest then up\n");
  resetSim(3.80f, MODE_HOLD);
  tick(0, true);
  tick(androidWireThrust(1.0f), true);
  checkTrue("  HOLD ignores the stick (TAKEOFF arms, not thrust-up)", !armed);

  printf("scenario: the web page Spin up stream (0.5 s at zero, 1 s ramp)\n");
  resetSim(3.80f, MODE_MANUAL);
  for (int k = 0; k < 25; k++) tick(0, true);   // streamHold = 25 zero frames
  checkTrue("  the 0.5 s zero hold does not arm", !armed);
  bool sawArm = false;
  for (uint16_t level = 1000; level <= 26812 && !sawArm; level = (uint16_t)(level + 1000)) {
    tick(level, true);
    sawArm = armed;
  }
  checkTrue("  the ramp arms once it passes 1000", sawArm);

  printf("\n%s\n", fails ? "COMMANDER MISMATCH PRESENT"
                         : "all commander tests passed");
  return fails ? 1 : 0;
}
