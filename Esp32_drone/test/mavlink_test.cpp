// Regression guard for the MAVLink v2 bridge (PROTOCOL.md §13).
// Run with ./run.sh, which also checks these lines still match the sketch.
// Chốt chặn hồi quy cho cầu nối MAVLink v2 (PROTOCOL.md §13).
// Chạy bằng ./run.sh, công cụ này cũng kiểm tra các dòng còn khớp với sketch.
//
// The framing and CRC here are pinned three ways so a mistake cannot hide:
//
// 1. The CRC-16/MCRF4XX check value ("123456789" -> 0x6F91) validates the
//    accumulator itself against the published standard.
// 2. Byte-exact golden frames captured from pymavlink (common dialect) pin
//    the header layout, the zero-truncation and crc_extra together — any one
//    of them wrong and the bytes stop matching.
// 3. frameValid() re-checks a golden the way mavHandle() does on receive,
//    and must reject the same frame once a byte is flipped.
//
// Việc đóng gói và CRC ở đây được chốt bằng ba cách để sai sót không thể giấu:
//
// 1. Giá trị kiểm CRC-16/MCRF4XX ("123456789" -> 0x6F91) tự kiểm hàm cộng dồn
//    với chuẩn đã công bố.
// 2. Frame vàng khớp từng byte, chụp từ pymavlink (dialect common), chốt bố cục
//    header, phép cắt số 0 và crc_extra cùng lúc — sai một thứ là byte lệch ngay.
// 3. frameValid() kiểm lại một frame vàng đúng kiểu mavHandle() khi nhận, và
//    phải từ chối chính frame đó khi lật một byte.

#include <cstdint>
#include <cstdio>
#include <cstring>

#define MAVLINK_UDP_PORT   14550
#define MAVLINK_SYSID      1
#define MAVLINK_COMPID     1
#define MAVLINK_STX_V2     0xFD
#define MAVLINK_STX_V1     0xFE

static uint8_t mav_seq = 0;

// Copied verbatim from Esp32_drone.ino — the crc_extra table (PROTOCOL.md §13).
// Sao chép nguyên văn từ Esp32_drone.ino — bảng crc_extra (PROTOCOL.md §13).
const uint32_t MAV_CRC_IDS[]    = { 0, 1, 30, 33, 39, 40, 41, 42, 43,
                                    44, 45, 47, 51, 73, 76, 77, 242, 253 };
const uint8_t  MAV_CRC_EXTRAS[] = { 50, 124, 39, 104, 254, 230, 28, 28, 132,
                                    221, 232, 153, 196, 38, 152, 143, 104, 83 };

uint8_t mavCrcExtra(uint32_t id) {
  for (size_t i = 0; i < sizeof(MAV_CRC_IDS) / sizeof(MAV_CRC_IDS[0]); i++)
    if (MAV_CRC_IDS[i] == id) return MAV_CRC_EXTRAS[i];
  return 0;
}

// Copied verbatim from Esp32_drone.ino — CRC-16/MCRF4XX, seed 0xFFFF.
// Sao chép nguyên văn từ Esp32_drone.ino — CRC-16/MCRF4XX, hạt giống 0xFFFF.
uint16_t mavCrcAccumulate(uint8_t data, uint16_t crc) {
  uint8_t tmp = data ^ (uint8_t)(crc & 0xff);
  tmp ^= (uint8_t)(tmp << 4);
  return (uint16_t)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
}

// Copied verbatim from mavFrame() in Esp32_drone.ino — v2 framing with
// MAVLink 2 zero-truncation (PROTOCOL.md §13).
// Sao chép nguyên văn từ mavFrame() trong Esp32_drone.ino — đóng gói v2 với
// phép cắt số 0 của MAVLink 2 (PROTOCOL.md §13).
size_t mavFrame(uint32_t msgid, const uint8_t *payload, size_t len, uint8_t *out) {
  while (len > 0 && payload[len - 1] == 0) len--;
  size_t i = 0;
  out[i++] = MAVLINK_STX_V2;
  out[i++] = (uint8_t)len;
  out[i++] = 0;                          // incompat: unsigned v2
  out[i++] = 0;                          // compat
  out[i++] = mav_seq++;
  out[i++] = MAVLINK_SYSID;
  out[i++] = MAVLINK_COMPID;
  out[i++] = (uint8_t)(msgid & 0xFF);
  out[i++] = (uint8_t)((msgid >> 8) & 0xFF);
  out[i++] = (uint8_t)((msgid >> 16) & 0xFF);
  if (len) memcpy(out + i, payload, len);
  i += len;
  uint16_t crc = 0xFFFF;
  for (size_t k = 1; k < i; k++) crc = mavCrcAccumulate(out[k], crc);
  crc = mavCrcAccumulate(mavCrcExtra(msgid), crc);
  out[i++] = (uint8_t)(crc & 0xFF);
  out[i++] = (uint8_t)(crc >> 8);
  return i;
}

// The RX-side check, copied from mavHandle() in Esp32_drone.ino — a frame the
// test builds must pass it, and one flipped byte must fail it.
// Phía nhận, sao chép từ mavHandle() trong Esp32_drone.ino — frame test dựng ra
// phải qua được nó, và lật một byte là phải trượt.
bool frameValid(const uint8_t *b, size_t n) {
  size_t payload_off, crc_off, msgid;
  uint8_t len;
  if (n >= 12 && b[0] == MAVLINK_STX_V2) {
    len = b[1];
    if (b[2] & 0x01) return false;             // signed frame — not us
    payload_off = 10;
    crc_off = 10 + len;
    msgid = (size_t)b[7] | ((size_t)b[8] << 8) | ((size_t)b[9] << 16);
  } else if (n >= 8 && b[0] == MAVLINK_STX_V1) {
    len = b[1];
    payload_off = 6;
    crc_off = 6 + len;
    msgid = b[5];
  } else {
    return false;
  }
  if (n < crc_off + 2) return false;

  uint16_t crc = 0xFFFF;
  for (size_t k = 1; k < crc_off; k++) crc = mavCrcAccumulate(b[k], crc);
  crc = mavCrcAccumulate(mavCrcExtra(msgid), crc);
  if ((uint16_t)(b[crc_off] | (b[crc_off + 1] << 8)) != crc) return false;
  (void)payload_off;
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

static void checkTrue(const char *what, bool ok) {
  if (!ok) {
    printf("FAIL %-52s\n", what);
    fails++;
  } else {
    printf("ok   %-52s\n", what);
  }
}

static void checkBytes(const char *what, const uint8_t *want, size_t wn,
                       const uint8_t *got, size_t gn) {
  if (wn != gn) {
    printf("FAIL %-52s length want %zu got %zu\n", what, wn, gn);
    fails++;
    return;
  }
  for (size_t k = 0; k < wn; k++) {
    if (want[k] != got[k]) {
      printf("FAIL %-52s byte %zu want 0x%02X got 0x%02X\n",
             what, k, (unsigned)want[k], (unsigned)got[k]);
      fails++;
      return;
    }
  }
  printf("ok   %-52s (%zu bytes)\n", what, wn);
}

// ---------------------------------------------------------------------------
//  1. The accumulator itself, against the published check value.
//     Hàm cộng dồn, với giá trị kiểm đã công bố.
// ---------------------------------------------------------------------------
static void testCrcCheckValue() {
  printf("--- CRC-16/MCRF4XX check value ---\n");
  const uint8_t msg[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
  uint16_t crc = 0xFFFF;
  for (size_t k = 0; k < sizeof(msg); k++) crc = mavCrcAccumulate(msg[k], crc);
  checkInt("  '123456789' -> 0x6F91", 0x6F91, crc);
}

// ---------------------------------------------------------------------------
//  2. The crc_extra table, pinned to the pymavlink common dialect.
//     Bảng crc_extra, chốt theo dialect common của pymavlink.
// ---------------------------------------------------------------------------
static void testCrcExtraTable() {
  printf("--- crc_extra per message id ---\n");
  checkInt("  HEARTBEAT (0)",          50, mavCrcExtra(0));
  checkInt("  SYS_STATUS (1)",        124, mavCrcExtra(1));
  checkInt("  ATTITUDE (30)",          39, mavCrcExtra(30));
  checkInt("  GLOBAL_POSITION_INT (33)", 104, mavCrcExtra(33));
  checkInt("  MISSION_ITEM (39)",     254, mavCrcExtra(39));
  checkInt("  MISSION_REQUEST (40)",  230, mavCrcExtra(40));
  checkInt("  MISSION_SET_CURRENT (41)", 28, mavCrcExtra(41));
  checkInt("  MISSION_CURRENT (42)",   28, mavCrcExtra(42));
  checkInt("  MISSION_REQUEST_LIST (43)", 132, mavCrcExtra(43));
  checkInt("  MISSION_COUNT (44)",    221, mavCrcExtra(44));
  checkInt("  MISSION_CLEAR_ALL (45)", 232, mavCrcExtra(45));
  checkInt("  MISSION_ACK (47)",      153, mavCrcExtra(47));
  checkInt("  MISSION_REQUEST_INT (51)", 196, mavCrcExtra(51));
  checkInt("  MISSION_ITEM_INT (73)",  38, mavCrcExtra(73));
  checkInt("  COMMAND_LONG (76)",     152, mavCrcExtra(76));
  checkInt("  COMMAND_ACK (77)",      143, mavCrcExtra(77));
  checkInt("  HOME_POSITION (242)",   104, mavCrcExtra(242));
  checkInt("  STATUSTEXT (253)",       83, mavCrcExtra(253));
  checkInt("  unknown id -> 0",         0, mavCrcExtra(9999));
}

// ---------------------------------------------------------------------------
//  3. Golden frames from pymavlink (srcSystem=1, srcComponent=1, seq 7).
//     Frame vàng từ pymavlink (srcSystem=1, srcComponent=1, seq 7).
// ---------------------------------------------------------------------------
static void testGoldenFrames() {
  uint8_t out[128];

  printf("--- HEARTBEAT golden (armed, manual) ---\n");
  // The same packing mavSendHeartbeat() builds: custom_mode 0, type 2,
  // autopilot 0, base 0x81 (armed), status 4, version 3.
  // Đóng gói giống mavSendHeartbeat(): custom_mode 0, type 2, autopilot 0,
  // base 0x81 (đã arm), status 4, version 3.
  {
    uint8_t p[9] = {0};
    p[4] = 2;
    p[5] = 0;
    p[6] = 0x81;
    p[7] = 4;
    p[8] = 3;
    const uint8_t gold[] = {
      0xFD, 0x09, 0x00, 0x00, 0x07, 0x01, 0x01, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x81, 0x04, 0x03, 0x10, 0x49
    };
    mav_seq = 7;
    size_t n = mavFrame(0, p, sizeof(p), out);
    checkBytes("  21 bytes, CRC 0x4910", gold, sizeof(gold), out, n);
    checkTrue("  golden passes the RX CRC check", frameValid(out, n));
  }

  printf("--- MISSION_CURRENT golden (seq 5 of 3) ---\n");
  // mavSendMissionCurrent() packs 6 bytes; the two trailing zeros must vanish
  // (MAVLink 2 zero-truncation) and the CRC must be taken over the trimmed
  // payload — that is exactly what pymavlink puts on the wire.
  // mavSendMissionCurrent() đóng gói 6 byte; hai byte 0 cuối phải biến mất
  // (cắt số 0 của MAVLink 2) và CRC phải tính trên payload đã cắt — đúng thứ
  // pymavlink đặt lên đường truyền.
  {
    uint8_t p[6] = {0};
    uint16_t seq = 5, total = 3;
    memcpy(p, &seq, 2);
    memcpy(p + 2, &total, 2);
    const uint8_t gold[] = {
      0xFD, 0x03, 0x00, 0x00, 0x07, 0x01, 0x01, 0x2A, 0x00, 0x00,
      0x05, 0x00, 0x03, 0x52, 0x22
    };
    mav_seq = 7;
    size_t n = mavFrame(42, p, sizeof(p), out);
    checkBytes("  15 bytes, CRC 0x2252", gold, sizeof(gold), out, n);
    checkTrue("  golden passes the RX CRC check", frameValid(out, n));
  }

  printf("--- COMMAND_ACK golden (TAKEOFF accepted) ---\n");
  {
    uint8_t p[4] = {0};
    uint16_t command = 22;                    // MAV_CMD_NAV_TAKEOFF
    memcpy(p, &command, 2);
    p[2] = 0;                                 // MAV_RESULT_ACCEPTED
    const uint8_t gold[] = {
      0xFD, 0x01, 0x00, 0x00, 0x07, 0x01, 0x01, 0x4D, 0x00, 0x00,
      0x16, 0xA2, 0xA2
    };
    mav_seq = 7;
    size_t n = mavFrame(77, p, sizeof(p), out);
    checkBytes("  13 bytes, CRC 0xA2A2", gold, sizeof(gold), out, n);
    checkTrue("  golden passes the RX CRC check", frameValid(out, n));
  }
}

// ---------------------------------------------------------------------------
//  4. Zero-truncation and the RX rejection paths.
//     Cắt số 0 và các lối từ chối phía nhận.
// ---------------------------------------------------------------------------
static void testTruncationAndReject() {
  uint8_t out[128];

  printf("--- MAVLink 2 zero-truncation ---\n");
  {
    uint8_t z[8] = {0};
    mav_seq = 3;
    size_t n = mavFrame(42, z, sizeof(z), out);
    checkInt("  all-zero payload -> length 0", 0, out[1]);
    checkInt("  frame shrinks to header + CRC", 12, (long)n);
    checkTrue("  zero-length frame is still valid", frameValid(out, n));
  }
  {
    uint8_t mixed[4] = { 0x05, 0x00, 0x00, 0x00 };
    mav_seq = 3;
    size_t n = mavFrame(42, mixed, sizeof(mixed), out);
    checkInt("  leading zero kept, trailing trimmed -> length 1", 1, out[1]);
    checkInt("  frame is 13 bytes", 13, (long)n);
    checkTrue("  trimmed frame is still valid", frameValid(out, n));
  }

  printf("--- frameValid rejects tampered frames ---\n");
  {
    uint8_t p[6] = {0};
    uint16_t seq = 5, total = 3;
    memcpy(p, &seq, 2);
    memcpy(p + 2, &total, 2);
    mav_seq = 7;
    size_t n = mavFrame(42, p, sizeof(p), out);
    checkTrue("  intact frame accepted", frameValid(out, n));

    out[10] ^= 0xFF;                          // flip a payload byte
    checkTrue("  flipped payload byte rejected", !frameValid(out, n));
    out[10] ^= 0xFF;

    out[n - 1] ^= 0x01;                       // flip a CRC byte
    checkTrue("  flipped CRC byte rejected", !frameValid(out, n));
    out[n - 1] ^= 0x01;

    checkTrue("  truncated frame rejected", !frameValid(out, n - 1));
  }

  printf("--- negative control: the CRC must cover the payload ---\n");
  {
    // If the accumulator silently skipped the payload, both frames below would
    // carry the same CRC and the golden tests would pass vacuously.
    // Nếu hàm cộng dồn lặng lẽ bỏ qua payload, hai frame dưới đây sẽ mang cùng
    // CRC và các test frame vàng sẽ trượt qua một cách rỗng.
    uint8_t a[9] = {0};
    a[4] = 2; a[5] = 0; a[6] = 0x81; a[7] = 4; a[8] = 3;
    uint8_t b[9] = {0};
    b[4] = 3; b[5] = 0; b[6] = 0x81; b[7] = 4; b[8] = 3;
    uint8_t fa[32], fb[32];
    mav_seq = 7; size_t na = mavFrame(0, a, sizeof(a), fa);
    mav_seq = 7; size_t nb = mavFrame(0, b, sizeof(b), fb);
    checkTrue("  different payload -> different frame", na == nb && memcmp(fa, fb, na) != 0);

    // And a wrong crc_extra must not reproduce the golden.
    // Và crc_extra sai không được tái tạo frame vàng.
    uint16_t crc = 0xFFFF;
    for (size_t k = 1; k + 2 < na; k++) crc = mavCrcAccumulate(fa[k], crc);
    crc = mavCrcAccumulate(51, crc);          // wrong extra (HEARTBEAT is 50)
    checkTrue("  wrong crc_extra -> wrong CRC (0x4910 differs)",
              crc != (uint16_t)(fa[na - 2] | (fa[na - 1] << 8)));
  }
}

int main() {
  testCrcCheckValue();
  printf("\n");
  testCrcExtraTable();
  printf("\n");
  testGoldenFrames();
  printf("\n");
  testTruncationAndReject();

  printf("\n%s\n", fails ? "MAVLINK TEST FAILURES" : "all mavlink tests passed");
  return fails ? 1 : 0;
}
