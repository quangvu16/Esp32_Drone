// Regression guard for the BLE control-byte segment reassembly.
// Run with ./run.sh, which also checks these functions still match the sketch.
// Chốt chặn hồi quy cho phần lắp ráp đoạn control byte của BLE.
// Chạy bằng ./run.sh, công cụ này cũng kiểm tra các hàm còn khớp với sketch.
//
// PROTOCOL.md §6: a CRTP datagram over BLE is split into 20-byte segments,
// seg[0] = bit7 start | bits6..5 pid | bits4..0 (totalLen-1), then up to 19
// CRTP bytes. The phone apps, the Python tuner and the firmware all speak it,
// so the reassembly state machine is pinned here: the 20-byte datagram edge
// (a packet that cannot fit one segment), pid matching, and the rule that the
// length field — not the write length — decides what reaches the parser.
// PROTOCOL.md mục 6: một gói CRTP qua BLE được tách thành các đoạn 20 byte,
// seg[0] = bit7 bắt đầu | bits6..5 pid | bits4..0 (tổng độ dài-1), rồi tối đa
// 19 byte CRTP. App điện thoại, tuner Python và firmware đều nói chuẩn này,
// nên trạng thái lắp ráp được chốt ở đây: mép gói 20 byte (gói không thể gọn
// trong một đoạn), khớp pid, và quy tắc trường độ dài — chứ không phải độ dài
// lần ghi — quyết định số byte đến bộ phân tích.

#include <cstdint>
#include <cstdio>
#include <cstring>

// ==== copied verbatim from Esp32_drone.ino (run.sh grep-guards these) ======
// ==== chép nguyên văn từ Esp32_drone.ino (run.sh grep-guards các dòng này) ==

#define CRTP_MAX_SIZE     30              // largest CRTP packet we handle

uint8_t  ble_rx_buf[CRTP_MAX_SIZE];
size_t   ble_rx_len    = 0;
uint8_t  ble_rx_pid    = 0xFF;

// Reassemble one control-byte segment into ble_rx_buf (see PROTOCOL.md §6).
//   ctrl: bit7 start | bits6..5 pid | bits4..0 (totalLen - 1)
//   pay:  the CRTP bytes after the control byte
//   n:    how many the client wrote
// Returns the assembled CRTP length when a packet just completed, else 0.
// The length field is authoritative: dispatch what was announced rather than
// what happened to arrive, so padding cannot reach the parser. A packet is
// single-segment when the whole thing fits here, NOT merely when that field
// reads < 20: the field saturates at 32 and a 20-byte datagram would
// otherwise be mistaken for a single segment it cannot fit into.
// (Copied verbatim by test/bleseg_test.cpp.)
size_t bleRxSeg(uint8_t ctrl, const uint8_t *pay, size_t n) {
  bool    start = (ctrl & 0x80) != 0;
  uint8_t pid   = (uint8_t)((ctrl >> 5) & 0x03);
  size_t  total = (size_t)((ctrl & 0x1F) + 1);

  if (start) {
    ble_rx_len = 0;
    ble_rx_pid = pid;
    if (n > sizeof(ble_rx_buf)) return 0;
    memcpy(ble_rx_buf, pay, n);
    ble_rx_len = n;
    if (total <= n) {                 // single-segment packet
      ble_rx_len = 0;
      return total;
    }
    return 0;
  }
  if (pid != ble_rx_pid || ble_rx_len == 0) { ble_rx_len = 0; return 0; }
  if (ble_rx_len + n > sizeof(ble_rx_buf))   { ble_rx_len = 0; return 0; }
  memcpy(ble_rx_buf + ble_rx_len, pay, n);
  ble_rx_len += n;
  if (ble_rx_len >= total) {
    ble_rx_len = 0;
    return total;
  }
  return 0;
}

// ===========================================================================

static int fails = 0;

static void check(const char *what, bool ok) {
  printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok) fails++;
}

// ctrl byte for one segment: start bit, pid, announced total length
// byte ctrl cho một đoạn: bit bắt đầu, pid, tổng độ dài đã báo
static uint8_t ctrlOf(bool start, uint8_t pid, size_t total) {
  return (uint8_t)((start ? 0x80 : 0x00) | ((pid & 0x03) << 5) |
                   ((total - 1) & 0x1F));
}

int main() {
  uint8_t pay[40];
  for (size_t i = 0; i < sizeof(pay); i++) pay[i] = (uint8_t)(0xA0 + i);

  {
    // A 15-byte CRTP packet fits one segment: start|pid0|len-1=14.
    // Một gói CRTP 15 byte gọn trong một đoạn: start|pid0|len-1=14.
    check("single-segment packet returns its length",
          bleRxSeg(ctrlOf(true, 0, 15), pay, 15) == 15);
    check("the packet sits at the buffer head",
          memcmp(ble_rx_buf, pay, 15) == 0);
    check("state resets after dispatch", ble_rx_len == 0);
    // A continuation after the reset is refused.
    // Đoạn tiếp tục sau khi reset bị từ chối.
    check("continuation after a completed packet is refused",
          bleRxSeg(ctrlOf(false, 0, 15), pay, 5) == 0);
  }
  {
    // Two segments: 24 bytes = 19 + 5, same pid.
    // Hai đoạn: 24 byte = 19 + 5, cùng pid.
    check("start segment waits for the rest",
          bleRxSeg(ctrlOf(true, 1, 24), pay, 19) == 0);
    check("continuation completes the packet",
          bleRxSeg(ctrlOf(false, 1, 24), pay + 19, 5) == 24);
    check("both halves reassemble in order", memcmp(ble_rx_buf, pay, 24) == 0);
  }
  {
    // The 20-byte datagram edge: total 20 cannot fit one 19-byte segment, so
    // the first segment must NOT complete it even though the field (19) reads
    // like a plausible segment length.
    // Mép gói 20 byte: tổng 20 không thể gọn trong một đoạn 19 byte, nên đoạn
    // đầu KHÔNG được kết thúc gói dù trường (19) trông như độ dài đoạn hợp lệ.
    check("a 20-byte packet is not mistaken for single-segment",
          bleRxSeg(ctrlOf(true, 2, 20), pay, 19) == 0);
    check("its last byte completes it", bleRxSeg(ctrlOf(false, 2, 20), pay + 19, 1) == 20);
  }
  {
    // A full 30-byte CRTP packet in two segments (19 + 11).
    // Một gói CRTP đầy đủ 30 byte qua hai đoạn (19 + 11).
    check("start of a 30-byte packet waits",
          bleRxSeg(ctrlOf(true, 3, 30), pay, 19) == 0);
    check("30-byte packet completes", bleRxSeg(ctrlOf(false, 3, 30), pay + 19, 11) == 30);
  }
  {
    // The length field is authoritative: a write padded beyond the announced
    // total dispatches only `total` bytes.
    // Trường độ dài là nguồn quyết định: lần ghi có phần đệm vượt tổng đã báo
    // chỉ xử lý đúng `total` byte.
    check("padding beyond the announced length is ignored",
          bleRxSeg(ctrlOf(true, 0, 4), pay, 19) == 4);
    check("dispatched length is the announced one",
          memcmp(ble_rx_buf, pay, 4) == 0 && ble_rx_len == 0);
  }
  {
    // pid mismatch drops the partial packet instead of splicing two together.
    // Sai pid loại gói dở dang thay vì ghép hai gói vào nhau.
    bleRxSeg(ctrlOf(true, 0, 24), pay, 19);
    check("continuation with another pid is refused",
          bleRxSeg(ctrlOf(false, 1, 24), pay + 19, 5) == 0);
    check("and the partial packet is gone", ble_rx_len == 0);
  }
  {
    // A continuation without a start carries no state.
    // Đoạn tiếp tục mà không có đoạn đầu thì không có trạng thái.
    check("continuation without start is refused",
          bleRxSeg(ctrlOf(false, 0, 24), pay, 5) == 0);
  }
  {
    // Guards: oversize single write, and accumulation past the buffer.
    // Bảo vệ: lần ghi đơn quá lớn, và tích lũy vượt bộ đệm.
    check("an oversize write is refused",
          bleRxSeg(ctrlOf(true, 0, 31), pay, 31) == 0);
    check("accumulation past CRTP_MAX_SIZE resets",
          bleRxSeg(ctrlOf(true, 0, 30), pay, 19) == 0 &&
          bleRxSeg(ctrlOf(false, 0, 30), pay + 19, 12) == 0 && ble_rx_len == 0);
  }

  printf("\n%s\n", fails ? "BLE SEGMENT MISMATCH PRESENT" : "all ble segment tests passed");
  return fails ? 1 : 0;
}
