// Regression guard for SBUS frame acceptance (sbusFrameUsable in the sketch).
// Run with ./run.sh, which checks the copied lines still match the sketch.
//
// SBUS flags live in byte 23 (bit2 frame lost, bit3 failsafe); byte 24 is the
// footer. The old check read byte 24, so a receiver in failsafe -- which keeps
// sending frames with the last stick positions -- was never noticed.

#include <cstdint>
#include <cstdio>
#include <cstring>

// ---- copied verbatim from Esp32_drone.ino ----
bool sbusFrameUsable(const uint8_t *f) {
  if (f[0] != 0x0F) return false;
  const uint8_t footer = f[24];
  if (footer != 0x00 && (footer & 0x0F) != 0x04) return false;
  if (f[23] & 0x08) return false;      // receiver failsafe
  if (f[23] & 0x04) return false;      // this frame was lost
  return true;
}
// ----------------------------------------------

static bool oldCheck(const uint8_t *f) { return f[0] == 0x0F && (f[24] & 0x0F) <= 0x03; }

static void frame(uint8_t *f, uint8_t flags, uint8_t footer) {
  memset(f, 0x55, 25);
  f[0] = 0x0F; f[23] = flags; f[24] = footer;
}

static int fails = 0;
static void check(const char *what, bool want, bool got) {
  printf("%s %-46s\n", want == got ? "ok  " : "FAIL", what);
  if (want != got) fails++;
}

int main() {
  uint8_t f[25];
  frame(f, 0x00, 0x00); check("normal frame accepted", true, sbusFrameUsable(f));
  frame(f, 0x03, 0x00); check("digital ch17/18 bits set, still accepted", true, sbusFrameUsable(f));
  frame(f, 0x00, 0x14); check("SBUS2 footer 0x14 accepted", true, sbusFrameUsable(f));
  frame(f, 0x08, 0x00); check("receiver FAILSAFE rejected", false, sbusFrameUsable(f));
  frame(f, 0x0C, 0x00); check("failsafe + lost rejected", false, sbusFrameUsable(f));
  frame(f, 0x04, 0x00); check("lost frame rejected", false, sbusFrameUsable(f));
  frame(f, 0x00, 0x55); check("bad footer (misaligned) rejected", false, sbusFrameUsable(f));
  frame(f, 0x00, 0x00); f[0] = 0x00; check("bad header rejected", false, sbusFrameUsable(f));

  printf("\n--- negative control: the old byte-24 check ---\n");
  frame(f, 0x08, 0x00); check("old check ACCEPTED a failsafe frame", true, oldCheck(f));

  if (fails) { printf("\n%d sbus test(s) FAILED\n", fails); return 1; }
  printf("\nall sbus tests passed\n");
  return 0;
}
