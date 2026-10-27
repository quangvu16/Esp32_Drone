// Pins the CREATE_LOGCONFIG handler against the exact bytes Logg.create() puts
// on the wire, and the idempotency the client's resend queue depends on.
//
// Run with ./run.sh, which checks the copied logic still matches the sketch.
//
// Two things here are not obvious from the protocol:
//
// 1. Logg.create() builds its request in a fixed 31-byte ByteBuffer and sends
//    the whole backing array (CrtpPacket.toByteArray() serialises untrimmed),
//    so real <type><tocId> pairs are followed by zero padding. Log type codes
//    start at 1, so a zero type byte is unambiguous padding and must end the
//    list. Without that the padding parses as repeated element 0.
//
// 2. The client retransmits an unacknowledged command every 500 ms and never
//    gives up. A repeated CREATE for the same block has to re-ack OK; answering
//    LOG_ERR_EXISTS surfaces a bogus error to the user and clears nothing.

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ---- constants copied from Esp32_drone.ino ----
#define CRTP_MAX_SIZE     30
#define LOG_DATA_OVERHEAD 5
#define LOG_MAX_PAYLOAD   (CRTP_MAX_SIZE - LOG_DATA_OVERHEAD)
#define LOG_MAX_VARS      12

#define LT_UINT16  0x02
#define LT_FLOAT   0x07

#define LOG_ERR_OK        0
#define LOG_ERR_NO_ID     2
#define LOG_ERR_TOO_LARGE 7
#define LOG_ERR_EXISTS    17

typedef struct {
  const char *group;
  const char *name;
  uint8_t     ctype;
  uint8_t     size;
  uint8_t     access;
} TocEntry;

// LOG_TOC from Esp32_drone.ino — order and sizes are what the parser indexes.
static const TocEntry LOG_TOC[] = {
  { "pm",         "vbat",  LT_FLOAT,  4, 0x40 },
  { "stabilizer", "roll",  LT_FLOAT,  4, 0x40 },
  { "stabilizer", "pitch", LT_FLOAT,  4, 0x40 },
  { "stabilizer", "yaw",   LT_FLOAT,  4, 0x40 },
  { "controller", "roll",  LT_FLOAT,  4, 0x40 },
  { "controller", "pitch", LT_FLOAT,  4, 0x40 },
  { "controller", "yaw",   LT_FLOAT,  4, 0x40 },
  { "motor",      "m1",    LT_UINT16, 2, 0x40 },
  { "motor",      "m2",    LT_UINT16, 2, 0x40 },
  { "motor",      "m3",    LT_UINT16, 2, 0x40 },
  { "motor",      "m4",    LT_UINT16, 2, 0x40 },
};
#define LOG_TOC_COUNT ((uint8_t)(sizeof(LOG_TOC) / sizeof(LOG_TOC[0])))

// ---- a minimal stand-in for LogBlock ----
typedef struct {
  bool    used;
  uint8_t id;
  uint8_t nvars;
  uint8_t var_id[LOG_MAX_VARS];
} LogBlock;

#define LOG_MAX_BLOCKS 4
static LogBlock blocks[LOG_MAX_BLOCKS];

static LogBlock *logFind(uint8_t id) {
  for (int i = 0; i < LOG_MAX_BLOCKS; i++)
    if (blocks[i].used && blocks[i].id == id) return &blocks[i];
  return nullptr;
}
static LogBlock *logAlloc() {
  for (int i = 0; i < LOG_MAX_BLOCKS; i++)
    if (!blocks[i].used) return &blocks[i];
  return nullptr;
}
static void logReset() { memset(blocks, 0, sizeof(blocks)); }

// ---- the CREATE half of crtpHandleLogging, copied verbatim ----
// Returns the <err> byte that would go back to the client.
static int handleCreate(uint8_t id, const uint8_t *p, size_t len) {
  uint8_t  ids[LOG_MAX_VARS];
  uint8_t  n     = 0;
  uint16_t total = 0;
  bool     ok    = true;

  LogBlock *b = logFind(id);

  for (size_t i = 3; i + 1 < len; i += 2) {
    if (p[i] == 0) break;
    uint8_t tid = p[i + 1];
    if (tid >= LOG_TOC_COUNT || n >= LOG_MAX_VARS ||
        total + LOG_TOC[tid].size > LOG_MAX_PAYLOAD) {
      ok = false;
      break;
    }
    total += LOG_TOC[tid].size;
    ids[n++] = tid;
  }
  if (n == 0) ok = false;

  if (!ok) return LOG_ERR_TOO_LARGE;

  if (b) {
    if (b->nvars == n && memcmp(b->var_id, ids, n) == 0) return LOG_ERR_OK;
    return LOG_ERR_EXISTS;
  }

  b = logAlloc();
  if (!b) return 12;                        // LOG_ERR_NO_MEMORY
  memset(b, 0, sizeof(*b));
  for (uint8_t i = 0; i < n; i++) b->var_id[i] = ids[i];
  b->nvars = n;
  b->used  = true;
  b->id    = id;
  return LOG_ERR_OK;
}

// ---- client side: rebuild Logg.create()'s datagram exactly ----
static size_t buildCreate(uint8_t *out /* CRTP bytes incl. header */,
                          uint8_t logConfigId,
                          const uint8_t *tocIds, uint8_t nvars) {
  uint8_t bb[31];
  memset(bb, 0, sizeof(bb));          // JVM zero-fills the backing array
  bb[0] = 0;                          // CMD_CREATE_LOGCONFIG
  bb[1] = logConfigId;
  for (uint8_t i = 0; i < nvars; i++) {
    bb[2 + 2 * i]     = LOG_TOC[tocIds[i]].ctype;
    bb[2 + 2 * i + 1] = tocIds[i];
  }
  out[0] = 0x51;                      // port 5, channel 1
  memcpy(out + 1, bb, 31);            // sendLoggPacket sends ALL 31 bytes
  return 1 + 31;
}

static int fails = 0;

static void expect(const char *what, int want, int got) {
  if (want != got) {
    printf("FAIL %-52s want %d got %d\n", what, want, got);
    fails++;
  } else {
    printf("ok   %-52s (%d)\n", what, got);
  }
}

int main() {
  uint8_t pkt[64];

  logReset();

  // The app's real config: one variable, "Standard" = pm.vbat.
  {
    uint8_t v[] = {0};
    size_t len = buildCreate(pkt, 0, v, 1);
    expect("single pm.vbat accepted", LOG_ERR_OK, handleCreate(0, pkt, len));
    LogBlock *b = logFind(0);
    expect("  one element stored", 1, b ? b->nvars : -1);
    expect("  it is toc id 0", 0, b ? b->var_id[0] : -1);
  }

  // The padding trap: without the zero terminator the trailing bytes of the
  // 31-byte buffer parse as repeated element 0. Guard against the test passing
  // vacuously if the terminator ever stops being checked.
  {
    uint8_t v[] = {0};
    size_t len = buildCreate(pkt, 0, v, 1);
    int naive = 0;
    for (size_t i = 3; i + 1 < len; i += 2) naive++;   // no `if (p[i]==0) break`
    if (naive <= 1) {
      printf("FAIL regression guard: no padding, the test is vacuous\n");
      return 1;
    }
    printf("ok   padding is present (%d pairs seen without the terminator)\n", naive);
    LogBlock *b = logFind(0);
    expect("  padding was not stored", 1, b ? b->nvars : -1);
  }

  // Idempotency: the client resends every 500 ms until something matches.
  {
    uint8_t v[] = {0};
    size_t len = buildCreate(pkt, 0, v, 1);
    expect("retransmitted CREATE re-acks OK", LOG_ERR_OK, handleCreate(0, pkt, len));
    LogBlock *b = logFind(0);
    expect("  block was not duplicated", 1, b ? b->nvars : -1);
    expect("  still toc id 0", 0, b ? b->var_id[0] : -1);
  }

  // A genuinely different list on a used id is a real conflict.
  {
    uint8_t v[] = {1};
    size_t len = buildCreate(pkt, 0, v, 1);
    expect("CREATE with a different list -> EXISTS", LOG_ERR_EXISTS,
           handleCreate(0, pkt, len));
    LogBlock *b = logFind(0);
    expect("  the old block is untouched", 1, b ? b->nvars : -1);
    expect("  still toc id 0", 0, b ? b->var_id[0] : -1);
  }

  // Size ceiling: 7 floats + 4 uint16 = 36 > 25 bytes of payload.
  {
    logReset();
    uint8_t v[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    size_t len = buildCreate(pkt, 3, v, 11);
    expect("all 11 variables -> TOO_LARGE", LOG_ERR_TOO_LARGE,
           handleCreate(3, pkt, len));
    expect("  nothing was allocated", 0, logFind(3) ? 1 : 0);
  }

  // 5 floats (20) + 2 uint16 (4) = 24 bytes, which just fits.
  {
    logReset();
    uint8_t v[] = {0, 1, 2, 3, 4, 7, 8};
    size_t len = buildCreate(pkt, 9, v, 7);
    expect("5 floats + 2 u16 (24 B) accepted", LOG_ERR_OK, handleCreate(9, pkt, len));
    LogBlock *b = logFind(9);
    expect("  seven elements", 7, b ? b->nvars : -1);
    // and the same request again must still be OK
    expect("  retransmission re-acks OK", LOG_ERR_OK, handleCreate(9, pkt, len));
    expect("  still seven elements", 7, logFind(9)->nvars);
  }

  // Independent ids do not collide.
  {
    logReset();
    uint8_t a[] = {0}, bb[] = {1};
    uint8_t pa[64], pb[64];
    size_t la = buildCreate(pa, 0, a, 1), lb = buildCreate(pb, 1, bb, 1);
    expect("block 0 created", LOG_ERR_OK, handleCreate(0, pa, la));
    expect("block 1 created", LOG_ERR_OK, handleCreate(1, pb, lb));
    expect("  block 0 is toc 0", 0, logFind(0)->var_id[0]);
    expect("  block 1 is toc 1", 1, logFind(1)->var_id[0]);
  }

  printf("\n%s\n", fails ? "FAILURES PRESENT" : "all log-config tests passed");
  return fails ? 1 : 0;
}
