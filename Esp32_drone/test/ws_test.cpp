// Regression guard for the WebSocket handshake and frame codec.
// Run with ./run.sh, which also checks these functions still match the sketch.
// Chốt chặn hồi quy cho phần bắt tay và đóng khung WebSocket.
// Chạy bằng ./run.sh, công cụ này cũng kiểm tra các hàm còn khớp với sketch.
//
// The web control page (PROTOCOL.md §8) rides on a hand-rolled RFC 6455
// subset: SHA-1 + base64 for the accept key, and 7-bit-length frames. The
// handshake math has golden vectors from RFC 6455 itself; the frame codec is
// pinned against the exact bytes a browser sends — masked, FIN, opcode 2 —
// because that is the only shape crtpReceiveFramed() will ever see from it.
// Trang điều khiển web (PROTOCOL.md mục 8) chạy trên một tập con RFC 6455
// viết tay: SHA-1 + base64 cho khóa chấp nhận, và khung độ dài 7 bit. Bắt tay
// có vector vàng từ chính RFC 6455; khung được chốt đúng byte trình duyệt gửi
// — có mặt nạ, FIN, opcode 2 — vì đó là dạng duy nhất crtpReceiveFramed()
// từng thấy từ nó.

#include <cstdint>
#include <cstdio>
#include <cstring>

// ==== copied verbatim from Esp32_drone.ino (run.sh grep-guards these) ======
// ==== chép nguyên văn từ Esp32_drone.ino (run.sh grep-guards các dòng này) ==

typedef struct {
  uint32_t h[5];
  uint64_t len;                        // total bytes fed
  uint8_t  block[64];
  size_t   used;
} WsSha1;

uint32_t wsRol(uint32_t v, int n) { return (v << n) | (v >> (32 - n)); }

void wsSha1Init(WsSha1 *s) {
  s->h[0] = 0x67452301;
  s->h[1] = 0xEFCDAB89;
  s->h[2] = 0x98BADCFE;
  s->h[3] = 0x10325476;
  s->h[4] = 0xC3D2E1F0;
  s->len = 0;
  s->used = 0;
}

void wsSha1Block(WsSha1 *s, const uint8_t p[64]) {
  uint32_t w[80];
  for (int i = 0; i < 16; i++)
    w[i] = ((uint32_t)p[i * 4] << 24) | ((uint32_t)p[i * 4 + 1] << 16) |
           ((uint32_t)p[i * 4 + 2] << 8) | (uint32_t)p[i * 4 + 3];
  for (int i = 16; i < 80; i++)
    w[i] = wsRol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
  uint32_t a = s->h[0], b = s->h[1], c = s->h[2], d = s->h[3], e = s->h[4];
  for (int i = 0; i < 80; i++) {
    uint32_t f, k;
    if (i < 20)      { f = (b & c) | (~b & d);          k = 0x5A827999; }
    else if (i < 40) { f = b ^ c ^ d;                   k = 0x6ED9EBA1; }
    else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
    else             { f = b ^ c ^ d;                   k = 0xCA62C1D6; }
    uint32_t t = wsRol(a, 5) + f + e + k + w[i];
    e = d; d = c; c = wsRol(b, 30); b = a; a = t;
  }
  s->h[0] += a; s->h[1] += b; s->h[2] += c; s->h[3] += d; s->h[4] += e;
}

void wsSha1Update(WsSha1 *s, const uint8_t *p, size_t n) {
  s->len += n;
  while (n > 0) {
    size_t take = 64 - s->used;
    if (take > n) take = n;
    memcpy(s->block + s->used, p, take);
    s->used += take;
    p += take;
    n -= take;
    if (s->used == 64) { wsSha1Block(s, s->block); s->used = 0; }
  }
}

void wsSha1Final(WsSha1 *s, uint8_t out[20]) {
  uint64_t bits = s->len * 8;
  uint8_t pad = 0x80, zero = 0;
  wsSha1Update(s, &pad, 1);
  while (s->used != 56) wsSha1Update(s, &zero, 1);
  uint8_t lenb[8];
  for (int i = 0; i < 8; i++) lenb[i] = (uint8_t)(bits >> (56 - 8 * i));
  wsSha1Update(s, lenb, 8);
  for (int i = 0; i < 5; i++) {
    out[i * 4]     = (uint8_t)(s->h[i] >> 24);
    out[i * 4 + 1] = (uint8_t)(s->h[i] >> 16);
    out[i * 4 + 2] = (uint8_t)(s->h[i] >> 8);
    out[i * 4 + 3] = (uint8_t)(s->h[i]);
  }
}

static const char WS_B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void wsBase64(const uint8_t *in, size_t n, char *out) {
  size_t k = 0;
  for (size_t i = 0; i < n; i += 3) {
    uint32_t v = (uint32_t)in[i] << 16;
    if (i + 1 < n) v |= (uint32_t)in[i + 1] << 8;
    if (i + 2 < n) v |= in[i + 2];
    out[k++] = WS_B64[(v >> 18) & 0x3F];
    out[k++] = WS_B64[(v >> 12) & 0x3F];
    out[k++] = (i + 1 < n) ? WS_B64[(v >> 6) & 0x3F] : '=';
    out[k++] = (i + 2 < n) ? WS_B64[v & 0x3F] : '=';
  }
  out[k] = '\0';
}

static const char WS_GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

// Sec-WebSocket-Accept = base64(SHA1(key + GUID)) — RFC 6455 §4.2.2.
void wsAcceptKey(const char *key, char out[29]) {
  WsSha1 s;
  uint8_t dig[20];
  wsSha1Init(&s);
  wsSha1Update(&s, (const uint8_t *)key, strlen(key));
  wsSha1Update(&s, (const uint8_t *)WS_GUID, 36);
  wsSha1Final(&s, dig);
  wsBase64(dig, 20, out);
}

#define WS_MAX_MSG 64                  // room for a CRTP packet + checksum

// >0: a complete frame of that length is at the head of acc
//  0: need more bytes
// -1: malformed — drop the connection
int wsFrameLen(const uint8_t *acc, size_t n) {
  if (n < 2) return 0;
  uint8_t b1 = acc[1];
  size_t len = b1 & 0x7F;
  if (len == 126 || len == 127) return -1;   // jumbo frames are refused
  if (len > WS_MAX_MSG) return -1;
  size_t off = 2 + ((b1 & 0x80) ? 4 : 0);
  if (n < off + len) return 0;
  return (int)(off + len);
}

uint8_t wsFrameTake(const uint8_t *acc, uint8_t *out, size_t *out_len) {
  uint8_t b0 = acc[0], b1 = acc[1];
  size_t len = b1 & 0x7F, off = 2;
  const uint8_t *mask = nullptr;
  if (b1 & 0x80) { mask = acc + off; off += 4; }
  for (size_t i = 0; i < len; i++)
    out[i] = mask ? (uint8_t)(acc[off + i] ^ mask[i & 3]) : acc[off + i];
  *out_len = len;
  return b0 & 0x0F;
}

size_t wsFramePut(uint8_t *out, uint8_t opcode, const uint8_t *payload, size_t len) {
  out[0] = (uint8_t)(0x80 | opcode);   // FIN + opcode
  out[1] = (uint8_t)len;               // len < 126 always holds here
  memcpy(out + 2, payload, len);
  return 2 + len;
}

// ==== end of the copied code ==============================================

static int fails = 0;

static void check(const char *what, bool ok) {
  printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok) fails++;
}

static void checkSha1(const char *label, const char *msg, const char *want_hex) {
  WsSha1 s;
  uint8_t dig[20];
  wsSha1Init(&s);
  wsSha1Update(&s, (const uint8_t *)msg, strlen(msg));
  wsSha1Final(&s, dig);
  char hex[41];
  for (int i = 0; i < 20; i++) sprintf(hex + i * 2, "%02x", dig[i]);
  bool ok = strcmp(hex, want_hex) == 0;
  printf("%s sha1(%s) = %s\n", ok ? "ok  " : "FAIL", label, hex);
  if (!ok) {
    printf("     want %s\n", want_hex);
    fails++;
  }
}

static void checkB64(const char *in, const char *want) {
  char out[64];
  wsBase64((const uint8_t *)in, strlen(in), out);
  bool ok = strcmp(out, want) == 0;
  printf("%s base64(\"%s\") = \"%s\"\n", ok ? "ok  " : "FAIL", in, out);
  if (!ok) {
    printf("     want \"%s\"\n", want);
    fails++;
  }
}

// Build a masked client frame the way a browser does: FIN + opcode, mask bit
// set, 4-byte key, payload XORed with the key (test-local encoder).
// Dựng khung client có mặt nạ theo cách trình duyệt làm: FIN + opcode, bit mặt
// nạ bật, khóa 4 byte, payload XOR với khóa (bộ mã cục bộ của test).
static size_t clientFrame(uint8_t *out, uint8_t opcode, const uint8_t *payload,
                          size_t len, const uint8_t key[4]) {
  out[0] = (uint8_t)(0x80 | opcode);
  out[1] = (uint8_t)(0x80 | len);
  memcpy(out + 2, key, 4);
  for (size_t i = 0; i < len; i++) out[6 + i] = (uint8_t)(payload[i] ^ key[i & 3]);
  return 6 + len;
}

int main() {
  printf("--- sha1 golden vectors (FIPS 180-2 / RFC 3174) ---\n");
  checkSha1("empty", "", "da39a3ee5e6b4b0d3255bfef95601890afd80709");
  checkSha1("\"abc\"", "abc", "a9993e364706816aba3e25717850c26c9cd0d89d");
  checkSha1("448-bit", "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
            "84983e441c3bd26ebaae4aa1f95129e5e54670f1");
  {
    // 1,000,000 x 'a' — exercises the block path hard.
    // 1.000.000 lần 'a' — ép mạnh nhánh xử lý block.
    WsSha1 s;
    uint8_t dig[20], buf[1000];
    memset(buf, 'a', sizeof(buf));
    wsSha1Init(&s);
    for (int i = 0; i < 1000; i++) wsSha1Update(&s, buf, sizeof(buf));
    wsSha1Final(&s, dig);
    char hex[41];
    for (int i = 0; i < 20; i++) sprintf(hex + i * 2, "%02x", dig[i]);
    check("sha1(1e6 x 'a') = 34aa973cd4c4daa4f61eeb2bdbad27316534016f",
          strcmp(hex, "34aa973cd4c4daa4f61eeb2bdbad27316534016f") == 0);
  }
  {
    // Chunked feeding must match one-shot feeding (the handshake feeds key
    // and GUID as two writes).
    // Nạp theo mảnh phải khớp nạp một lần (bắt tay nạp khóa và GUID hai lần).
    WsSha1 s;
    uint8_t d1[20], d2[20];
    wsSha1Init(&s);
    wsSha1Update(&s, (const uint8_t *)"ab", 2);
    wsSha1Update(&s, (const uint8_t *)"c", 1);
    wsSha1Final(&s, d1);
    wsSha1Init(&s);
    wsSha1Update(&s, (const uint8_t *)"abc", 3);
    wsSha1Final(&s, d2);
    check("chunked sha1 == one-shot sha1", memcmp(d1, d2, 20) == 0);
  }

  printf("\n--- base64 ---\n");
  checkB64("", "");
  checkB64("f", "Zg==");
  checkB64("fo", "Zm8=");
  checkB64("foo", "Zm9v");
  checkB64("foob", "Zm9vYg==");
  checkB64("fooba", "Zm9vYmE=");
  checkB64("foobar", "Zm9vYmFy");

  printf("\n--- handshake accept key (RFC 6455 §4.2.2 example) ---\n");
  {
    char accept[29];
    wsAcceptKey("dGhlIHNhbXBsZSBub25jZQ==", accept);
    bool ok = strcmp(accept, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") == 0;
    printf("%s accept(\"dGhlIHNhbXBsZSBub25jZQ==\") = %s\n", ok ? "ok  " : "FAIL", accept);
    if (!ok) {
      printf("     want s3pPLMBiTxaQ9kYGzzhZRbK+xOo=\n");
      fails++;
    }
  }

  printf("\n--- frame codec ---\n");
  {
    // The browser's MOTOR_TEST stop packet (PROTOCOL.md §5): port 7 ch 0
    // header 0x70, cmd 0x0C, enable 0, four zero thrusts, checksum.
    // Gói MOTOR_TEST dừng của trình duyệt (PROTOCOL.md mục 5): header port 7
    // ch 0 là 0x70, cmd 0x0C, enable 0, bốn lực đẩy không, checksum.
    uint8_t crtp[12] = { 0x70, 0x0C, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    unsigned sum = 0;
    for (int i = 0; i < 11; i++) sum += crtp[i];
    crtp[11] = (uint8_t)sum;
    const uint8_t key[4] = { 0x12, 0x34, 0x56, 0x78 };
    uint8_t frame[32];
    size_t n = clientFrame(frame, 0x2, crtp, 12, key);

    int want = (int)n;
    check("wsFrameLen sees one complete masked frame",
          wsFrameLen(frame, n) == want);
    check("wsFrameLen asks for more on a partial frame",
          wsFrameLen(frame, n - 1) == 0);
    check("wsFrameLen asks for more below the 2-byte header",
          wsFrameLen(frame, 1) == 0);

    uint8_t pay[WS_MAX_MSG];
    size_t plen = 999;
    uint8_t op = wsFrameTake(frame, pay, &plen);
    check("opcode comes back as 2 (binary)", op == 0x2);
    check("FIN is set on an unfragmented frame", (frame[0] & 0x80) != 0);
    check("mask bit is set (browsers always mask)", (frame[1] & 0x80) != 0);
    check("payload is unmasked byte-for-byte",
          plen == 12 && memcmp(pay, crtp, 12) == 0);
    check("checksum byte survives the frame",
          pay[11] == (uint8_t)sum && pay[0] == 0x70 && pay[1] == 0x0C);
  }
  {
    // Server frames (what wsSend() puts on the wire) parse back the same way
    // — the codec accepts unmasked frames too.
    // Khung server (thứ wsSend() đưa lên dây) cũng phân tích lại được — codec
    // chấp nhận cả khung không mặt nạ.
    uint8_t crtp[4] = { 0x70, 0x07, 0x00, 0x7E };   // ARM ack: cmd 07, status 0
    uint8_t frame[16];
    size_t n = wsFramePut(frame, 0x2, crtp, 4);
    check("wsFramePut: FIN + binary + length header",
          frame[0] == 0x82 && frame[1] == 0x04 && n == 6);
    check("wsFrameLen on an unmasked server frame",
          wsFrameLen(frame, n) == (int)n);
    uint8_t pay[WS_MAX_MSG];
    size_t plen = 0;
    uint8_t op = wsFrameTake(frame, pay, &plen);
    check("server frame round-trips", op == 0x2 && plen == 4 &&
          memcmp(pay, crtp, 4) == 0);
  }
  {
    // ping -> pong echo: same payload, opcode 0xA.
    // ping -> pong: cùng payload, opcode 0xA.
    uint8_t frame[16], hi[2] = { 'h', 'i' };
    size_t n = wsFramePut(frame, 0xA, hi, 2);
    check("ping answered as pong 0x8A",
          n == 4 && frame[0] == 0x8A && frame[1] == 0x02 &&
          frame[2] == 'h' && frame[3] == 'i');
  }
  {
    // Jumbo frames are refused: CRTP packets are at most 32 bytes.
    // Khung khổng lồ bị từ chối: gói CRTP dài tối đa 32 byte.
    uint8_t a[2] = { 0x82, 0x7E }, b[2] = { 0x82, 0x7F }, c[2] = { 0x82, 65 };
    check("len=126 form is refused", wsFrameLen(a, 2) == -1);
    check("len=127 form is refused", wsFrameLen(b, 2) == -1);
    check("length above WS_MAX_MSG is refused", wsFrameLen(c, 2) == -1);
    check("empty accumulator wants more", wsFrameLen(c, 0) == 0);
  }

  printf("\n%s\n", fails ? "WS CODEC MISMATCH PRESENT" : "all ws tests passed");
  return fails ? 1 : 0;
}
