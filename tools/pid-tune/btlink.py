#!/usr/bin/env python3
"""Bluetooth links for the ESP32 drone tuner — BLE GATT and BT Classic SPP.

The transport half Drone() leaves pluggable (drone.py sends raw CRTP and gets
raw CRTP back). Two of the three wire formats carry a checksum and one does
not, so the boundary sits exactly there:

  SppLink   [CRTP ...][uint8 checksum] over an RFCOMM stream, byte-for-byte
            the UDP framing; the stream deframer resyncs like the firmware's
            btFrameLen()/btPoll() (shortest checksum-valid length, and a
            CommanderPacket only ever at its real 16-byte wire length).
  BleLink   raw CRTP over GATT — never a checksum (PROTOCOL.md §6). Short
            packets go raw on 00000202; long ones are split into control-byte
            segments on 00000203, and drone→phone notifications on 00000204
            are always control-byte framed and reassembled here.

Pure helpers (crtp_checksum, seg_pack, SegReassembler, frame_len,
StreamDeframer, ble_writes) carry the wire rules and are what selftest.py
exercises; the link classes are thin I/O around them. bleak is imported lazily
inside BleLink so the other tools and the tests need no Bluetooth stack.

Các liên kết Bluetooth cho tuner của drone ESP32 — BLE GATT và BT Classic SPP.
Nửa truyền thông mà Drone() cho phép thay thế (drone.py gửi và nhận CRTP thô).
Hai trong ba định dạng dây có checksum, một cái thì không, nên ranh giới nằm
đúng ở đó:

  SppLink   [CRTP ...][uint8 checksum] trên luồng RFCOMM, giống hệt định dạng
            UDP; bộ tách khung đồng bộ lại như btFrameLen()/btPoll() của
            firmware (độ dài hợp lệ ngắn nhất, và CommanderPacket chỉ ở đúng
            độ dài dây 16 byte).
  BleLink   CRTP thô qua GATT — không bao giờ có checksum (PROTOCOL.md mục 6).
            Gói ngắn gửi thô lên 00000202; gói dài tách thành các đoạn có
            control byte trên 00000203, còn notification drone→điện thoại trên
            00000204 luôn có control byte và được lắp ráp tại đây.

Các hàm thuần (crtp_checksum, seg_pack, SegReassembler, frame_len,
StreamDeframer, ble_writes) chứa quy tắc dây và là phần selftest.py kiểm
tra; các lớp liên kết chỉ là I/O mỏng quanh chúng. bleak được import lười bên
trong BleLink nên các công cụ khác và bài kiểm tra không cần tầng Bluetooth.
"""
import asyncio
import queue
import socket
import sys
import threading

# --- wire constants (PROTOCOL.md §1, §6, §8) --------------------------------

CRTP_MAX_SIZE = 30                # largest CRTP packet either side handles
                                  # gói CRTP lớn nhất hai bên xử lý
BLE_SEG_BYTES = 19                # CRTP bytes per control-byte segment
                                  # byte CRTP mỗi đoạn có control byte
BLE_RAW_MAX = 20                  # 0x202 carries raw CRTP up to this size
                                  # 0x202 mang CRTP thô tới kích thước này
PORT_COMMANDER = 3
CMD_PACKET_SIZE = 16              # 1 header + 14 payload + 1 checksum
BT_FRAME_MAX = CRTP_MAX_SIZE + 1  # a wire frame is raw + its checksum
                                  # một khung dây là raw + checksum

BLE_UUID_SERVICE  = "00000201-1C7F-4F9E-947B-43B7C00A9A08"
BLE_UUID_CRTP     = "00000202-1C7F-4F9E-947B-43B7C00A9A08"   # write, raw
BLE_UUID_CRTPUP   = "00000203-1C7F-4F9E-947B-43B7C00A9A08"   # write, segmented
BLE_UUID_CRTPDOWN = "00000204-1C7F-4F9E-947B-43B7C00A9A08"   # notify, segmented


def crtp_checksum(raw):
    """The low byte of the sum of every CRTP byte (PROTOCOL.md §1). Same
    formula as drone.Drone._checksum; selftest.py pins the two together.
    Byte thấp của tổng mọi byte CRTP (PROTOCOL.md mục 1). Cùng công thức với
    drone.Drone._checksum; selftest.py chốt hai cái lại với nhau."""
    return sum(raw) & 0xFF


# --- BLE control-byte segments (PROTOCOL.md §6) -----------------------------
# Mirrors the firmware's bleSend() on the way out and bleRxSeg() on the way in;
# the state machine is pinned by Esp32_drone/test/bleseg_test.cpp and re-checked
# in Python by selftest.py so the two cannot drift apart.
# Gương theo bleSend() của firmware khi gửi và bleRxSeg() khi nhận; máy trạng
# thái được chốt bởi Esp32_drone/test/bleseg_test.cpp và kiểm lại bằng Python
# trong selftest.py để hai bên không thể lệch nhau.

def seg_pack(crtp, pid):
    """Split one raw CRTP datagram into control-byte segments:
    seg[0] = bit7 start | bits6..5 pid | bits4..0 (totalLen-1), then up to 19
    CRTP bytes. The pid must match across a datagram's segments and is best
    bumped once per datagram (wrapping 0..3).
    Tách một gói CRTP thô thành các đoạn có control byte: ... Pid phải khớp
    giữa các đoạn của một gói và nên tăng mỗi gói (quay vòng 0..3)."""
    crtp = bytes(crtp)
    if not 0 < len(crtp) <= CRTP_MAX_SIZE:
        raise ValueError("CRTP datagram must be 1..%d bytes" % CRTP_MAX_SIZE)
    out = []
    off = 0
    while off < len(crtp):
        n = min(BLE_SEG_BYTES, len(crtp) - off)
        ctrl = ((pid & 0x03) << 5) | ((len(crtp) - 1) & 0x1F)
        if off == 0:
            ctrl |= 0x80
        out.append(bytes([ctrl]) + crtp[off:off + n])
        off += n
    return out


class SegReassembler:
    """Control-byte reassembly, the mirror of the firmware's bleRxSeg().

    The length field is authoritative: dispatch what was announced rather than
    what happened to arrive, so padding cannot reach the parser. A packet is
    single-segment when the whole thing fits in the segment, NOT merely when
    the length field reads < 20 — a 20-byte datagram cannot fit one segment.

    Lắp ráp đoạn có control byte, gương theo bleRxSeg() của firmware.
    Trường độ dài là nguồn quyết định: xử lý đúng số đã báo chứ không phải số
    byte thực tế đến, để phần đệm không lọt vào bộ phân tích. Một gói là một
    đoạn duy nhất khi toàn bộ gói nằm gọn trong đoạn, CHỨ không chỉ khi trường
    độ dài < 20 — gói 20 byte không thể gọn trong một đoạn."""

    def __init__(self):
        self.buf = bytearray()
        self.pid = 0xFF

    def feed(self, ctrl, pay):
        """One segment (control byte + its payload). Returns the assembled raw
        CRTP datagram when a packet just completed, else None.
        Một đoạn (control byte + phần dữ liệu). Trả về gói CRTP thô đã lắp ráp
        xong khi vừa hoàn tất một gói, ngược lại None."""
        pay = bytes(pay)
        start = (ctrl & 0x80) != 0
        pid = (ctrl >> 5) & 0x03
        total = (ctrl & 0x1F) + 1

        if start:
            self.buf = bytearray()
            self.pid = pid
            if len(pay) > CRTP_MAX_SIZE:
                return None
            self.buf = bytearray(pay)
            if total <= len(pay):          # single-segment packet
                self.buf = bytearray()
                return pay[:total]
            return None
        if pid != self.pid or not self.buf:
            self.buf = bytearray()
            return None
        if len(self.buf) + len(pay) > CRTP_MAX_SIZE:
            self.buf = bytearray()
            return None
        self.buf += pay
        if len(self.buf) >= total:
            out = bytes(self.buf[:total])
            self.buf = bytearray()
            return out
        return None


def ble_writes(crtp, pid):
    """The exact GATT writes for one raw CRTP datagram: (characteristic, data)
    pairs. Short packets go raw on 00000202; long ones are segmented on
    00000203. The wire never carries a checksum here (PROTOCOL.md §6) — what
    this returns is what goes out, which is what makes that testable.
    Chính xác các lần ghi GATT cho một gói CRTP thô: các cặp (đặc trưng, dữ
    liệu). Gói ngắn gửi thô lên 00000202; gói dài tách đoạn lên 00000203. Dây
    không bao giờ mang checksum ở đây (PROTOCOL.md mục 6) — đúng những gì hàm
    này trả về là những gì được gửi, đó là điều khiến nó kiểm tra được."""
    crtp = bytes(crtp)
    if not 0 < len(crtp) <= CRTP_MAX_SIZE:
        raise ValueError("CRTP datagram must be 1..%d bytes" % CRTP_MAX_SIZE)
    if len(crtp) <= BLE_RAW_MAX:
        return [(BLE_UUID_CRTP, crtp)]
    return [(BLE_UUID_CRTPUP, seg) for seg in seg_pack(crtp, pid)]


# --- SPP stream framing ------------------------------------------------------
# Mirrors the firmware's btFrameLen(): a coincidental checksum must not split
# a frame, and a partial CommanderPacket must never become another command.
# Gương theo btFrameLen() của firmware: checksum trùng ngẫu nhiên không được
# cắt một khung làm đôi, và CommanderPacket dở dang không bao giờ thành lệnh khác.

def frame_len(buf):
    """How many bytes at the head of `buf` form one SPP frame: > 0 = a whole
    frame of that length (checksum included), 0 = need more bytes, -1 = the
    head byte cannot start a frame, drop it and resync.
    Bao nhiêu byte ở đầu `buf` tạo thành một khung SPP: > 0 = một khung trọn
    vẹn độ dài đó (gồm checksum), 0 = cần thêm byte, -1 = byte đầu không thể
    mở khung, bỏ nó rồi đồng bộ lại."""
    if len(buf) < 2:
        return 0
    if (buf[0] >> 4) & 0x0F == PORT_COMMANDER:
        # Pinned at its real length so a short checksum match inside it cannot
        # turn half a CommanderPacket into a shorter frame.
        # Chốt ở độ dài thật để một checksum khớp ngắn bên trong không biến nửa
        # gói CommanderPacket thành khung ngắn hơn.
        if len(buf) < CMD_PACKET_SIZE:
            return 0
        ok = crtp_checksum(buf[:CMD_PACKET_SIZE - 1]) == buf[CMD_PACKET_SIZE - 1]
        return CMD_PACKET_SIZE if ok else -1
    maxl = min(len(buf), BT_FRAME_MAX)
    for l in range(2, maxl + 1):
        if crtp_checksum(buf[:l - 1]) == buf[l - 1]:
            return l
    return -1 if len(buf) >= BT_FRAME_MAX else 0


class StreamDeframer:
    """Accumulate the SPP byte stream and yield checksum-stripped CRTP frames,
    resyncing exactly like the firmware's btPoll(): a whole frame is handed
    over, a hopeless head byte is dropped one at a time.
    Tích lũy luồng byte SPP và trả về các khung CRTP đã bỏ checksum, đồng bộ
    lại đúng như btPoll() của firmware: khung trọn vẹn được giao đi, byte đầu
    vô vọng bị bỏ từng cái."""

    def __init__(self):
        self.acc = bytearray()

    def feed(self, data):
        """Add bytes; return every raw CRTP frame that completed (no checksum).
        Thêm byte; trả về mọi khung CRTP thô vừa hoàn tất (không có checksum)."""
        self.acc += data
        out = []
        while True:
            l = frame_len(self.acc)
            if l == 0:
                break
            if l > 0:
                out.append(bytes(self.acc[:l - 1]))   # strip the trailing checksum
                                                      # bỏ checksum cuối
            del self.acc[:1 if l < 0 else l]
        return out


# --- the links ---------------------------------------------------------------

class SppLink:
    """CRTP over Bluetooth Classic serial (RFCOMM). The wire is the UDP framing
    — [CRTP ...][uint8 checksum] — so this link appends and strips the checksum
    while StreamDeframer resyncs like the firmware's btPoll().
    CRTP qua serial Bluetooth Classic (RFCOMM). Dây dùng định dạng UDP — [CRTP
    ...][uint8 checksum] — nên liên kết này cộng và bỏ checksum, còn
    StreamDeframer đồng bộ lại như btPoll() của firmware."""

    def __init__(self, address, channel=1, sock=None):
        self.address = address
        self.channel = channel
        self._sock = sock          # injectable for the tests (socketpair)
                                   # cắm vào được cho các bài kiểm (socketpair)
        self._rx = StreamDeframer()
        self._out = []
        self._lock = threading.Lock()

    def connect(self):
        if self._sock is not None:
            self._sock.settimeout(0.05)
            return
        if not hasattr(socket, "AF_BLUETOOTH"):
            raise RuntimeError("this Python has no Bluetooth RFCOMM support")
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                          socket.BTPROTO_RFCOMM)
        s.settimeout(10.0)
        try:
            s.connect((self.address, self.channel))
        except OSError as e:
            s.close()
            raise RuntimeError("SPP connect to %s failed: %s" % (self.address, e))
        s.settimeout(0.05)
        self._sock = s

    def describe(self):
        return "spp %s" % self.address

    def send(self, raw):
        raw = bytes(raw)
        with self._lock:                       # whole frames only, never interleave
                                               # chỉ khung trọn vẹn, không xen kẽ
            self._sock.sendall(raw + bytes([crtp_checksum(raw)]))

    def recv(self):
        """One raw CRTP frame, None when nothing arrived yet, OSError when the
        link is dead. Thread-safe against send().
        Một khung CRTP thô, None khi chưa có gì, OSError khi liên kết chết.
        An toàn luồng khi gọi song song với send()."""
        if self._out:
            return self._out.pop(0)
        try:
            data = self._sock.recv(512)
        except socket.timeout:
            return None
        if not data:
            raise OSError("SPP link closed")
        self._out = self._rx.feed(data)
        return self._out.pop(0) if self._out else None

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass


class BleLink:
    """CRTP over BLE GATT (PROTOCOL.md §6) via bleak. Raw CRTP on the wire — no
    checksum ever. bleak is asyncio-based, so the link runs its own event loop
    in a daemon thread and stays synchronous (send/recv) for Drone().
    `target` is the advertised name ("ESP32_Done") or a MAC address.
    CRTP qua BLE GATT (PROTOCOL.md mục 6) bằng bleak. CRTP thô trên dây —
    không bao giờ có checksum. bleak dựa trên asyncio nên liên kết chạy vòng
    lặp sự kiện riêng trong một luồng daemon và vẫn đồng bộ (send/recv) với
    Drone(). `target` là tên quảng bá ("ESP32_Done") hoặc địa chỉ MAC."""

    def __init__(self, target="ESP32_Done", timeout=10.0):
        self.timeout = timeout
        # A target is an address when it looks like a MAC (Linux/BlueZ) or a
        # CoreBluetooth UUID (macOS); anything else is an advertised name. Names
        # may contain '-' ("ESP-Drone"), so a UUID is pinned by shape (8-4-4-4-12),
        # not by a stray hyphen. The device picker feeds addresses so the
        # connect is direct — no name scan to rediscover the same drone.
        # Mục tiêu là địa chỉ khi trông như MAC (Linux/BlueZ) hoặc UUID
        # CoreBluetooth (macOS); còn lại là tên quảng bá. Tên có thể chứa '-'
        # ("ESP-Drone"), nên UUID được chốt theo hình dạng (8-4-4-4-12), không
        # phải một dấu gạch ngẫu nhiên. Trình chọn thiết bị đưa địa chỉ để kết
        # nối thẳng — không cần quét tên để tìm lại đúng drone đó.
        if target.count(":") == 5 or (len(target) == 36 and target.count("-") == 4):
            self.address, self.name = target, None
        else:
            self.address, self.name = None, target
        self._loop = None
        self._thread = None
        self._client = None
        self._pid = 0
        self._rx = SegReassembler()
        self._q = queue.Queue()
        self._send_lock = threading.Lock()
        self._dead = None

    # --- lifecycle ---

    def connect(self):
        self._dead = None
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()
        fut = asyncio.run_coroutine_threadsafe(self._open(), self._loop)
        try:
            # +5 s slack: a name target scans for up to timeout s before it can
            # even start connecting.
            # Dư +5 s: mục tiêu kiểu tên phải quét tới timeout s rồi mới kết nối.
            fut.result(timeout=self.timeout + 5.0)
        except Exception:
            self.close()          # a failed open must not leak the loop thread
                                  # mở thất bại không được rò luồng vòng lặp
            raise

    async def _open(self):
        try:
            from bleak import BleakClient, BleakScanner  # lazy: tests need no bleak
                                                         # lười: bài kiểm không cần bleak
        except ImportError as e:
            # Without bleak BLE cannot run at all — say how to get it instead
            # of dropping a bare ImportError on the user.
            # Thiếu bleak thì BLE không chạy được — báo cách cài thay vì ném
            # ImportError trống cho người dùng.
            raise RuntimeError("BLE needs the bleak package (pip install bleak, "
                               "or run tools/pid-tune/.venv/bin/python)") from e
        try:
            target = self.address
            if target is None:
                dev = await BleakScanner.find_device_by_filter(
                    lambda d, ad: d.name == self.name, timeout=self.timeout)
                if dev is None:
                    raise RuntimeError("no BLE device named %r found" % self.name)
                target = dev
            self._client = BleakClient(target, disconnected_callback=self._on_gone)
            await self._client.connect()
            await self._client.start_notify(BLE_UUID_CRTPDOWN, self._on_notify)
        except RuntimeError:
            raise
        except Exception as e:
            # bleak raises its own error type, which none of the callers catch;
            # RuntimeError is what they already handle (SppLink's connect raises
            # it too).
            # bleak ném kiểu lỗi riêng của nó, không nơi gọi nào bắt được;
            # RuntimeError là thứ họ đã xử lý (connect của SppLink cũng ném vậy).
            raise RuntimeError("BLE connect to %s failed: %s"
                               % (self.address or self.name, e)) from e

    def _on_gone(self, _client):
        self._dead = OSError("BLE link lost")

    def _on_notify(self, _handle, data):
        if not data:
            return
        done = self._rx.feed(data[0], data[1:])
        if done is not None:
            self._q.put(done)

    # --- transport ---

    def describe(self):
        return "ble %s" % (self.address or self.name)

    def send(self, raw):
        # Fail fast like recv() when the link is gone. bleak's own error type
        # is a bare Exception, so left uncaught it sails past every
        # except (TimeoutError, OSError, RuntimeError, ...) in the GUI as an
        # "Exception in Tkinter callback" traceback — the dead-link state is
        # raised as OSError, which all of them already handle.
        # Thất bại nhanh như recv() khi liên kết đã mất. Kiểu lỗi riêng của
        # bleak là Exception trần, nên nếu không bắt nó sẽ lọt qua mọi
        # except (TimeoutError, OSError, RuntimeError, ...) trong GUI thành
        # traceback "Exception in Tkinter callback" — trạng thái chết được ném
        # như OSError, thứ tất cả các nơi đó đã xử lý.
        if self._dead is not None:
            raise self._dead
        if self._loop is None:
            raise OSError("BLE link is not connected")
        with self._send_lock:
            pid = self._pid
            self._pid = (self._pid + 1) & 0x03
        writes = ble_writes(raw, pid)          # raw CRTP: never a checksum
                                               # CRTP thô: không bao giờ checksum
        fut = asyncio.run_coroutine_threadsafe(self._write_all(writes), self._loop)
        fut.result(timeout=self.timeout)

    async def _write_all(self, writes):
        for uuid, data in writes:
            # ATT write request (with response). The firmware's write chars are
            # PROPERTY_WRITE only, and macOS silently drops write commands to
            # chars that do not advertise write-without-response — the bytes
            # would never reach the drone (verified against bleak's CoreBluetooth
            # backend: response=False maps to CBCharacteristicWriteWithoutResponse).
            # ATT write request (có response). Các char ghi của firmware chỉ có
            # PROPERTY_WRITE, mà macOS âm thầm bỏ write command tới char không
            # quảng bá write-without-response — byte sẽ không bao giờ tới drone
            # (đã kiểm với backend CoreBluetooth của bleak: response=False映
            # sang CBCharacteristicWriteWithoutResponse).
            try:
                await self._client.write_gatt_char(uuid, data, response=True)
            except Exception as e:
                # After a drop the CoreBluetooth backend clears its services,
                # so every later write raises BleakError("Service Discovery has
                # not been performed yet"). Translate it at the boundary and
                # remember the link is dead so later send()/recv() fail fast.
                # Sau khi rơi, backend CoreBluetooth xóa services nên mọi lần
                # ghi sau đều ném BleakError("Service Discovery has not been
                # performed yet"). Dịch nó ở ranh giới và nhớ liên kết đã chết
                # để send()/recv() sau thất bại nhanh.
                self._dead = OSError("BLE link lost (%s)" % e)
                raise self._dead from e

    def recv(self):
        if self._dead is not None:
            raise self._dead
        try:
            return self._q.get(timeout=0.05)
        except queue.Empty:
            return None

    def close(self):
        # A closed link is a dead link: send() after close() must raise a
        # clean OSError, not crash on the None loop.
        # Liên kết đã đóng cũng là liên kết đã chết: send() sau close() phải
        # ném OSError sạch, không được sập vì vòng lặp là None.
        if self._dead is None:
            self._dead = OSError("BLE link closed")
        if self._loop is None:
            return
        if self._client is not None:
            try:
                fut = asyncio.run_coroutine_threadsafe(self._client.disconnect(),
                                                       self._loop)
                fut.result(timeout=2.0)
            except Exception:                  # already gone / loop stopping
                                               # đã mất / vòng lặp đang dừng
                pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop = None


# --- device discovery ---------------------------------------------------------
# The device picker (tuner_gui's Devices button) lists what the radio can see so
# the user chooses the drone instead of typing a name or address. Both return
# rows of (name, address, rssi|None) with drone-like names first.
# Trình chọn thiết bị (nút Devices của tuner_gui) liệt kê những gì radio thấy
# được để người dùng chọn drone thay vì gõ tên hay địa chỉ. Cả hai trả về các
# hàng (name, address, rssi|None) với tên giống drone lên trước.

DRONE_NAMES = ("ESP32_Done", "ESP-Drone", "Crazyflie")
               # the names the drone firmware advertises under
               # các tên mà firmware drone dùng để quảng bá


def _picker_rows(rows):
    """Drone-like names first, then strongest signal; `rows` are the raw
    (name, address, rssi) tuples.
    Tên giống drone lên trước, sau đó là tín hiệu mạnh nhất; `rows` là các bộ
    ba thô (name, address, rssi)."""
    return sorted(rows, key=lambda r: (r[0] not in DRONE_NAMES,
                                       -(r[2] if r[2] is not None else -127)))


def scan_ble(timeout=5.0):
    """Every BLE advertiser the radio can see: (name, address, rssi). Needs
    bleak and a working Bluetooth radio; runs its own event loop, so it is safe
    to call from a worker thread (which is how tuner_gui uses it).
    Mọi thiết bị BLE mà radio thấy được: (name, address, rssi). Cần bleak và
    radio Bluetooth hoạt động; tự chạy vòng lặp sự kiện riêng nên gọi được từ
    luồng phụ (đúng cách tuner_gui dùng nó)."""
    from bleak import BleakScanner  # lazy: tests need no bleak
                                     # lười: bài kiểm không cần bleak
    found = asyncio.run(BleakScanner.discover(timeout=timeout, return_adv=True))
    rows = []
    for addr, (dev, adv) in found.items():
        name = dev.name or adv.local_name or ""
        rows.append((name or "(no name)", addr, adv.rssi))
    return _picker_rows(rows)


def scan_spp():
    """Paired Bluetooth Classic devices — SPP has no service scan, so the drone
    must already be paired: (name, address, None). Best effort per platform:
    macOS lists its paired devices via system_profiler, Linux asks bluetoothctl;
    anything else returns [].
    Thiết bị Bluetooth Classic đã ghép đôi — SPP không có quét dịch vụ nên drone
    phải đã ghép đôi từ trước: (name, address, None). Tốt nhất theo nền tảng:
    macOS liệt kê thiết bị đã ghép đôi qua system_profiler, Linux hỏi
    bluetoothctl; nền tảng khác trả về []."""
    import json
    import subprocess
    rows = []
    try:
        if sys.platform == "darwin":
            out = subprocess.run(["system_profiler", "SPBluetoothDataType", "-json"],
                                 capture_output=True, text=True, timeout=15).stdout
            for bus in json.loads(out).get("SPBluetoothDataType", []):
                for key in ("device_connected", "device_not_connected"):
                    for entry in bus.get(key, []):
                        # macOS lists each device as {name: {device_address: …}}
                        # (older versions use flat device_name/device_address).
                        # macOS liệt kê mỗi thiết bị dạng {name: {device_address: …}}
                        # (bản cũ dùng device_name/device_address phẳng).
                        if "device_address" in entry:
                            items = [(entry.get("device_name") or "(no name)", entry)]
                        else:
                            items = list(entry.items())
                        for name, info in items:
                            addr = info.get("device_address") or ""
                            if addr:
                                rssi = info.get("device_rssi")
                                rows.append((name or "(no name)", addr,
                                             int(rssi) if rssi not in (None, "") else None))
        else:
            out = subprocess.run(["bluetoothctl", "devices", "Paired"],
                                 capture_output=True, text=True, timeout=10).stdout
            for line in out.splitlines():
                parts = line.split(maxsplit=2)     # "Device AA:BB:.. Name"
                if len(parts) == 3 and parts[0] == "Device":
                    rows.append((parts[2], parts[1], None))
    except (OSError, subprocess.SubprocessError, ValueError):
        pass                                       # no tool / no radio / bad JSON
                                                   # thiếu công cụ / thiếu radio / JSON hỏng
    return _picker_rows(rows)
