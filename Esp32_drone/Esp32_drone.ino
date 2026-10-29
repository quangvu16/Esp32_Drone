/* =============================================================================
 *  XW_Z1_V2.0  欣薇_无人机 — ESP32 Flight Controller Firmware
 * =============================================================================
 *  Board : ESP32 DevKit + MPU6500 (SPI) + SBUS RX + 4x AO3416 brushed drivers
 *  Bo mạch : ESP32 DevKit + MPU6500 (SPI) + SBUS RX + 4x driver chổi than AO3416
 *  Motors: 8520 coreless, direct MOSFET PWM (NOT ESC / servo pulse)
 *  Động cơ: 8520 coreless, PWM MOSFET trực tiếp (KHÔNG phải ESC / xung servo)
 *
 *  VERIFIED PIN MAP (from schematic Screenshot 2026-10-26):
 *  ┌──────────────┬────────────┬────────────────────────────────────────┐
 *  │ Function     │ ESP32 GPIO │ Notes                                  │
 *  ├──────────────┼────────────┼────────────────────────────────────────┤
 *  │ PWM1  RF/Q1  │ 15         │ front-right  motor (AO3416 low-side)   │
 *  │ PWM2  RR/Q2  │ 13         │ rear-right   motor                     │
 *  │ PWM3  LR/Q3  │ 12         │ rear-left    motor                     │
 *  │ PWM4  LF/Q4  │ 14         │ front-left   motor                     │
 *  │ SBUS RX      │  4         │ UART2, 100000 8E2, inverted            │
 *  │ MPU SCK      │ 18         │ VSPI SCK                               │
 *  │ MPU MISO     │ 19         │ VSPI MISO (MPU pin 7 ADO)              │
 *  │ MPU MOSI     │ 23         │ VSPI MOSI (MPU pin 4 SDA)              │
 *  │ MPU NCS      │  5         │ VSPI CS                                │
 *  │ VBAT sense   │ 36 (VP)    │ R11 10K / R10 33K → VBAT = Vad*43/33   │
 *  └──────────────┴────────────┴────────────────────────────────────────┘
 *
 *  Quad layout & motor numbering (top view, matches schematic labels):
 *  Bố trí quad và đánh số động cơ (nhìn từ trên xuống, khớp nhãn trên sơ đồ):
 *
 *         FRONT
 *     LF(4)   RF(1)
 *        \   /
 *         \ /
 *         / \
 *        /   \
 *     LR(3)   RR(2)
 *         REAR
 *
 *  Prop direction (viewed from above) — MATCH YOUR PROPS TO THIS:
 *  Hướng quay cánh quạt (nhìn từ trên xuống) — LẮP CÁNH THEO ĐÚNG NHƯ SAU:
 *     RF & LR = CW      RR & LF = CCW
 *  (If yaw is inverted in flight, flip YAW_DIRECTION below.)
 *  (Nếu yaw bị ngược khi bay, đổi dấu YAW_DIRECTION bên dưới.)
 *
 *  SBUS channel map (default, adjust to your radio):
 *  Ánh xạ kênh SBUS (mặc định, chỉnh theo tay điều khiển của bạn):
 *     CH1 = roll, CH2 = pitch, CH3 = throttle, CH4 = yaw, CH5 = arm switch
 *
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │ BUILD: select the "Huge APP" partition scheme.                       │
 *  │ BUILD: chọn scheme phân vùng "Huge APP".                            │
 *  │   Arduino IDE  -> Tools -> Partition Scheme -> Huge APP              │
 *  │   arduino-cli  -> --fqbn esp32:esp32:esp32:PartitionScheme=huge_app  │
 *  │                                                                      │
 *  │ WiFi + Bluetooth Classic + BLE all compile in, which is ~1.7 MB and  │
 *  │ does not fit the default 1.25 MB app partition. Huge APP gives 3 MB  │
 *  │ and drops OTA, which a drone does not need.                          │
 *  │ WiFi + Bluetooth Classic + BLE đều được biên dịch vào, khoảng 1.7 MB │
 *  │ và không vừa phân vùng app mặc định 1.25 MB. Huge APP cho 3 MB và   │
 *  │ bỏ OTA — thứ mà drone không cần.                                    │
 *  └──────────────────────────────────────────────────────────────────────┘
 *
 *  Protocol reference for the phone apps: ../PROTOCOL.md
 *  Tài liệu giao thức cho các app điện thoại: ../PROTOCOL.md
 * ==========================================================================*/

#include <SPI.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <Preferences.h>

// Optional transports. Turning one off saves flash; the link profile that needs
// it is then reported as unavailable at boot instead of being offered.
// Các tầng truyền tùy chọn. Tắt một cái sẽ tiết kiệm flash; hồ sơ liên kết cần
// nó sẽ được báo là không khả dụng khi khởi động thay vì được đưa ra lựa chọn.
#define LINK_USE_BT_SPP  1
#define LINK_USE_BLE     1

#if LINK_USE_BT_SPP
#include <BluetoothSerial.h>
#endif
#if LINK_USE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif

// ============================ PIN DEFINITIONS ==============================
#define PIN_MOSI       23
#define PIN_MISO       19
#define PIN_SCK        18
#define PIN_MPU_CS      5

#define PIN_SBUS_RX     4          // UART2 RX (signal only; VBAT/GND on H3)

#define PIN_MOTOR_RF   15          // PWM1 -> Q1  front-right
#define PIN_MOTOR_RR   13          // PWM2 -> Q2  rear-right
#define PIN_MOTOR_LR   12          // PWM3 -> Q3  rear-left
#define PIN_MOTOR_LF   14          // PWM4 -> Q4  front-left

#define PIN_VBAT       36          // VP = ADC1_CH0

// ============================ TIMING / LOOP ================================
#define LOOP_TIME_US   4000        // 250 Hz
#define GYRO_SCALE     0.0000611f  // 1/65.5 LSB/(deg/s) * 0.004 s  (±500 dps)
#define RAD_TO_DEG     57.2958f

// ============================ RC / SBUS ====================================
#define SBUS_FRAME_LEN       25
#define SBUS_MIN           172     // FrSky/standard SBUS low
#define SBUS_MAX          1811
#define SBUS_CENTER        992
#define RC_MIN            1000     // mapped µs range
#define RC_MAX            2000
#define RC_CENTER         1500
#define THROTTLE_ARM_MAX  1050     // stick must be below this to arm
#define THROTTLE_HOVER    1200     // informational only
#define YAW_ARM_THRESHOLD 1700     // hold right to arm
#define YAW_DISARM_THR    1300     // hold left to disarm
#define ARM_HOLD_MS       1000
#define FAILSAFE_MS        500

// channel index (0-based) — change to match your radio trainer map
// chỉ mục kênh (bắt đầu từ 0) — đổi theo bản đồ trainer của tay điều khiển
#define CH_ROLL           0
#define CH_PITCH          1
#define CH_THROTTLE       2
#define CH_YAW            3
#define CH_ARM            4        // optional switch; if unused, stick-arm works

// ============================ BATTERY ======================================
#define VBAT_DIVIDER      (43.0f / 33.0f)   // R11=10K top, R10=33K bottom
#define VBAT_CELLS        1                 // 1S LiPo typical for 8520 whoop
#define VBAT_CELL_WARN    3.50f
#define VBAT_CELL_CRIT    3.30f
// Low-battery protection. The ADC reading is smoothed and only a SUSTAINED drop
// counts, so a throttle punch that sags the cell for a moment does not trigger
// it. Set VBAT_ARM_CHECK to 0 to allow arming on USB power with no battery.
// Bảo vệ pin yếu. Giá trị đọc ADC được làm mượt và chỉ mức giảm DUY TRÌ mới
// được tính, nên một cú đẩy throttle làm sụt pin trong chốc lát sẽ không kích hoạt.
// Đặt VBAT_ARM_CHECK = 0 để cho phép kích hoạt khi cấp nguồn USB, không cần pin.
#define VBAT_ARM_CHECK    1
#define VBAT_FILTER_ALPHA 0.02f    // per 4 ms loop, ~0.2 s time constant
#define VBAT_CRIT_HOLD_MS 3000

// ============================ PID GAINS (tune in flight!) ==================
//  Untested placeholders — tune these with the phone's tuning screen or the
//  tuning screen's auto-tune before trusting a hover. Defaults live in macros
//  so cfgLoad()'s "restore defaults" and these declarations cannot drift apart.
//  Giá trị placeholder chưa được thử — hãy tinh chỉnh bằng màn hình tinh chỉnh
//  của điện thoại hoặc auto-tune của màn hình đó trước khi tin vào hover.
//  Giá trị mặc định nằm trong macro để "restore defaults" của cfgLoad() và các
//  khai báo này không thể lệch nhau.
#define PID_ROLL_KP   1.3f
#define PID_ROLL_KI   0.04f
#define PID_ROLL_KD   18.0f
#define PID_PITCH_KP  1.3f
#define PID_PITCH_KI  0.04f
#define PID_PITCH_KD  18.0f
#define PID_YAW_KP    4.0f
#define PID_YAW_KI    0.02f
#define PID_YAW_KD    0.0f

float pid_p_gain_roll  = PID_ROLL_KP;   float pid_i_gain_roll  = PID_ROLL_KI;  float pid_d_gain_roll  = PID_ROLL_KD;
float pid_p_gain_pitch = PID_PITCH_KP;  float pid_i_gain_pitch = PID_PITCH_KI; float pid_d_gain_pitch = PID_PITCH_KD;
float pid_p_gain_yaw   = PID_YAW_KP;    float pid_i_gain_yaw   = PID_YAW_KI;   float pid_d_gain_yaw   = PID_YAW_KD;

#define PID_LIMIT        400.0f   // ± output clamp
#define I_TERM_LIMIT     400.0f
#define LEVEL_SENSITIVITY 15.0f   // angle(deg) -> rate-setpoint scaling
#define SETPOINT_DIVISOR   3.0f   // stick deflection -> rate setpoint
#define THROTTLE_MAX      1800    // leave headroom for PID at full stick

// CRTP commander safety clamps
// Các mức kẹp an toàn cho CRTP commander
#define MAX_TILT_DEG       25.0f  // max commanded roll/pitch angle (deg)
#define MAX_YAW_RATE      200.0f  // max commanded yaw rate (deg/s)

// Axis / sign config — flip these if a rotation is inverted after first hover
// Cấu hình trục / dấu — đảo các giá trị này nếu một phép quay bị ngược sau lần giữ độ cao đầu tiên
#define GYRO_X_SIGN       1       // pitch rate
#define GYRO_Y_SIGN       1       // roll  rate
#define GYRO_Z_SIGN       1       // yaw   rate
#define ACC_X_SIGN        1
#define ACC_Y_SIGN        1
#define YAW_DIRECTION     1       // +1 = RF/LR are CW props (default above)

// ============================ WIFI / CRTP / UDP ============================
//  Matches the Espressif ESP-Drone apps (Android EspUdpDriver / iOS ESPUDPLink).
//  Every UDP datagram is:  [CRTP bytes ...][uint8 checksum = 8-bit sum of CRTP bytes]
//  Phone -> drone : udp://192.168.43.42:2390
//  Drone  -> phone: udp://<phone ip>:2399
//  Khớp với các app ESP-Drone của Espressif (Android EspUdpDriver / iOS ESPUDPLink).
//  Mỗi gói UDP là:  [CRTP bytes ...][uint8 checksum = tổng 8-bit của các byte CRTP]
//  Điện thoại -> drone : udp://192.168.43.42:2390
//  Drone -> điện thoại: udp://<ip điện thoại>:2399
#define WIFI_AP_SSID      "ESP-Drone"
#define WIFI_AP_PASS      "12345678"      // WPA2, >=8 chars
#define WIFI_AP_CHANNEL   6
#define WIFI_AP_IP        IPAddress(192, 168, 43, 42)
#define WIFI_AP_GW        IPAddress(192, 168, 43, 42)
#define WIFI_AP_MASK      IPAddress(255, 255, 255, 0)
#define UDP_DEVICE_PORT   2390            // drone listens here
#define UDP_APP_PORT      2399            // drone sends replies here
#define CRTP_MAX_SIZE     30              // largest CRTP packet we handle

// CRTP header:  bits 7..4 = port, bits 3..2 reserved, bits 1..0 = channel
// CRTP header:  bit 7..4 = port, bit 3..2 dành riêng, bit 1..0 = channel
#define CRTP_PORT_CONSOLE     0
#define CRTP_PORT_PARAMS      2
#define CRTP_PORT_COMMANDER   3
#define CRTP_PORT_MEMORY      4
#define CRTP_PORT_LOGGING     5
#define CRTP_PORT_GENERIC     7
#define CRTP_PORT_LINK        15

// CommanderPacket (CRTP port 3, channel 0), little-endian:
// CommanderPacket (CRTP port 3, channel 0), định dạng little-endian:
//   [0] header 0x30
//   [1..4]  float roll  (deg)
//   [5..8]  float pitch (deg, NOTE: app sends -pitch)
//   [5..8]  float pitch (độ, LƯU Ý: app gửi -pitch)
//   [9..12] float yaw   (deg/s)
//   [13..14] uint16 thrust (0..65535)
#define CMD_PACKET_SIZE   16              // 1 header + 14 payload + 1 checksum

// ==================== CRTP TOC / PARAM / LOG CONSTANTS =====================
// Command and type codes plus the TocEntry/LogBlock structs. These sit up here
// with the other #defines because the Arduino preprocessor auto-generates a
// prototype for every function in the sketch and injects them before the first
// function definition — so any type named in a function signature must already
// be complete by that point. The protocol itself is documented in the
// "CRTP TOC / PARAMETERS / LOGGING" section further down.
// Mã lệnh và mã kiểu cùng các struct TocEntry/LogBlock. Chúng nằm ở đây cùng
// các #define khác vì bộ tiền xử lý Arduino tự tạo prototype cho mọi hàm trong
// sketch và chèn chúng trước định nghĩa hàm đầu tiên — nên mọi kiểu được đặt tên
// trong chữ ký hàm phải đã hoàn chỉnh ở thời điểm đó. Bản thân giao thức được
// mô tả trong phần "CRTP TOC / PARAMETERS / LOGGING" ở phía dưới.
// ===========================================================================

// --- TOC request commands (identical on the params and logging ports) ---
// --- lệnh yêu cầu TOC (giống nhau trên cả port tham số và port log) ---
#define TOC_CMD_GET_ELEMENT  0
#define TOC_CMD_GET_INFO     1

// --- CRTP port 2 (params) channels ---
#define PARAM_CHAN_TOC       0
#define PARAM_CHAN_READ      1
#define PARAM_CHAN_WRITE     2

// --- CRTP port 5 (logging) channels ---
#define LOG_CHAN_TOC         0
#define LOG_CHAN_SETTINGS    1
#define LOG_CHAN_LOGDATA     2

// --- logging control commands (log/Logg.java) ---
// --- lệnh điều khiển log (log/Logg.java) ---
#define LOG_CMD_CREATE       0
#define LOG_CMD_APPEND       1
#define LOG_CMD_DELETE       2
#define LOG_CMD_START        3
#define LOG_CMD_STOP         4
#define LOG_CMD_RESET        5

// --- error codes Logg.java turns into user-visible strings ---
// --- mã lỗi mà Logg.java chuyển thành chuỗi hiển thị cho người dùng ---
#define LOG_ERR_OK           0
#define LOG_ERR_NO_ID        2
#define LOG_ERR_TOO_LARGE    7
#define LOG_ERR_NO_CMD       8
#define LOG_ERR_NO_MEMORY   12
#define LOG_ERR_EXISTS      17

// Parameter type codes — low nibble of the TOC type byte (toc/Toc.java).
// Mã kiểu tham số — nibble thấp của byte kiểu TOC (toc/Toc.java).
#define PT_INT8    0x00
#define PT_INT16   0x01
#define PT_FLOAT   0x06
#define PT_UINT8   0x08
#define PT_UINT16  0x09
#define PT_UINT32  0x0A

// Log type codes — a DIFFERENT numbering from the parameter codes above.
// Mã kiểu log — cách đánh số KHÁC với mã tham số ở trên.
#define LT_UINT8   0x01
#define LT_UINT16  0x02
#define LT_FLOAT   0x07

// TocElement.java reads read-only from bit 6 of the type byte.
// TocElement.java đọc cờ chỉ đọc từ bit 6 của byte kiểu.
#define TOC_ACCESS_RO  0x40
#define TOC_ACCESS_RW  0x00

// Log data budget: <header><id><ts24><values...> must fit in CRTP_MAX_SIZE.
// Ngân sách dữ liệu log: <header><id><ts24><values...> phải vừa trong CRTP_MAX_SIZE.
#define LOG_DATA_OVERHEAD  5                 // header + id + 24-bit timestamp
#define LOG_MAX_PAYLOAD    (CRTP_MAX_SIZE - LOG_DATA_OVERHEAD)
#define LOG_MAX_BLOCKS     3
#define LOG_MAX_VARS       12

// ==================== TRANSPORT / LINK PROFILES ============================
//  The same CRTP framing ([CRTP ...][uint8 checksum]) runs over four
//  transports. Which one is live is chosen at boot from NVS (key "link"), so a
//  radio change always starts from a known state.
//  Cùng một khung CRTP ([CRTP ...][uint8 checksum]) chạy trên bốn tầng truyền.
//  Tầng nào đang hoạt động được chọn khi khởi động từ NVS (key "link"), nên đổi
//  radio luôn bắt đầu từ một trạng thái đã biết.
//
//    0  WIFI_AP     drone hosts "ESP-Drone", phone joins it (default)
//    0  WIFI_AP     drone phát "ESP-Drone", điện thoại kết nối vào (mặc định)
//    1  WIFI_STA    drone joins an existing network, mDNS esp-drone.local
//    1  WIFI_STA    drone tham gia mạng có sẵn, mDNS esp-drone.local
//    2  BT_SPP      Bluetooth Classic serial — no WiFi involved
//    2  BT_SPP      Bluetooth Classic serial — không dùng WiFi
//    3  BLE         BLE GATT, the Bitcraze Crazyflie service (works on iOS)
//    3  BLE         BLE GATT, service Bitcraze Crazyflie (chạy trên iOS)
//    4  WIFI_AP_BLE WiFi softAP plus BLE, sharing the radio
//    4  WIFI_AP_BLE WiFi softAP kèm BLE, dùng chung radio
//    5  WIFI_STA_BLE WiFi station plus BLE
//    5  WIFI_STA_BLE WiFi station kèm BLE
//
//  ESP32 WiFi and Bluetooth Classic share one 2.4 GHz radio, so BT_SPP is an
//  alternative to WiFi rather than something that runs alongside it. BLE has a
//  coexistence mode and can share with WiFi, which is what the *_BLE profiles
//  use. Pick one with "link <n>" on the serial console or FC_CMD_LINK over
//  CRTP; the board reboots to apply it.
//  WiFi và Bluetooth Classic trên ESP32 dùng chung một radio 2.4 GHz, nên BT_SPP
//  là phương án thay thế WiFi chứ không phải chạy song song. BLE có chế độ chung
//  sống và có thể dùng chung với WiFi — đó là các hồ sơ *_BLE. Chọn bằng "link <n>"
//  trên console serial hoặc FC_CMD_LINK qua CRTP; board sẽ khởi động lại để áp dụng.
#define LINK_MODE_WIFI_AP      0
#define LINK_MODE_WIFI_STA     1
#define LINK_MODE_BT_SPP       2
#define LINK_MODE_BLE          3
#define LINK_MODE_WIFI_AP_BLE  4
#define LINK_MODE_WIFI_STA_BLE 5
#define LINK_MODE_COUNT        6

// Station-mode credentials, used by LINK_MODE_WIFI_STA*. Empty SSID falls back
// to the softAP profile so a board with no config still comes up reachable.
// Thông tin đăng nhập chế độ station, dùng bởi LINK_MODE_WIFI_STA*. SSID rỗng sẽ
// chuyển về hồ sơ softAP để board chưa cấu hình vẫn lên sóng và kết nối được.
#define WIFI_STA_SSID          ""            // <-- set yours, or via serial "sta <ssid> <pass>"
#define WIFI_STA_PASS          ""
#define WIFI_STA_TIMEOUT_MS    15000
#define MDNS_HOSTNAME          "esp-drone"

// ---- BLE GATT framing ------------------------------------------------
//  Matches the Bitcraze service the Android BleLink talks to, so that client
//  works unmodified:
//  Khớp với service Bitcraze mà BleLink trên Android kết nối tới, nên client đó
//  chạy được nguyên bản, không cần sửa:
//    00000201-1C7F-4F9E-947B-43B7C00A9A08  service
//      00000202  CRTP      write   phone -> drone, raw CRTP when <= 20 bytes
//      00000202  CRTP      write   điện thoại -> drone, CRTP thô khi <= 20 byte
//      00000203  CRTPUP    write   phone -> drone, segmented with control byte
//      00000203  CRTPUP    write   điện thoại -> drone, phân đoạn theo control byte
//      00000204  CRTPDOWN  notify  drone -> phone, always control-byte framed
//      00000204  CRTPDOWN  notify  drone -> điện thoại, luôn đóng khung bằng control byte
//
//  Control byte:  bit7 start | bits6..5 pid (0..3) | bits4..0 (totalLen-1)
//  A datagram of N CRTP bytes goes out in ceil(N/19) segments of 20 bytes:
//     [ctrl start, len=N-1]  crtp[0 .. 18]
//     [ctrl cont , pid    ]  crtp[19 .. 37]   ... and so on
//  The receiver keys single- vs multi-segment on whether the announced length
//  fits in the segment, not on the length field alone — see BLE_SEG_BYTES.
//  Một gói N byte CRTP được gửi ra thành ceil(N/19) đoạn 20 byte:
//     [ctrl start, len=N-1]  crtp[0 .. 18]
//     [ctrl cont , pid    ]  crtp[19 .. 37]   ... và tiếp tục như vậy
//  Bên nhận phân biệt một đoạn hay nhiều đoạn bằng việc độ dài được báo có vừa
//  trong đoạn hay không, chứ không chỉ dựa vào trường độ dài — xem BLE_SEG_BYTES.
#define BLE_SEG_BYTES        19             // CRTP bytes per 20-byte BLE segment
#define BLE_MAX_SEGMENTS     2              // CRTP_MAX_SIZE 30 -> 2 segments

// ---- CRTP port 7, channel 0: flight control (this firmware's own extension)
// ---- CRTP port 7, channel 0: điều khiển bay (phần mở rộng riêng của firmware này)
//  [0] 0x70 | [1] cmd | [2..] args        reply: [0x70][cmd][status u8]
//
//  0x01 SET_MODE      u8 mode     0 manual | 1 hold | 2 auto
//  0x02 TAKEOFF       u16 thrust  target hover thrust 0..65535 -> AUTO
//  0x03 LAND          —           ramp to idle then disarm       -> AUTO
//  0x04 SET_HOVER     u16 thrust  change the hover thrust
//  0x05 EMERGENCY     —           motors off + disarm, always honoured
//  0x06 SET_HEADING   f32 deg     heading locked in hold/auto
//  0x07 ARM           —
//  0x08 DISARM        —
//  0x09 SAVE          —           persist gains + flight config to NVS
//  0x0A DEFAULTS      —           restore default gains (not saved)
//  0x0B LOAD          —           reload gains + flight config from NVS
//  0x10 LINK          u8 mode     store transport profile, then reboot
//
//  Modes:
//    MANUAL  pilot commands attitude / rate / thrust directly (CommanderPacket)
//    MANUAL  người lái ra lệnh thẳng tư thế / tốc độ góc / lực đẩy (CommanderPacket)
//    HOLD    attitude level, heading locked, thrust = hover_thrust
//    HOLD    tư thế cân bằng, khóa hướng mũi, lực đẩy = hover_thrust
//    AUTO    TAKEOFF ramp -> HOLD -> LAND ramp -> disarm
//    AUTO    bám TAKEOFF -> HOLD -> bám LAND -> ngắt kích hoạt
//
//  There is no barometer, ToF or GPS on this board, so HOLD holds a thrust
//  level rather than an altitude, and AUTO is a takeoff/hover/land sequence
//  rather than waypoint flight. Hover altitude is approximate and drifts —
//  the app says so rather than pretending otherwise.
//  Bo mạch này không có áp kế, ToF hay GPS, nên HOLD giữ mức lực đẩy chứ không
//  giữ độ cao, và AUTO là chuỗi cất cánh/giữ độ cao/hạ cánh chứ không bay theo
//  điểm hành trình. Độ cao khi hover chỉ là xấp xỉ và bị trôi — app nói rõ điều
//  đó thay vì giả vờ ngược lại.
#define FC_HDR               (CRTP_PORT_GENERIC << 4)   // 0x70
#define FC_CMD_SET_MODE      0x01
#define FC_CMD_TAKEOFF       0x02
#define FC_CMD_LAND          0x03
#define FC_CMD_SET_HOVER     0x04
#define FC_CMD_EMERGENCY     0x05
#define FC_CMD_SET_HEADING   0x06
#define FC_CMD_ARM           0x07
#define FC_CMD_DISARM        0x08
#define FC_CMD_SAVE          0x09
#define FC_CMD_DEFAULTS      0x0A
#define FC_CMD_LOAD          0x0B
#define FC_CMD_LINK          0x10
// Bench motor test: drive m1..m4 individually, PID bypassed. Disarmed-only,
// watchdog-held — see the motor_test_* state and the loop() hook.
// Thử động cơ trên bàn: lái m1..m4 riêng lẻ, bỏ qua PID. Chỉ khi chưa armed,
// giữ bằng watchdog — xem trạng thái motor_test_* và hook trong loop().
#define FC_CMD_MOTOR_TEST    0x0C
#define MOTOR_TEST_TIMEOUT_MS 300     // no GUI update for this long -> motors stop

// Flight modes and auto sequence states, as reported in the log TOC.
// Các chế độ bay và trạng thái chuỗi tự động, được báo trong log TOC.
#define MODE_MANUAL          0
#define MODE_HOLD            1
#define MODE_AUTO            2
#define MODE_FOLLOW          3              // fly toward the phone's GPS offset
#define MODE_RTH             4              // return to launch

#define AUTO_IDLE            0
#define AUTO_TAKEOFF         1
#define AUTO_HOLD            2
#define AUTO_LAND            3
#define AUTO_MISSION         4        // flying an onboard mission item (§12)
                                     // bay một mục nhiệm vụ trên bo (§12)

// ---- Onboard mission engine — AP_Mission port (PROTOCOL.md §12) ----------
// ---- Động cơ nhiệm vụ trên bo — chuyển thể từ AP_Mission (PROTOCOL.md §12) --
//  ArduPilot semantics (ArduPilot/ardupilot libraries/AP_Mission +
//  ArduCopter/mode_auto.cpp): a stored item list flown in AUTO, transferred
//  with the MAVLink MISSION_* handshake over CRTP port 7, channel 2.
//  Ngữ nghĩa ArduPilot (ArduPilot/ardupilot libraries/AP_Mission +
//  ArduCopter/mode_auto.cpp): danh sách mục được lưu, bay ở chế độ AUTO, truyền
//  bằng bắt tay MISSION_* của MAVLink qua CRTP cổng 7, kênh 2.
#define MSN_HDR              ((CRTP_PORT_GENERIC << 4) | 2)   // 0x72
#define MISSION_MAX_ITEMS    32
#define MISSION_STOP_RADIUS_M 2.5f    // default acceptance radius, m
#define MISSION_RADIUS_MIN_M  0.5f

#define MSN_CMD_COUNT        0x30
#define MSN_CMD_REQUEST      0x31
#define MSN_CMD_ITEM         0x32
#define MSN_CMD_ACK          0x33
#define MSN_CMD_REQUEST_LIST 0x34
#define MSN_CMD_SET_CURRENT  0x35
#define MSN_CMD_CURRENT      0x36
#define MSN_CMD_START        0x37

// MAV_CMD values (ArduPilot common.xml) — the only ones the runner executes;
// anything else is stored but skipped (PROTOCOL.md §12).
// Giá trị MAV_CMD (common.xml của ArduPilot) — những lệnh duy nhất runner chạy;
// lệnh khác chỉ được lưu và bỏ qua (PROTOCOL.md §12).
#define MAV_CMD_NAV_WAYPOINT 16
#define MAV_CMD_NAV_RTL      20
#define MAV_CMD_NAV_LAND     21
#define MAV_CMD_NAV_TAKEOFF  22
#define MAV_CMD_DO_JUMP      177

// MAV_MISSION result codes (common.xml) — the ACK payload.
// Mã kết quả MAV_MISSION (common.xml) — phần dữ liệu của ACK.
#define MAV_MISSION_ACCEPTED 0
#define MAV_MISSION_ERROR    1
#define MAV_MISSION_NO_SPACE 4
#define MAV_MISSION_INVALID  5
#define MAV_MISSION_DENIED   14

// One mission item — the MAVLink MISSION_ITEM_INT packing (PROTOCOL.md §12).
// p3/p4 arrive over MAVLink and in .waypoints files; the CRTP channel zeroes
// them (30-byte packet ceiling, §1).
// Một mục nhiệm vụ — dạng đóng gói MISSION_ITEM_INT của MAVLink (PROTOCOL.md
// §12). p3/p4 đến qua MAVLink và trong tệp .waypoints; kênh CRTP đặt chúng về 0
// (trần gói 30 byte, §1).
typedef struct {
  uint16_t seq;
  uint16_t command;          // MAV_CMD
  float    p1, p2, p3, p4;
  int32_t  lat_e7, lon_e7;   // degrees x 1e7
  float    alt;              // metres — stored and reported, never flown
} MissionItem;

// ---- MAVLink bridge (PROTOCOL.md §13) ------------------------------------
// ---- Cầu nối MAVLink (PROTOCOL.md §13) -----------------------------------
#define MAVLINK_UDP_PORT     14550   // the conventional MAVLink-over-WiFi port
#define MAVLINK_SYSID        1
#define MAVLINK_COMPID       1       // MAV_COMP_ID_AUTOPILOT1
#define MAVLINK_STX_V2       0xFD
#define MAVLINK_STX_V1       0xFE

// Flight-control tuning. Hover thrust lives in the CommanderPacket domain
// (0..65535) so the app can reuse its existing thrust mapping.
// Tinh chỉnh điều khiển bay. Lực đẩy hover nằm trong miền CommanderPacket
// (0..65535) để app dùng lại phép ánh xạ lực đẩy sẵn có của mình.
#define HOVER_THRUST_DEFAULT 30000         // ~46% — a starting guess, trim it
#define HOVER_THRUST_MIN     8000
#define HOVER_THRUST_MAX     55000
#define TAKEOFF_MS_DEFAULT   2000
#define LAND_MS_DEFAULT      2500
#define HEADING_KP           2.0f           // deg/s of yaw rate per deg of error

// ---- CRTP port 7, channel 1: navigation / phone reference ----------------
// ---- CRTP port 7, channel 1: dẫn đường / tham chiếu từ điện thoại --------
//  The phone reads its OWN GPS and compass (no SensorServer needed) and sends
//  the target as an offset from the launch point, already rotated into the
//  drone's launch heading frame. The drone then estimates where it is by
//  integrating its own accelerometer, and flies toward that offset.
//  Điện thoại đọc GPS và la bàn của CHÍNH nó (không cần SensorServer) và gửi
//  mục tiêu dạng độ lệch so với điểm cất cánh, đã quay sẵn vào khung hướng cất
//  cánh của drone. Drone sau đó ước lượng vị trí của mình bằng cách tích phân
//  gia tốc kế, rồi bay về phía độ lệch đó.
//
//  [0] 0x71 | [1] cmd | [2..] args        reply: [0x71][cmd][status u8]
//
//  0x20 TARGET   f32 x_m  f32 y_m   target offset from launch, launch frame
//                                    (+x = where the nose pointed at launch,
//                                     +y = 90 deg left of that)
//  0x20 TARGET   f32 x_m  f32 y_m   độ lệch mục tiêu so với điểm cất cánh, theo khung cất cánh
//                                    (+x = hướng mũi tại lúc cất cánh,
//                                     +y = lệch 90 độ sang trái hướng đó)
//  0x21 RESET    —                   zero the dead-reckoned position
//  0x21 RESET    —                   đặt lại vị trí tự dẫn đường quán tính về 0
//  0x22 ALIGN    f32 alpha_deg       compass bearing of the nose at launch;
//                                    the phone sends this so its GPS offsets
//                                    can be rotated into the drone's frame
//  0x22 ALIGN    f32 alpha_deg       phương hướng la bàn của mũi tại lúc cất cánh;
//                                    điện thoại gửi giá trị này để các độ lệch GPS
//                                    của nó được quay vào khung của drone
//
//  HONEST LIMIT: the position below is accelerometer double integration on
//  MEMS hardware. Bias makes it drift — metres within a minute. It is good
//  enough to fly a rough "toward the pilot" and nothing more. Both apps label
//  it approximate rather than implying GPS-grade tracking.
//  GIỚI HẠN THẬT: vị trí bên dưới là tích phân đôi gia tốc kế trên phần cứng MEMS.
//  Sai lệch làm nó trôi — vài mét trong một phút. Nó đủ để bay đại khái "về phía
//  người lái" và không hơn. Cả hai app đều ghi rõ đây là xấp xỉ chứ không ngụ ý
//  độ chính xác cấp GPS.
#define NAV_HDR              0x71
#define NAV_CMD_TARGET       0x20
#define NAV_CMD_RESET        0x21
#define NAV_CMD_ALIGN        0x22
#define NAV_CMD_ORIGIN       0x23     // launch latitude/longitude (§12, §13)
                                      // vĩ độ/kinh độ điểm cất cánh

// Dead-reckoning tuning.
// Tinh chỉnh tự dẫn đường quán tính.
#define NAV_ACC_LSB_PER_G    4096.0f       // MPU6500 at +-8 g -- must match MPU_REG_ACCEL_CFG in mpuInit()
#define NAV_GRAVITY          9.80665f
// Velocity leak per second. Bounds the velocity error that accelerometer bias
// would otherwise grow without limit; costs a little of any sustained move.
// Rò rỉ vận tốc mỗi giây. Giới hạn sai số vận tốc mà sai lệch gia tốc kế vốn sẽ
// làm tăng vô hạn; đổi lại mất một chút khi di chuyển liên tục.
#define NAV_VEL_LEAK_PER_S   0.60f
#define NAV_TARGET_TIMEOUT_MS 3000         // stale phone reference -> hold
#define NAV_STOP_BAND_M      0.6f          // within this of the target, stop
#define NAV_MAX_TILT_DEG     12.0f
// Sensor axis signs for the horizontal plane. Flip here if "follow" flies
// away from you instead of toward you — the same trick as GYRO_*_SIGN.
// Dấu các trục cảm biến trên mặt phẳng ngang. Đảo tại đây nếu chế độ "follow" bay
// ra xa bạn thay vì bay về phía bạn — cùng cách làm như với GYRO_*_SIGN.
#define NAV_FWD_SIGN         1             // +1 if sensor +Y is nose-forward
#define NAV_LEFT_SIGN        1             // +1 if sensor +X is to the left
// Sign of the yaw that rotates body axes into the launch frame. This is a
// separate question from the two above: not the accelerometer, but which way
// heading_deg turns.
// Dấu của yaw dùng để quay các trục thân vào khung cất cánh. Đây là câu hỏi riêng
// so với hai mục trên: không phải gia tốc kế, mà là heading_deg quay theo chiều nào.
//
// heading_deg is a compass bearing: it wraps to 0..360 and hold_heading_deg is
// assigned straight from the phone's FC_CMD_SET_HEADING, which is a compass
// bearing. A compass bearing rises as the nose turns RIGHT.
// heading_deg là phương hướng la bàn: nó gói về 0..360 và hold_heading_deg được
// gán thẳng từ FC_CMD_SET_HEADING của điện thoại, vốn là phương hướng la bàn.
// Phương hướng la bàn tăng khi mũi máy bay quay sang PHẢI.
//
// The body->launch rotation needs the opposite sign: +y is the drone's LEFT, so
// turning right must rotate the nose toward -y. With +1 the two disagree and
// navSetpoints() rolls toward the target's cross-axis instead of pitching at
// it -- up to 8.66 deg of spurious roll at 30 deg of yaw, and a spiral away
// from anything not dead ahead. test/yawrot_test.cpp pins both directions.
// Phép quay thân -> cất cánh cần dấu ngược lại: +y là bên TRÁI của drone, nên
// quay phải phải làm mũi hướng về -y. Với +1 thì hai thứ mâu thuẫn và
// navSetpoints() sẽ roll về trục phụ của mục tiêu thay vì pitch về phía nó --
// tới 8.66 độ roll giả khi yaw 30 độ, và xoắn ốc ra xa bất kỳ mục tiêu nào
// không thẳng phía trước. test/yawrot_test.cpp cố định cả hai chiều.
//
// This feeds BOTH navUpdate() and navSetpoints(), so they stay an exact
// inverse pair whichever way it is set.
// Giá trị này cấp cho CẢ navUpdate() và navSetpoints(), nên hai hàm luôn là một
// cặp nghịch đảo chính xác bất kể được đặt theo chiều nào.
#define NAV_HEADING_SIGN     -1

typedef struct {
  const char *group;
  const char *name;
  uint8_t     ctype;    // PT_* for params, LT_* for logs
                        // PT_* cho tham số, LT_* cho log
  uint8_t     size;     // value width in bytes
                        // độ rộng giá trị tính bằng byte
  uint8_t     access;   // params only: TOC_ACCESS_*
                        // chỉ dành cho tham số: TOC_ACCESS_*
  void       *value;    // live variable the entry exposes
                        // biến trực tiếp mà mục này phơi ra
  float       lo, hi;   // RW params only: phone writes are clamped into [lo, hi]
                        // and NaN/inf is refused (see paramStore)
                        // chỉ tham số RW: giá trị ghi từ điện thoại bị kẹp trong [lo, hi]
                        // và NaN/inf bị từ chối (xem paramStore)
} TocEntry;

typedef struct {
  bool     used;
  bool     started;
  uint8_t  id;
  uint8_t  period;      // 10 ms units, exactly as the client sends it
                        // đơn vị 10 ms, đúng như client gửi
  uint8_t  nvars;
  uint8_t  var_id[LOG_MAX_VARS];   // indices into LOG_TOC
                                   // các chỉ mục vào LOG_TOC
  uint32_t start_ms;    // timestamp origin
                        // mốc thời gian gốc
  uint32_t last_ms;     // next-send pacing
                        // nhịp cho lần gửi kế tiếp
} LogBlock;

// ============================ MPU6500 REGISTERS ============================
#define MPU_REG_SMPLRT_DIV  0x19
#define MPU_REG_CONFIG      0x1A
#define MPU_REG_GYRO_CFG    0x1B
#define MPU_REG_ACCEL_CFG   0x1C
#define MPU_REG_ACCEL_XOUT  0x3B
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_REG_PWR_MGMT_2  0x6C
#define MPU_REG_WHO_AM_I    0x75
#define MPU_WHO_AM_I_6500   0x70
#define MPU_WHO_AM_I_9250   0x71

// ============================ GLOBAL STATE =================================
// --- sensor ---
int16_t  raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
int32_t  gyro_x_cal, gyro_y_cal, gyro_z_cal;
float    gyro_pitch, gyro_roll, gyro_yaw;       // body rates, deg/s (for PID)
float    angle_pitch_acc, angle_roll_acc;       // raw accel tilt, deg (filter input)
float    angle_pitch_deg = 0.0f;                // filtered attitude angles, deg
float    angle_roll_deg  = 0.0f;
float    acc_x_g, acc_y_g, acc_z_g;             // accelerometer, g (for the IMU log)
                                                  // gia tốc kế, g (cho khối log IMU)
bool     gyro_angles_set = false;

// --- receiver ---
int16_t  sbus_raw[16];                            // 0..2047
int16_t  rc[5];                                   // mapped 1000..2000
bool     sbus_frame_ok = false;
uint32_t sbus_last_frame_ms = 0;
uint8_t  sbus_buf[SBUS_FRAME_LEN];
uint8_t  sbus_idx = 0;

// --- control ---
float    pid_i_mem_roll, pid_i_mem_pitch, pid_i_mem_yaw;
float    pid_output_roll, pid_output_pitch, pid_output_yaw;
float    pid_last_roll_err, pid_last_pitch_err, pid_last_yaw_err;
float    roll_setpoint, pitch_setpoint, yaw_setpoint;
float    roll_level_adjust, pitch_level_adjust;

// Per-term breakdown of the last pidCompute(), kept only so the phone's tuning
// screen can chart which term is doing the work (P fighting D, I windup, ...).
// Same sign convention as the corresponding pid_output_*.
// Phân rã từng thành phần của pidCompute() gần nhất, chỉ giữ lại để màn hình tinh
// chỉnh của điện thoại vẽ được thành phần nào đang làm việc (P đối kháng D, I windup,...).
// Cùng quy ước dấu với pid_output_* tương ứng.
float    pid_p_roll,  pid_i_roll,  pid_d_roll;
float    pid_p_pitch, pid_i_pitch, pid_d_pitch;
float    pid_p_yaw,   pid_i_yaw,   pid_d_yaw;

int16_t  motor_rf, motor_rr, motor_lr, motor_lf;  // 1000..2000 µs convention

// --- motor test (bench only, CRTP FC_CMD_MOTOR_TEST) ---
// Individual motor thrusts for bench testing (motor direction, vibration, prop
// balance) with the PID loop bypassed. Only honoured while DISARMED, and held
// only while the GUI re-sends within MOTOR_TEST_TIMEOUT_MS — a dropped link or
// a closed window stops the motors by itself.
// Gần các biến motor:
// Lực đẩy từng động cơ riêng lẻ để thử trên bàn (hướng động cơ, rung, cân bằng
// cánh quạt) với vòng PID bị bỏ qua. Chỉ có hiệu lực khi CHƯA armed, và chỉ giữ
// trong khi GUI gửi lại trong MOTOR_TEST_TIMEOUT_MS — mất liên kết hay đóng cửa
// sổ là động cơ tự dừng.
bool     motor_test_active = false;
uint32_t motor_test_last_ms = 0;
uint16_t motor_test_m[4] = { 1000, 1000, 1000, 1000 };  // m1..m4, µs

// --- status ---
bool     armed = false;
bool     arm_switch_seen = false;   // becomes true once CH5 reads high
uint32_t arm_stick_ms = 0;
uint32_t disarm_stick_ms = 0;

// --- WiFi / CRTP link ---
WiFiUDP   Udp;
IPAddress app_ip;                    // learned from first inbound datagram
bool      app_ip_known = false;
uint32_t  crtp_last_rx_ms = 0;
uint8_t   crtp_rx_buf[64];

// commander setpoints (from CRTP CommanderPacket)
// các setpoint từ commander (từ CRTP CommanderPacket)
float    cmd_roll_deg = 0.0f;
float    cmd_pitch_deg = 0.0f;
float    cmd_yaw_dps = 0.0f;
uint16_t cmd_thrust = 0;             // 0..65535
bool     cmd_valid = false;          // a commander packet has been parsed
uint32_t cmd_last_ms = 0;
uint32_t loop_timer;
uint32_t last_telemetry_ms = 0;
float    vbat = 0.0f;
bool     vbat_warn = false;
bool     vbat_crit = false;            // filtered vbat below critical for VBAT_CRIT_HOLD_MS
uint32_t vbat_low_since_ms = 0;
bool     crtp_thrust_was_low = false;  // CRTP manual arming needs thrust seen at zero first

// --- transport ---
uint8_t  link_mode   = LINK_MODE_WIFI_AP;   // live profile, from NVS at boot
bool     link_ready  = false;               // a transport is up and carrying CRTP
uint8_t  pending_link_mode = 0xFF;          // set by FC_CMD_LINK, reboot follows
uint32_t reboot_at_ms = 0;
char     sta_ssid[33] = WIFI_STA_SSID;
char     sta_pass[65] = WIFI_STA_PASS;

// --- flight mode ---
uint8_t  flight_mode = MODE_MANUAL;
uint8_t  auto_state  = AUTO_IDLE;
uint32_t auto_phase_start_ms = 0;
uint16_t hover_thrust = HOVER_THRUST_DEFAULT;   // 0..65535, latched by SET_HOVER
uint16_t takeoff_ms   = TAKEOFF_MS_DEFAULT;
uint16_t land_ms      = LAND_MS_DEFAULT;
float    heading_deg  = 0.0f;               // integrated from gyro-z, 0..360
float    hold_heading_deg = 0.0f;           // heading locked when hold/auto engaged

// --- navigation (launch frame: +x = nose at launch, +y = 90 deg left) ---
// --- dẫn đường (khung cất cánh: +x = hướng mũi lúc cất cánh, +y = lệch 90 độ sang trái) ---
float    pos_x = 0.0f, pos_y = 0.0f;        // dead-reckoned offset from launch, m
float    vel_x = 0.0f, vel_y = 0.0f;        // m/s
float    heading_ref_deg = 0.0f;            // heading_deg at launch
float    align_alpha_deg = 0.0f;            // compass bearing of nose at launch
float    tgt_x = 0.0f, tgt_y = 0.0f;        // target offset from launch, m
bool     tgt_valid = false;                 // the phone has sent a recent target
uint32_t tgt_last_ms = 0;
float    nav_kp = 1.2f;                     // deg of tilt per m of position error

// --- onboard mission + MAVLink bridge (PROTOCOL.md §12, §13) ---
// --- nhiệm vụ trên bo + cầu nối MAVLink (PROTOCOL.md §12, §13) ---
MissionItem mission_items[MISSION_MAX_ITEMS];
MissionItem mission_stage[MISSION_MAX_ITEMS];  // upload staging — commit on ACK
uint16_t mission_count = 0;
int16_t  mission_current = -1;         // seq index the runner is on, -1 = none
bool     mission_active = false;       // the runner is flying the mission
uint16_t mission_stage_count = 0;      // items expected in the current upload
uint16_t mission_stage_seq = 0;        // next seq the drone wants staged
uint16_t jump_exec_count[MISSION_MAX_ITEMS];  // DO_JUMP executions consumed
bool     home_valid = false;           // ORIGIN received — lat/lon known
double   home_lat = 0.0, home_lon = 0.0;

// MAVLink bridge runtime (§13). The socket listens on 14550 for any ArduPilot
// GCS; mav_gcs_ip is learned from the first packet, the same way linkSend()
// learns the app address.
// Runtime của cầu nối MAVLink (§13). Socket nghe ở cổng 14550 cho mọi GCS
// ArduPilot; mav_gcs_ip học từ gói đầu tiên, giống cách linkSend() học địa chỉ app.
WiFiUDP MavUdp;
static bool      mav_gcs_known = false;
static IPAddress mav_gcs_ip;
static uint16_t  mav_gcs_port = 0;
static uint8_t   mav_seq = 0;              // MAVLink tx sequence counter
static uint32_t  mav_heartbeat_ms = 0;
static uint32_t  mav_pos_ms = 0;
static bool      mav_mission_current_dirty = true;
static bool      mav_home_dirty = true;    // (re)send HOME_POSITION

// Setpoints the flight-mode stage hands to the PID. loop() writes these and
// never touches the PID inputs directly.
// Các setpoint mà tầng chế độ bay giao cho PID. loop() ghi các giá trị này và
// không bao giờ chạm thẳng vào các đầu vào của PID.
//
// Roll/pitch carry BOTH an attitude target and a rate feed-forward, because
// the two control sources speak different languages: CRTP CommanderPackets
// command a tilt ANGLE, while SBUS sticks command a body RATE. Holding mode
// wants angle 0 with no rate; manual wants whatever the pilot asked for.
// Roll/pitch mang CẢ mục tiêu tư thế lẫn feed-forward tốc độ góc, vì hai nguồn
// điều khiển nói hai thứ tiếng khác nhau: CRTP CommanderPacket ra lệnh một GÓC
// nghiêng, còn cần SBUS ra lệnh TỐC ĐỘ GÓC thân. Chế độ giữ muốn góc 0 và không
// có tốc độ góc; chế độ manual lấy đúng giá trị người lái yêu cầu.
float    sp_roll_deg    = 0.0f;             // attitude target, deg
float    sp_pitch_deg   = 0.0f;
float    sp_roll_rate   = 0.0f;             // rate feed-forward, deg/s
float    sp_pitch_rate  = 0.0f;
float    sp_yaw_rate    = 0.0f;             // yaw rate setpoint, deg/s
int16_t  sp_throttle_us = 1000;             // 1000..2000

// =============================================================================
//  MPU6500 SPI low-level
// =============================================================================
inline void mpuSelect()   { digitalWrite(PIN_MPU_CS, LOW);  }
inline void mpuDeselect() { digitalWrite(PIN_MPU_CS, HIGH); }

void mpuWrite(uint8_t reg, uint8_t val) {
  mpuSelect();
  SPI.transfer(reg & 0x7F);       // bit7 = 0 -> write
                                  // bit7 = 0 -> ghi
  SPI.transfer(val);
  mpuDeselect();
  delayMicroseconds(10);
}

uint8_t mpuRead(uint8_t reg) {
  mpuSelect();
  SPI.transfer(reg | 0x80);       // bit7 = 1 -> read
                                  // bit7 = 1 -> đọc
  uint8_t v = SPI.transfer(0x00);
  mpuDeselect();
  return v;
}

void mpuReadBurst(uint8_t reg, uint8_t *buf, uint8_t n) {
  mpuSelect();
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < n; i++) buf[i] = SPI.transfer(0x00);
  mpuDeselect();
}

// =============================================================================
//  MPU6500 init & read
// =============================================================================
bool mpuInit() {
  pinMode(PIN_MPU_CS, OUTPUT);
  mpuDeselect();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_MPU_CS);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  delay(50);
  mpuWrite(MPU_REG_PWR_MGMT_1, 0x80);   // device reset
                                        // reset thiết bị
  delay(100);
  mpuWrite(MPU_REG_PWR_MGMT_1, 0x01);   // clock = PLL with X-axis gyro
                                        // clock = PLL với gyro trục X
  delay(10);
  mpuWrite(MPU_REG_PWR_MGMT_2, 0x00);   // all axes on
                                        // bật tất cả các trục
  mpuWrite(MPU_REG_SMPLRT_DIV, 0x03);   // 1kHz/(1+3) = 250 Hz sample rate
                                        // tần số lấy mẫu 1kHz/(1+3) = 250 Hz
  mpuWrite(MPU_REG_CONFIG,     0x02);   // DLPF 92 Hz gyro BW — good for 250 Hz loop
                                        // DLPF 92 Hz băng thông gyro — phù hợp vòng lặp 250 Hz
  mpuWrite(MPU_REG_GYRO_CFG,   0x08);   // ±500 dps  (65.5 LSB/(deg/s))
  mpuWrite(MPU_REG_ACCEL_CFG,  0x10);   // ±8 g      (4096 LSB/g)
  delay(10);

  uint8_t who = mpuRead(MPU_REG_WHO_AM_I);
  return (who == MPU_WHO_AM_I_6500 || who == MPU_WHO_AM_I_9250);
}

void mpuReadRaw() {
  uint8_t buf[14];
  mpuReadBurst(MPU_REG_ACCEL_XOUT, buf, 14);
  raw_ax = (int16_t)(buf[0]  << 8 | buf[1]);
  raw_ay = (int16_t)(buf[2]  << 8 | buf[3]);
  raw_az = (int16_t)(buf[4]  << 8 | buf[5]);
  /* buf[6..7] = temperature (skip) */
  /* buf[6..7] = nhiệt độ (bỏ qua) */
  raw_gx = (int16_t)(buf[8]  << 8 | buf[9]);
  raw_gy = (int16_t)(buf[10] << 8 | buf[11]);
  raw_gz = (int16_t)(buf[12] << 8 | buf[13]);
}

void mpuCalibrateGyro() {
  Serial.println(F("[MPU] Calibrating gyro — keep the drone FLAT and STILL..."));
  int32_t sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 2000; i++) {
    mpuReadRaw();
    sx += raw_gx; sy += raw_gy; sz += raw_gz;
    delay(3);
  }
  gyro_x_cal = sx / 2000;
  gyro_y_cal = sy / 2000;
  gyro_z_cal = sz / 2000;
  Serial.printf("[MPU] Gyro bias  X:%ld  Y:%ld  Z:%ld\n",
                (long)gyro_x_cal, (long)gyro_y_cal, (long)gyro_z_cal);
}

// =============================================================================
//  SBUS  (UART2, 100000 baud, 8E2, inverted)
// =============================================================================
void sbusInit() {
  // 5th arg `true` = invert RX — required for standard SBUS levels
  // Đối số thứ 5 `true` = đảo RX — bắt buộc với mức điện SBUS tiêu chuẩn
  Serial2.begin(100000, SERIAL_8E2, PIN_SBUS_RX, -1, true);
}

// Decode 16 x 11-bit channels out of a 25-byte SBUS frame
// Giải mã 16 kênh 11-bit từ một khung SBUS 25 byte
void sbusDecode(const uint8_t *b) {
  sbus_raw[0]  = (b[1]        | b[2]  << 8)                 & 0x7FF;
  sbus_raw[1]  = (b[2]  >> 3  | b[3]  << 5)                 & 0x7FF;
  sbus_raw[2]  = (b[3]  >> 6  | b[4]  << 2 | b[5] << 10)    & 0x7FF;
  sbus_raw[3]  = (b[5]  >> 1  | b[6]  << 7)                 & 0x7FF;
  sbus_raw[4]  = (b[6]  >> 4  | b[7]  << 4)                 & 0x7FF;
  sbus_raw[5]  = (b[7]  >> 7  | b[8]  << 1 | b[9]  << 9)    & 0x7FF;
  sbus_raw[6]  = (b[9]  >> 2  | b[10] << 6)                 & 0x7FF;
  sbus_raw[7]  = (b[10] >> 5  | b[11] << 3)                 & 0x7FF;
  sbus_raw[8]  = (b[12]       | b[13] << 8)                 & 0x7FF;
  sbus_raw[9]  = (b[13] >> 3  | b[14] << 5)                 & 0x7FF;
  sbus_raw[10] = (b[14] >> 6  | b[15] << 2 | b[16] << 10)   & 0x7FF;
  sbus_raw[11] = (b[16] >> 1  | b[17] << 7)                 & 0x7FF;
  sbus_raw[12] = (b[17] >> 4  | b[18] << 4)                 & 0x7FF;
  sbus_raw[13] = (b[18] >> 7  | b[19] << 1 | b[20] << 9)    & 0x7FF;
  sbus_raw[14] = (b[20] >> 2  | b[21] << 6)                 & 0x7FF;
  sbus_raw[15] = (b[21] >> 5  | b[22] << 3)                 & 0x7FF;

  for (int i = 0; i < 4; i++) {
    int32_t v = map(sbus_raw[i], SBUS_MIN, SBUS_MAX, RC_MIN, RC_MAX);
    rc[i] = constrain(v, RC_MIN, RC_MAX);
  }
  // CH5 as a simple high/low arm channel -> map to 1000/2000
  // CH5 như một kênh kích hoạt cao/thấp đơn giản -> ánh xạ về 1000/2000
  rc[CH_ARM] = (sbus_raw[CH_ARM] > 1024) ? 2000 : 1000;

  sbus_frame_ok = true;
  sbus_last_frame_ms = millis();
}

// A frame is used only when it is well-formed AND the receiver still hears the
// transmitter. Layout: [0] 0x0F header | [1..22] channels |
// [23] flags (bit2 = frame lost, bit3 = failsafe) | [24] footer.
// The footer is 0x00 on plain SBUS; SBUS2 receivers cycle 0x04/0x14/0x24/0x34.
// A receiver in failsafe often keeps sending frames carrying the LAST stick
// positions, so a failsafe frame must NOT refresh sbus_last_frame_ms: letting
// the FAILSAFE_MS timeout run out is what stops the motors.
// (The old check read the flags from byte 24, the footer, so it never saw them.)
// Một khung chỉ được dùng khi nó đúng cấu trúc VÀ bộ thu vẫn còn nghe thấy máy
// phát. Bố cục: [0] 0x0F header | [1..22] kênh |
// [23] cờ (bit2 = mất khung, bit3 = failsafe) | [24] footer.
// Footer là 0x00 trên SBUS thường; bộ thu SBUS2 luân phiên 0x04/0x14/0x24/0x34.
// Bộ thu đang ở failsafe thường vẫn gửi các khung mang vị trí cần gạt LẦN CUỐI,
// nên khung failsafe KHÔNG được làm mới sbus_last_frame_ms: để hết thời gian
// chờ FAILSAFE_MS mới là thứ dừng động cơ.
// (Bản kiểm tra cũ đọc cờ từ byte 24, tức footer, nên không bao giờ thấy chúng.)
bool sbusFrameUsable(const uint8_t *f) {
  if (f[0] != 0x0F) return false;
  const uint8_t footer = f[24];
  if (footer != 0x00 && (footer & 0x0F) != 0x04) return false;
  if (f[23] & 0x08) return false;      // receiver failsafe
  if (f[23] & 0x04) return false;      // this frame was lost
  return true;
}

// Pull any pending bytes; parse frames as they complete
// Kéo các byte còn chờ; giải mã khung khi chúng hoàn tất
void sbusPoll() {
  while (Serial2.available()) {
    uint8_t c = (uint8_t)Serial2.read();
    if (sbus_idx == 0 && c != 0x0F) continue;        // wait for header
    sbus_buf[sbus_idx++] = c;
    if (sbus_idx >= SBUS_FRAME_LEN) {
      sbus_idx = 0;
      if (sbusFrameUsable(sbus_buf)) sbusDecode(sbus_buf);
    }
  }
}

// =============================================================================
//  CRTP / UDP link  (Espressif ESP-Drone app protocol)
//  Framing on the wire:  [CRTP ...][uint8 checksum]  where checksum is the
//  8-bit sum of the CRTP bytes INCLUDING the header byte.
//  Đóng khung trên đường truyền:  [CRTP ...][uint8 checksum]  trong đó checksum
//  là tổng 8-bit của các byte CRTP BAO GỒM cả byte header.
// =============================================================================
uint8_t crtpChecksum(const uint8_t *data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return (uint8_t)(sum & 0xFF);
}

// =============================================================================
//  TRANSPORT  — WiFi UDP / Bluetooth Classic SPP / BLE GATT
//
//  The CRTP layer only ever calls linkSend() and hands inbound bytes to
//  crtpReceiveFramed()/crtpReceiveRaw(). Everything below picks the active
//  transport from link_mode, which is read back from NVS at boot.
//  Tầng CRTP chỉ gọi linkSend() và chuyển byte vào cho
//  crtpReceiveFramed()/crtpReceiveRaw(). Mọi thứ bên dưới chọn tầng truyền đang
//  hoạt động từ link_mode, giá trị đọc lại từ NVS khi khởi động.
//
//  Framing differs by transport because the clients differ:
//    UDP / SPP : [CRTP ...][uint8 checksum]   checksum over the CRTP bytes
//    BLE       : raw CRTP wrapped in control-byte segments (see bleSend)
//  Cách đóng khung khác nhau theo tầng truyền vì client khác nhau:
//    UDP / SPP : [CRTP ...][uint8 checksum]   checksum trên các byte CRTP
//    BLE       : CRTP thô bọc trong các đoạn có control byte (xem bleSend)
// =============================================================================
void crtpDispatch(const uint8_t *p, size_t len);

// The flight-mode stage is defined further down but called from the CRTP
// handlers above it. Declaring them here keeps the sketch order-independent
// for a real C++ pass as well as the Arduino preprocessor.
// Tầng chế độ bay được định nghĩa phía dưới nhưng được gọi từ các handler CRTP
// ở trên. Khai báo các hàm ở đây giúp sketch không phụ thuộc thứ tự, cả với một
// lượt biên dịch C++ thật lẫn với bộ tiền xử lý Arduino.
void flightSetMode(uint8_t mode);
bool flightTakeoff(uint16_t thrust);
bool armChecksOk(const char *who);
void flightLand();
void flightEmergency();
void flightConfigDefaults();
void flightSetpoints(uint32_t now, bool use_crtp, bool sbus_ok);
void navReset();
void navUpdate(float dt, bool holding);
void navSetpoints();
void cfgSave();
void cfgLoad();
void armUpdate(uint32_t now, bool use_crtp);
void logUpdate();
void motorStop();
void pidReset();

// Mission engine + MAVLink bridge — also defined further down (§12, §13).
// Động cơ nhiệm vụ + cầu nối MAVLink — cũng định nghĩa phía dưới (§12, §13).
void crtpHandleMission(const uint8_t *p, size_t len);
bool missionStart();
void missionStop();
void missionTick(uint32_t now);
void missionSendCurrent();
void missionClear();
void missionSave();
void missionLoad();
void mavlinkPoll();

Preferences nvs;

bool linkUsesUdp(uint8_t mode) {
  return mode == LINK_MODE_WIFI_AP     || mode == LINK_MODE_WIFI_STA ||
         mode == LINK_MODE_WIFI_AP_BLE || mode == LINK_MODE_WIFI_STA_BLE;
}
bool linkUsesBle(uint8_t mode) {
  return mode == LINK_MODE_BLE || mode == LINK_MODE_WIFI_AP_BLE ||
         mode == LINK_MODE_WIFI_STA_BLE;
}

// Inbound UDP/SPP: `data` is [CRTP ...][checksum].
// Vào từ UDP/SPP: `data` là [CRTP ...][checksum].
void crtpReceiveFramed(const uint8_t *data, size_t len) {
  if (len < 2 || len > CRTP_MAX_SIZE + 1) return;
  if (crtpChecksum(data, len - 1) != data[len - 1]) return;
  crtpDispatch(data, len - 1);
  crtp_last_rx_ms = millis();
}

// Inbound BLE: the transport already framed it, so no checksum is present.
// Vào từ BLE: tầng truyền đã đóng khung sẵn, nên không có checksum.
void crtpReceiveRaw(const uint8_t *data, size_t len) {
  if (len < 1 || len > CRTP_MAX_SIZE) return;
  crtpDispatch(data, len);
  crtp_last_rx_ms = millis();
}

// ---------------------------------------------------------------------------
//  Bluetooth Classic SPP
// ---------------------------------------------------------------------------
#if LINK_USE_BT_SPP
BluetoothSerial *btSpp = nullptr;
// SPP is a byte stream: the phone's writes can arrive split or merged, so packet
// boundaries are recovered the way BtSppLink.java's reader does it -- the
// shortest length whose trailing checksum validates. CommanderPackets are only
// accepted at their real length (16), so a chance checksum match can never turn
// part of one into a different command.
// Everything here is non-blocking. The old Stream::readBytes() waited up to its
// 1 s default timeout for a full buffer, freezing the flight loop.
// SPP là một luồng byte: các lần ghi từ điện thoại có thể bị tách hoặc ghép, nên
// ranh giới gói tin được khôi phục theo cách trình đọc của BtSppLink.java -- độ
// dài ngắn nhất có checksum cuối gói hợp lệ. CommanderPacket chỉ được chấp nhận
// ở đúng độ dài thật (16), nên một checksum trùng ngẫu nhiên không bao giờ biến
// một phần gói thành lệnh khác.
// Mọi thứ ở đây đều không chặn. Stream::readBytes() cũ đợi tới hết thời gian chờ
// mặc định 1 s cho đủ bộ đệm, làm đông cứng vòng lặp bay.
#define BT_FRAME_MAX  (CRTP_MAX_SIZE + 1)
uint8_t bt_acc[2 * BT_FRAME_MAX];
size_t  bt_acc_len = 0;

// >0: a whole frame of that length is at the head of bt_acc
//  0: need more bytes
// -1: the head byte cannot start a frame -- drop it and resync
// >0: một khung trọn vẹn độ dài đó đang ở đầu bt_acc
//  0: cần thêm byte
// -1: byte đầu không thể mở một khung -- bỏ nó và đồng bộ lại
int btFrameLen() {
  if (bt_acc_len < 2) return 0;
  if (((bt_acc[0] >> 4) & 0x0F) == CRTP_PORT_COMMANDER) {
    if (bt_acc_len < CMD_PACKET_SIZE) return 0;
    return crtpChecksum(bt_acc, CMD_PACKET_SIZE - 1) == bt_acc[CMD_PACKET_SIZE - 1]
               ? CMD_PACKET_SIZE : -1;
  }
  size_t maxl = bt_acc_len < BT_FRAME_MAX ? bt_acc_len : BT_FRAME_MAX;
  for (size_t l = 2; l <= maxl; l++)
    if (crtpChecksum(bt_acc, l - 1) == bt_acc[l - 1]) return (int)l;
  return bt_acc_len >= BT_FRAME_MAX ? -1 : 0;
}

void btPoll() {
  if (!btSpp) return;
  while (bt_acc_len < sizeof(bt_acc)) {
    int c = btSpp->read();               // returns -1 at once when empty
    if (c < 0) break;
    bt_acc[bt_acc_len++] = (uint8_t)c;
  }
  for (;;) {
    int l = btFrameLen();
    if (l == 0) break;
    if (l > 0) crtpReceiveFramed(bt_acc, (size_t)l);
    size_t drop = (l < 0) ? 1 : (size_t)l;
    memmove(bt_acc, bt_acc + drop, bt_acc_len - drop);
    bt_acc_len -= drop;
  }
}

bool btInit(const char *name) {
  btSpp = new BluetoothSerial();
  if (!btSpp->begin(name)) {
    Serial.println(F("[BT] BluetoothSerial.begin failed (BT not compiled in?)"));
    return false;
  }
  Serial.printf("[BT] SPP '%s' ready\n", name);
  return true;
}
#endif

// ---------------------------------------------------------------------------
//  BLE GATT — Bitcraze Crazyflie service, so the Android BleLink works as-is.
//  BLE GATT — service Bitcraze Crazyflie, để BleLink trên Android chạy nguyên bản.
// ---------------------------------------------------------------------------
#if LINK_USE_BLE
static const char *BLE_UUID_SERVICE  = "00000201-1C7F-4F9E-947B-43B7C00A9A08";
static const char *BLE_UUID_CRTP     = "00000202-1C7F-4F9E-947B-43B7C00A9A08";
static const char *BLE_UUID_CRTPUP   = "00000203-1C7F-4F9E-947B-43B7C00A9A08";
static const char *BLE_UUID_CRTPDOWN = "00000204-1C7F-4F9E-947B-43B7C00A9A08";

BLECharacteristic *bleCrtpDown = nullptr;   // notify, drone -> phone
bool     ble_connected = false;
uint8_t  ble_seg_pid   = 0;
uint8_t  ble_rx_buf[CRTP_MAX_SIZE];
size_t   ble_rx_len    = 0;
uint8_t  ble_rx_pid    = 0xFF;

// Split a CRTP datagram into 20-byte control-byte-framed notifications.
//   seg[0] = bit7 start | bits6..5 pid | bits4..0 (totalLen-1)
//   seg[1..] = CRTP bytes
// Tách một gói CRTP thành các notification 20 byte có control byte.
bool bleSend(const uint8_t *data, size_t len) {
  if (!bleCrtpDown || !ble_connected || len == 0 || len > CRTP_MAX_SIZE) return false;
  uint8_t seg[BLE_SEG_BYTES + 1];
  size_t off = 0;
  while (off < len) {
    size_t n = len - off;
    if (n > BLE_SEG_BYTES) n = BLE_SEG_BYTES;
    uint8_t ctrl = (uint8_t)((ble_seg_pid & 0x03) << 5) | (uint8_t)((len - 1) & 0x1F);
    if (off == 0) ctrl |= 0x80;
    seg[0] = ctrl;
    memcpy(seg + 1, data + off, n);
    bleCrtpDown->setValue(seg, (uint8_t)(n + 1));
    bleCrtpDown->notify();
    off += n;
  }
  ble_seg_pid = (uint8_t)((ble_seg_pid + 1) & 0x03);
  return true;
}

// Copy the characteristic value out, whichever core version we are on.
// Chép giá trị characteristic ra ngoài, bất kể đang ở bản core nào.
static size_t bleReadValue(BLECharacteristic *ch, uint8_t *out, size_t cap) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  String v = ch->getValue();
  size_t n = (size_t)v.length();
  if (n > cap) n = cap;
  memcpy(out, v.c_str(), n);
#else
  std::string v = ch->getValue();
  size_t n = v.length();
  if (n > cap) n = cap;
  memcpy(out, v.data(), n);
#endif
  return n;
}

// 0x202: the client sends short packets raw, with no control byte.
// 0x202: client gửi các gói ngắn dạng thô, không có control byte.
class BleRawCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) {
    uint8_t buf[CRTP_MAX_SIZE];
    size_t n = bleReadValue(ch, buf, sizeof(buf));
    if (n >= 1) crtpReceiveRaw(buf, n);
  }
};

// 0x203: long packets arrive here segmented with a control byte.
// 0x203: các gói dài đến đây theo từng đoạn có control byte.
class BleSegCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) {
    uint8_t buf[BLE_SEG_BYTES + 1];
    size_t n = bleReadValue(ch, buf, sizeof(buf));
    if (n < 2) return;

    uint8_t ctrl = buf[0];
    bool    start = (ctrl & 0x80) != 0;
    uint8_t pid   = (uint8_t)((ctrl >> 5) & 0x03);
    // The length field is (total CRTP length - 1). A packet is single-segment
    // when the whole thing fits here, NOT merely when that field reads < 20:
    // the field saturates at 32 and a 20-byte datagram would otherwise be
    // mistaken for a single segment it cannot fit into.
    // Trường độ dài là (tổng độ dài CRTP - 1). Một gói là một đoạn duy nhất khi
    // toàn bộ gói nằm gọn trong đây, CHỨ không chỉ khi trường đó < 20:
    // trường này bão hòa ở 32 và một gói 20 byte sẽ bị hiểu nhầm thành một đoạn
    // duy nhất mà nó không thể chứa được.
    size_t total = (size_t)((ctrl & 0x1F) + 1);
    size_t body  = n - 1;

    if (start) {
      ble_rx_len = 0;
      ble_rx_pid = pid;
      if (body > sizeof(ble_rx_buf)) return;
      memcpy(ble_rx_buf, buf + 1, body);
      ble_rx_len = body;
      if (total <= body) {
        // The length field is authoritative. A well-behaved client sends
        // exactly `total` bytes here, but dispatch what was announced rather
        // than what happened to arrive, so padding cannot reach the parser.
        // Trường độ dài là nguồn quyết định. Client đúng chuẩn gửi đúng `total`
        // byte vào đây, nhưng hãy xử lý đúng số đã báo chứ không phải số byte thực
        // tế đến, để phần đệm không lọt vào bộ phân tích.
        crtpReceiveRaw(ble_rx_buf, total);
        ble_rx_len = 0;
      }
    } else {
      if (pid != ble_rx_pid || ble_rx_len == 0) { ble_rx_len = 0; return; }
      if (ble_rx_len + body > sizeof(ble_rx_buf)) { ble_rx_len = 0; return; }
      memcpy(ble_rx_buf + ble_rx_len, buf + 1, body);
      ble_rx_len += body;
      if (ble_rx_len >= total) {
        crtpReceiveRaw(ble_rx_buf, total);
        ble_rx_len = 0;
      }
    }
  }
};

class BleServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer *s) {
    ble_connected = true;
    Serial.println(F("[BLE] phone connected"));
  }
  void onDisconnect(BLEServer *s) {
    ble_connected = false;
    ble_rx_len = 0;
    Serial.println(F("[BLE] phone disconnected"));
    s->startAdvertising();              // ready for the next phone
                                        // sẵn sàng cho điện thoại tiếp theo
  }
};

bool bleInit(const char *name) {
  BLEDevice::init(name);
  BLEDevice::setMTU(64);
  BLEServer  *srv = BLEDevice::createServer();
  BLEService *svc = srv->createService(BLE_UUID_SERVICE);
  srv->setCallbacks(new BleServerCb());

  BLECharacteristic *c;
  c = svc->createCharacteristic(BLE_UUID_CRTP,   BLECharacteristic::PROPERTY_WRITE);
  c->setCallbacks(new BleRawCb());
  c = svc->createCharacteristic(BLE_UUID_CRTPUP, BLECharacteristic::PROPERTY_WRITE);
  c->setCallbacks(new BleSegCb());
  bleCrtpDown = svc->createCharacteristic(BLE_UUID_CRTPDOWN, BLECharacteristic::PROPERTY_NOTIFY);
  bleCrtpDown->addDescriptor(new BLE2902());

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_UUID_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("[BLE] advertising as '%s'\n", name);
  return true;
}
#endif

// ---------------------------------------------------------------------------
//  WiFi
// ---------------------------------------------------------------------------
bool wifiApInit() {
  if (!WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_GW, WIFI_AP_MASK)) {
    Serial.println(F("[WIFI] softAPConfig failed — apps expect 192.168.43.42"));
  }
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL, 0, 4)) {
    Serial.println(F("[WIFI] softAP failed"));
    return false;
  }
  Udp.begin(UDP_DEVICE_PORT);
  Serial.printf("[WIFI] SoftAP '%s' at %s, UDP %d\n",
                WIFI_AP_SSID, WiFi.softAPIP().toString().c_str(), UDP_DEVICE_PORT);
  return true;
}

bool wifiStaInit() {
  if (sta_ssid[0] == '\0') {
    Serial.println(F("[WIFI] no station SSID configured — use \"sta <ssid> <pass>\""));
    return false;
  }
  WiFi.begin(sta_ssid, sta_pass);
  Serial.printf("[WIFI] joining '%s'", sta_ssid);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_STA_TIMEOUT_MS) {
    delay(100); Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[WIFI] join timed out"));
    return false;
  }
  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("esp-drone", "udp", UDP_DEVICE_PORT);
  }
  Udp.begin(UDP_DEVICE_PORT);
  Serial.printf("[WIFI] joined '%s' at %s (try %s.local), UDP %d\n",
                sta_ssid, WiFi.localIP().toString().c_str(), MDNS_HOSTNAME, UDP_DEVICE_PORT);
  return true;
}

void udpPoll() {
  int packetSize = Udp.parsePacket();
  if (packetSize <= 0) return;

  int n = Udp.read(crtp_rx_buf, sizeof(crtp_rx_buf));
  if (n < 2) return;

  // Learn the phone address — this is also where replies go (apps bind 2399).
  // Học địa chỉ điện thoại — đây cũng là nơi gửi phản hồi (app bind cổng 2399).
  IPAddress src = Udp.remoteIP();
  if (!app_ip_known || src != app_ip) {
    app_ip = src;
    app_ip_known = true;
    Serial.printf("[CRTP] App at %s\n", app_ip.toString().c_str());
  }
  crtpReceiveFramed(crtp_rx_buf, (size_t)n);
}

// ---------------------------------------------------------------------------
//  Send / poll / init
// ---------------------------------------------------------------------------
// `payload` is a complete CRTP packet, header included, no checksum.
// `payload` là một gói CRTP hoàn chỉnh, có header, không có checksum.
bool linkSend(const uint8_t *payload, size_t len) {
  if (len == 0 || len > CRTP_MAX_SIZE) return false;

#if LINK_USE_BT_SPP
  if (link_mode == LINK_MODE_BT_SPP) {
    if (!btSpp || !btSpp->connected()) return false;
    uint8_t out[CRTP_MAX_SIZE + 1];
    memcpy(out, payload, len);
    out[len] = crtpChecksum(payload, len);
    return btSpp->write(out, len + 1) == (size_t)(len + 1);
  }
#endif

#if LINK_USE_BLE
  if (linkUsesBle(link_mode) && !linkUsesUdp(link_mode)) return bleSend(payload, len);
#endif

  // UDP (including the WiFi+BLE profiles, which carry CRTP over UDP)
  // UDP (gồm cả các hồ sơ WiFi+BLE, vốn chở CRTP qua UDP)
  if (!app_ip_known) return false;
  uint8_t out[CRTP_MAX_SIZE + 1];
  memcpy(out, payload, len);
  out[len] = crtpChecksum(payload, len);
  Udp.beginPacket(app_ip, UDP_APP_PORT);
  Udp.write(out, len + 1);
  return Udp.endPacket() == 1;
}

void linkPoll() {
  if (linkUsesUdp(link_mode)) udpPoll();
#if LINK_USE_BT_SPP
  if (link_mode == LINK_MODE_BT_SPP) btPoll();
#endif
  // BLE RX arrives in callbacks; nothing to poll.
  // BLE RX đến trong các callback; không có gì để poll.
}

void linkInit() {
  link_mode = nvs.getUChar("link", LINK_MODE_WIFI_AP);
  if (link_mode >= LINK_MODE_COUNT) link_mode = LINK_MODE_WIFI_AP;

  bool wifi = linkUsesUdp(link_mode);
  bool ble  = linkUsesBle(link_mode);

  // BLE coexists with WiFi, so the *_BLE profiles just enable both stacks.
  // BLE chung sống được với WiFi, nên các hồ sơ *_BLE chỉ bật cả hai giao thức.
  if (wifi)      WiFi.mode(link_mode == LINK_MODE_WIFI_STA || link_mode == LINK_MODE_WIFI_STA_BLE
                            ? WIFI_STA : WIFI_AP);
  else if (ble)  WiFi.mode(WIFI_OFF);

  bool ok = true;
  switch (link_mode) {
    case LINK_MODE_WIFI_AP:       ok = wifiApInit();  break;
    case LINK_MODE_WIFI_STA:      ok = wifiStaInit(); break;
    case LINK_MODE_WIFI_AP_BLE:   ok = wifiApInit();  break;
    case LINK_MODE_WIFI_STA_BLE:  ok = wifiStaInit(); break;
#if LINK_USE_BT_SPP
    case LINK_MODE_BT_SPP:        ok = btInit(WIFI_AP_SSID); break;
#endif
#if LINK_USE_BLE
    case LINK_MODE_BLE:           ok = bleInit(WIFI_AP_SSID); break;
#endif
    default:
      Serial.printf("[LINK] profile %u not built in — falling back to softAP\n", link_mode);
      link_mode = LINK_MODE_WIFI_AP;
      ok = wifiApInit();
      break;
  }

#if LINK_USE_BLE
  if (ok && ble && link_mode != LINK_MODE_BLE) bleInit(WIFI_AP_SSID);
#endif

  link_ready = ok;
  Serial.printf("[LINK] profile %u %s\n", link_mode, ok ? "up" : "FAILED");
}

// Send a CRTP packet (no checksum) to the phone. linkSend adds whatever
// framing the active transport needs.
// Gửi một gói CRTP (không checksum) tới điện thoại. linkSend thêm phần đóng khung
// mà tầng truyền đang hoạt động cần.
bool crtpSend(const uint8_t *payload, size_t len) {
  return linkSend(payload, len);
}

// Convenience: send human-readable console text (CRTP port 0, channel 0).
// NOTE: the iOS app decodes inbound datagrams as UTF-8 text and would crash on
// raw binary, so console text is the only payload both apps tolerate.
// Tiện ích: gửi văn bản console dạng người đọc được (CRTP port 0, channel 0).
// LƯU Ý: app iOS giải mã gói vào dạng văn bản UTF-8 và sẽ crash với dữ liệu nhị
// phân thô, nên văn bản console là phần dữ liệu duy nhất cả hai app chấp nhận.
void crtpConsole(const char *text) {
  // Never emit an empty datagram: the client's resend-queue matcher indexes
  // into the payload without a bounds check, so a 1-byte packet could throw.
  // Không bao giờ phát một gói rỗng: bộ so khớp hàng đợi gửi lại của client truy
  // cập phần dữ liệu không kiểm tra biên, nên gói 1 byte có thể gây lỗi.
  if (!text || !*text) return;
  uint8_t buf[CRTP_MAX_SIZE];
  size_t n = strlen(text);
  if (n > CRTP_MAX_SIZE - 1) n = CRTP_MAX_SIZE - 1;
  buf[0] = (CRTP_PORT_CONSOLE << 4);   // port 0, channel 0
  memcpy(buf + 1, text, n);
  crtpSend(buf, n + 1);
}

// ---- CommanderPacket (CRTP port 3, channel 0) ------------------------------
// [0] 0x30 header | [1..4] float roll | [5..8] float pitch (already negated
// by the client) | [9..12] float yaw | [13..14] uint16 thrust | [15] checksum
// [0] 0x30 header | [1..4] float roll | [5..8] float pitch (client đã đảo dấu)
// | [9..12] float yaw | [13..14] uint16 thrust | [15] checksum
void crtpHandleCommander(const uint8_t *p, size_t len) {
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

// ---- Flight control (CRTP port 7, channel 0) ------------------------------
// [0] 0x70 | [1] cmd | [2..] args      reply: [0x70][cmd][status u8]
// Every command re-acks on retransmission: the client retries unacknowledged
// packets every 500 ms, and these must be safe to apply twice. The only
// non-idempotent one is TAKEOFF, which simply restarts the ramp.
// Mỗi lệnh đều ack lại khi được gửi lại: client thử lại các gói chưa có ack mỗi
// 500 ms, nên các lệnh này phải an toàn khi áp dụng hai lần. Lệnh duy nhất không
// idempotent là TAKEOFF, vốn chỉ khởi động lại đoạn bám lực đẩy.
void crtpHandleFlightControl(const uint8_t *p, size_t len) {
  if (len < 2) return;
  uint8_t cmd = p[1];
  uint8_t status = 0;                 // 0 = OK, 1 = bad request

  switch (cmd) {
    case FC_CMD_SET_MODE:
      // Modes run 0..4 (MANUAL..RTH), not 0..2 — the old bound silently
      // rejected FOLLOW and RTH, which are the whole point of the nav stack.
      // Các chế độ chạy 0..4 (MANUAL..RTH), không phải 0..2 — biên cũ từng lặng
      // lẽ từ chối FOLLOW và RTH, vốn là mục đích chính của hệ dẫn đường.
      if (len < 3 || p[2] > MODE_RTH) { status = 1; break; }
      // Once the battery is critical the drone is landing. A mode command (the
      // GPS screen resends SET_MODE every tick) must not cancel that.
      // Khi pin đã ở mức nguy kịch, drone sẽ hạ cánh. Một lệnh chế độ (màn hình
      // GPS gửi lại SET_MODE mỗi nhịp) không được hủy việc đó.
      if (armed && vbat_crit) { status = 1; break; }
      flightSetMode(p[2]);
      break;

    case FC_CMD_TAKEOFF: {
      uint16_t thrust = hover_thrust;
      if (len >= 4) memcpy(&thrust, p + 2, 2);
      if (!flightTakeoff(thrust)) status = 1;
      break;
    }

    case FC_CMD_LAND:
      flightLand();
      break;

    case FC_CMD_SET_HOVER:
      if (len < 4) { status = 1; break; }
      { uint16_t t = 0;
        memcpy(&t, p + 2, 2);
        hover_thrust = constrain(t, HOVER_THRUST_MIN, HOVER_THRUST_MAX); }
      break;

    case FC_CMD_EMERGENCY:
      flightEmergency();
      break;

    case FC_CMD_SET_HEADING:
      if (len < 6) { status = 1; break; }
      { float h = 0.0f;
        memcpy(&h, p + 2, 4);
        if (!isfinite(h)) { status = 1; break; }
        hold_heading_deg = fmodf(h, 360.0f);
        if (hold_heading_deg < 0.0f) hold_heading_deg += 360.0f; }
      break;

    case FC_CMD_ARM:
      // Refused unless the motors would start from idle: MANUAL with the
      // phone's thrust at zero, or AUTO before a takeoff. In HOLD/FOLLOW/RTH the
      // motors would jump straight to hover thrust -- use TAKEOFF for that.
      // Từ chối trừ khi động cơ sẽ khởi động từ trạng thái nghỉ: MANUAL với lực
      // đẩy từ điện thoại bằng không, hoặc AUTO trước khi cất cánh. Ở HOLD/FOLLOW/RTH
      // động cơ sẽ nhảy thẳng tới lực đẩy hover -- hãy dùng TAKEOFF cho việc đó.
      if (!armed) {
        bool from_idle = (flight_mode == MODE_MANUAL && cmd_thrust <= 500) ||
                         (flight_mode == MODE_AUTO && auto_state == AUTO_IDLE);
        if (!from_idle || !armChecksOk("CRTP ARM")) { status = 1; break; }
        armed = true;
        pidReset();
      }
      break;

    case FC_CMD_DISARM:
      armed = false;
      motor_test_active = false;   // never leave a bench test running // không bao giờ để thử bàn chạy tiếp
      motorStop();
      pidReset();
      break;

    case FC_CMD_SAVE:
      // NVS writes erase flash, which stalls both cores for milliseconds.
      // Ghi NVS sẽ xóa flash, làm cả hai nhân đứng yên hàng milli giây.
      if (armed) { status = 1; break; }
      cfgSave();
      break;

    case FC_CMD_DEFAULTS:
      flightConfigDefaults();
      break;

    case FC_CMD_LOAD:
      cfgLoad();
      break;

    case FC_CMD_LINK:
      // Never write flash and reboot in the air.
      // Không bao giờ ghi flash và khởi động lại khi đang bay.
      if (armed || len < 3 || p[2] >= LINK_MODE_COUNT) { status = 1; break; }
      nvs.putUChar("link", p[2]);
      pending_link_mode = p[2];
      reboot_at_ms = millis() + 1500;   // time for the reply to get out first
      break;

    case FC_CMD_MOTOR_TEST:
      // Bench motor test — drive m1..m4 directly, PID bypassed. Refused while
      // armed; held only while the GUI keeps re-sending within the watchdog
      // window; p[2]==0 (or EMERGENCY/DISARM/link loss) stops everything.
      // Payload: [2]=enable, [3..4]=m1, [5..6]=m2, [7..8]=m3, [9..10]=m4, µs.
      // Thử động cơ trên bàn — lái m1..m4 trực tiếp, bỏ qua PID. Từ chối khi đang
      // armed; chỉ giữ trong khi GUI tiếp tục gửi lại trong cửa sổ watchdog;
      // p[2]==0 (hoặc EMERGENCY/DISARM/mất liên kết) dừng tất cả.
      // Phần dữ liệu: [2]=enable, [3..4]=m1, [5..6]=m2, [7..8]=m3, [9..10]=m4, µs.
      if (len < 3) { status = 1; break; }
      if (!p[2]) {                     // explicit stop // lệnh dừng rõ ràng
        motor_test_active = false;
        motorStop();
        break;
      }
      if (armed) { status = 1; break; }  // bench only, never while armed
      if (len < 11) { status = 1; break; }
      for (int i = 0; i < 4; i++) {
        uint16_t t = 0;
        memcpy(&t, p + 3 + i * 2, 2);
        motor_test_m[i] = constrain((int)t, 1000, 2000);
      }
      motor_test_active = true;
      motor_test_last_ms = millis();
      break;

    default:
      status = 1;
      break;
  }

  // Two payload bytes after the header, so the client's resend-queue matcher
  // can index it without a bounds check.
  // Hai byte phần dữ liệu sau header, để bộ so khớp hàng đợi gửi lại của client
  // có thể truy cập mà không cần kiểm tra biên.
  uint8_t out[3] = { FC_HDR, cmd, status };
  crtpSend(out, sizeof(out));
}

// ---- Navigation (CRTP port 7, channel 1) ---------------------------------
// The phone computes the target from its own GPS and compass and sends it as
// an offset from the launch point. Everything the drone does with it is an
// estimate — see the NAVIGATION section above.
// Điện thoại tính mục tiêu từ GPS và la bàn của chính nó rồi gửi dạng độ lệch so
// với điểm cất cánh. Mọi thứ drone làm với nó đều là ước lượng — xem phần
// NAVIGATION ở trên.
void crtpHandleNav(const uint8_t *p, size_t len) {
  if (len < 2) return;
  uint8_t cmd = p[1];
  uint8_t status = 0;

  switch (cmd) {
    case NAV_CMD_TARGET:
      // north/east offset of the target from the launch point, in metres.
      // The drone has no compass, so it cannot rotate these into its own
      // frame by itself — NAV_CMD_ALIGN supplied that angle at launch.
      // độ lệch bắc/đông của mục tiêu so với điểm cất cánh, tính bằng mét.
      // Drone không có la bàn nên không thể tự quay các giá trị này vào khung của
      // mình — NAV_CMD_ALIGN cung cấp góc đó tại lúc cất cánh.
      if (len < 10) { status = 1; break; }
      { float n = 0.0f, e = 0.0f;
        memcpy(&n, p + 2, 4);
        memcpy(&e, p + 6, 4);
        if (!isfinite(n) || !isfinite(e)) { status = 1; break; }
        // Launch frame: +x is where the nose pointed at ALIGN, +y is 90 deg to
        // the LEFT of that -- the same frame navUpdate() integrates into, so
        // navSetpoints() can subtract the two directly. Projecting (n, e) onto
        // the unit vectors at bearing alpha and bearing alpha-90:
        //   +x = n*cos(a) + e*sin(a)
        //   +y = n*sin(a) - e*cos(a)
        // The second line used to carry the opposite sign, which put the target
        // on the STARBOARD side and made the drone fly away from any target
        // that was not straight ahead. frame_test.cpp pins this.
        // Khung cất cánh: +x là hướng mũi tại ALIGN, +y lệch 90 độ sang TRÁI --
        // đúng khung mà navUpdate() tích phân vào, nên navSetpoints() có thể trừ
        // hai giá trị trực tiếp. Chiếu (n, e) lên các vectơ đơn vị tại phương
        // hướng alpha và alpha-90:
        //   +x = n*cos(a) + e*sin(a)
        //   +y = n*sin(a) - e*cos(a)
        // Dòng thứ hai trước đây mang dấu ngược, làm mục tiêu nằm về phía MẠN
        // PHẢI và khiến drone bay ra xa mọi mục tiêu không thẳng phía trước.
        // frame_test.cpp cố định điều này.
        const float a = align_alpha_deg * DEG_TO_RAD;
        const float ca = cosf(a), sa = sinf(a);
        tgt_x = n * ca + e * sa;
        tgt_y = n * sa - e * ca;
        tgt_valid = true;
        tgt_last_ms = millis(); }
      break;

    case NAV_CMD_RESET:
      navReset();
      break;

    case NAV_CMD_ALIGN:
      // The compass bearing of the nose at launch. This is the one frame
      // relationship the drone cannot measure — it has no magnetometer — so
      // the phone supplies it and the drone treats this moment as launch:
      // heading_ref_deg is zeroed here and the dead reckoning restarts.
      // Phương hướng la bàn của mũi tại lúc cất cánh. Đây là mối quan hệ khung
      // duy nhất drone không đo được — nó không có từ kế — nên điện thoại cung
      // cấp giá trị và drone xem khoảnh khắc này là lúc cất cánh:
      // heading_ref_deg được đặt về 0 tại đây và tự dẫn đường quán tính khởi động lại.
      if (len < 6) { status = 1; break; }
      { float a = 0.0f;
        memcpy(&a, p + 2, 4);
        if (!isfinite(a)) { status = 1; break; }
        align_alpha_deg = fmodf(a, 360.0f);
        if (align_alpha_deg < 0.0f) align_alpha_deg += 360.0f;
        navReset();
        Serial.printf("[NAV] aligned: nose at compass %.1f deg, position zeroed\n",
                      align_alpha_deg); }
      break;

    case NAV_CMD_ORIGIN:
      // The launch latitude/longitude. Mission items and MAVLink telemetry are
      // WGS-84 degrees, the dead reckoning is a local frame — this origin is
      // the only thing that converts between them (§12). Pair it with ALIGN.
      // Vĩ độ/kinh độ điểm cất cánh. Mục nhiệm vụ và telemetry MAVLink dùng độ
      // WGS-84, tự dẫn đường dùng khung cục bộ — điểm gốc này là thứ duy nhất
      // chuyển đổi giữa hai hệ (§12). Gửi kèm ALIGN.
      if (len < 18) { status = 1; break; }
      { double lat = 0.0, lon = 0.0;
        memcpy(&lat, p + 2, 8);
        memcpy(&lon, p + 10, 8);
        if (!isfinite(lat) || !isfinite(lon) ||
            lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
          status = 1;
          break;
        }
        home_lat = lat;
        home_lon = lon;
        home_valid = true;
        mav_home_dirty = true;   // the GCS home moves too // home của GCS cũng đổi
        Serial.printf("[NAV] origin: lat %.7f lon %.7f\n", home_lat, home_lon); }
      break;

    default:
      status = 1;
      break;
  }

  uint8_t out[3] = { NAV_HDR, cmd, status };
  crtpSend(out, sizeof(out));
}

// =============================================================================
//  ONBOARD MISSION ENGINE  (PROTOCOL.md §12 — AP_Mission port)
//  A stored item list flown in AUTO, transferred with the MAVLink MISSION_*
//  handshake over CRTP port 7 channel 2, and over the MAVLink bridge (§13).
//  ĐỘNG CƠ NHIỆM VỤ TRÊN BO  (PROTOCOL.md §12 — chuyển thể từ AP_Mission)
//  Danh sách mục được lưu, bay ở AUTO, truyền bằng bắt tay MISSION_* của
//  MAVLink qua CRTP cổng 7 kênh 2, và qua cầu nối MAVLink (§13).
// =============================================================================

// Flat-earth conversion at the launch point. Accurate to well beyond the
// flight radius, and the inverse is exact for the same approximation.
// Chuyển đổi đất phẳng tại điểm cất cánh. Chính xác tới xa hơn nhiều bán kính
// bay, và phép nghịch đảo đúng cho chính phép xấp xỉ đó.
void missionToLocal(int32_t lat_e7, int32_t lon_e7, float *x, float *y) {
  const float a = align_alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  double n = ((double)lat_e7 * 1e-7 - home_lat) * 111319.49;
  double e = ((double)lon_e7 * 1e-7 - home_lon) * 111319.49 * cos(home_lat * DEG_TO_RAD);
  // The §6 rotation — the same matrix NAV_CMD_TARGET uses.
  // Phép quay §6 — cùng ma trận mà NAV_CMD_TARGET dùng.
  *x = (float)(n * ca + e * sa);
  *y = (float)(n * sa - e * ca);
}

void localToMission(float x, float y, int32_t *lat_e7, int32_t *lon_e7) {
  const float a = align_alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  // The rotation above is its own inverse.
  // Ma trận ở trên tự nghịch đảo.
  double n = (double)x * ca + (double)y * sa;
  double e = (double)x * sa - (double)y * ca;
  *lat_e7 = (int32_t)llround((home_lat + n / 111319.49) * 1e7);
  *lon_e7 = (int32_t)llround((home_lon + e / (111319.49 * cos(home_lat * DEG_TO_RAD))) * 1e7);
}

int missionIndexForSeq(uint16_t seq) {
  for (uint16_t i = 0; i < mission_count; i++)
    if (mission_items[i].seq == seq) return (int)i;
  return -1;
}

void missionSendCurrent() {
  uint16_t seq = (mission_current >= 0 && mission_current < (int)mission_count)
               ? mission_items[mission_current].seq : 0;
  uint8_t out[4] = { MSN_HDR, MSN_CMD_CURRENT, 0, 0 };
  memcpy(out + 2, &seq, 2);
  crtpSend(out, sizeof(out));
  mav_mission_current_dirty = true;   // the GCS copy follows // bản của GCS cũng theo
}

void missionClear() {
  mission_count = 0;
  mission_current = -1;
  mission_active = false;
  memset(jump_exec_count, 0, sizeof(jump_exec_count));
  mav_mission_current_dirty = true;
}

// Validate a staged mission before it goes live — AP_Mission's temp-write then
// finalize. Every DO_JUMP must target an existing seq that is not itself.
// Kiểm tra nhiệm vụ trong vùng đệm trước khi kích hoạt — kiểu ghi tạm rồi hoàn
// tất của AP_Mission. Mọi DO_JUMP phải trỏ tới một seq có thật, không phải chính nó.
int missionValidate(const MissionItem *items, uint16_t count) {
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

// Start a staged upload of `count` items (0 = clear the mission).
// Bắt đầu tải `count` mục vào vùng đệm (0 = xóa nhiệm vụ).
int missionUploadBegin(uint16_t count) {
  if (count == 0) {
    missionClear();
    return MAV_MISSION_ACCEPTED;
  }
  if (count > MISSION_MAX_ITEMS) return MAV_MISSION_NO_SPACE;
  mission_stage_count = count;
  mission_stage_seq = 0;
  return MAV_MISSION_ACCEPTED;
}

// Stage one item. *done flips when the last item has just been stored and the
// staged mission committed. A retransmitted last item (the client resends until
// it sees the ACK) is acknowledged again instead of erroring — §12 idempotency.
// Đưa một mục vào vùng đệm. *done bật khi mục cuối vừa được lưu và nhiệm vụ được
// xác nhận. Mục cuối gửi lại (client gửi lại tới khi thấy ACK) được ack lần nữa
// thay vì báo lỗi — tính idempotent của §12.
int missionUploadItem(const MissionItem *it, bool *done) {
  *done = false;
  if (mission_stage_count == 0) {
    // Retransmit of the committed mission's last item?
    // Có phải mục cuối của nhiệm vụ đã xác nhận được gửi lại không?
    if (mission_count > 0 && (int)it->seq + 1 == (int)mission_count &&
        memcmp(it, &mission_items[it->seq], sizeof(MissionItem)) == 0) {
      *done = true;
      return MAV_MISSION_ACCEPTED;
    }
    return MAV_MISSION_ERROR;
  }
  if (it->seq != mission_stage_seq) return MAV_MISSION_INVALID;   // out of order
  mission_stage[mission_stage_seq] = *it;
  mission_stage_seq++;
  if (mission_stage_seq < mission_stage_count) return MAV_MISSION_ACCEPTED;

  int res = missionValidate(mission_stage, mission_stage_count);
  if (res == MAV_MISSION_ACCEPTED) {
    memcpy(mission_items, mission_stage, sizeof(MissionItem) * mission_stage_count);
    mission_count = mission_stage_count;
    mission_current = 0;
    memset(jump_exec_count, 0, sizeof(jump_exec_count));
    mav_mission_current_dirty = true;
    *done = true;
    Serial.printf("[MSN] mission uploaded: %u items\n", mission_count);
  }
  mission_stage_count = 0;
  return res;
}

// Consume the items that complete without flying — DO_JUMP repeat accounting
// and the takeoff item the AUTO ramp already flew. Returns false on a broken
// jump chain (a jump to itself; missionValidate() rejects those at upload —
// belt and braces).
// Tiêu các mục hoàn thành mà không cần bay — đếm lần lặp của DO_JUMP và mục
// cất cánh mà đoạn bám AUTO đã bay qua. Trả về false khi chuỗi nhảy hỏng (tự
// nhảy; missionValidate() đã từ chối khi tải lên — an toàn hai lớp).
bool missionAdvanceInstant() {
  for (int guard = 0; guard < MISSION_MAX_ITEMS; guard++) {
    if (mission_current < 0 || mission_current >= (int)mission_count) return true;
    MissionItem *it = &mission_items[mission_current];
    if (it->command == MAV_CMD_DO_JUMP) {
      int target = missionIndexForSeq((uint16_t)it->p1);
      if (target < 0 || target == mission_current) return false;
      uint16_t times = (uint16_t)it->p2;    // p2 <= 0 = forever // p2 <= 0 = vĩnh viễn
      if (it->p2 > 0.0f && jump_exec_count[mission_current] >= times) {
        jump_exec_count[mission_current] = 0;
        mission_current++;                  // fall through to the next item
        continue;
      }
      jump_exec_count[mission_current]++;
      mission_current = (int16_t)target;
      missionSendCurrent();
      continue;
    }
    if (it->command == MAV_CMD_NAV_TAKEOFF &&
        (auto_state == AUTO_HOLD || auto_state == AUTO_MISSION)) {
      mission_current++;                    // the ramp has flown already
      missionSendCurrent();
      continue;
    }
    break;
  }
  return true;
}

// MISSION_START — arm (through the normal checks) + AUTO + run from the current
// item (PROTOCOL.md §12). A retransmitted start is a no-op ack, like TAKEOFF.
// MISSION_START — kích hoạt (qua các kiểm tra bình thường) + AUTO + chạy từ mục
// hiện tại (PROTOCOL.md §12). Lệnh gửi lại chỉ là ack không-thực-hiện, như TAKEOFF.
bool missionStart() {
  if (mission_count == 0) return false;
  if (mission_active && armed && flight_mode == MODE_AUTO) return true;   // resend
  if (mission_current < 0 || mission_current >= (int)mission_count) mission_current = 0;
  memset(jump_exec_count, 0, sizeof(jump_exec_count));
  if (!armed) {
    if (!armChecksOk("MISSION_START")) return false;
    hold_heading_deg = heading_deg;
    navReset();                            // the launch point // điểm cất cánh
    armed = true;
    pidReset();
    flight_mode = MODE_AUTO;
    auto_state = AUTO_TAKEOFF;
    auto_phase_start_ms = millis();
  } else {
    // Never cancel a landing; otherwise take the flight over.
    // Không bao giờ hủy việc hạ cánh; còn lại thì tiếp quản chuyến bay.
    if (flight_mode == MODE_AUTO && auto_state == AUTO_LAND) return false;
    flight_mode = MODE_AUTO;
    if (auto_state != AUTO_TAKEOFF) {
      auto_state = AUTO_HOLD;              // → AUTO_MISSION on the next tick
      auto_phase_start_ms = millis();
    }
  }
  mission_active = true;
  missionSendCurrent();
  Serial.printf("[MSN] mission start: %u items from seq %u\n", mission_count,
                mission_items[mission_current].seq);
  return true;
}

void missionStop() {
  if (mission_active) Serial.println(F("[MSN] mission stopped"));
  mission_active = false;
  mav_mission_current_dirty = true;
}

// One runner tick while AUTO flies a mission (auto_state == AUTO_MISSION).
// ArduPilot semantics (mode_auto.cpp): walk the items, steer at waypoints,
// land when the list ends. The thrust is the AUTO hold thrust — this board
// holds a thrust level, not a height (§5); item alt is reported, never flown.
// Một nhịp runner khi AUTO bay nhiệm vụ (auto_state == AUTO_MISSION).
// Ngữ nghĩa ArduPilot (mode_auto.cpp): duyệt mục, lái về các waypoint, hạ cánh
// khi hết danh sách. Lực đẩy là lực giữ của AUTO — bo này giữ mức lực đẩy, không
// giữ độ cao (§5); alt của mục chỉ để báo, không bao giờ bay theo.
void missionTick(uint32_t now) {
  if (!mission_active) return;
  if (!missionAdvanceInstant() || mission_current >= (int)mission_count) {
    // Ran off the end (or a broken jump chain): land where we are —
    // ArduPilot's MIS_DONE_BEHAVE = 0 default.
    // Chạy hết danh sách (hoặc chuỗi nhảy hỏng): hạ cánh tại chỗ —
    // mặc định MIS_DONE_BEHAVE = 0 của ArduPilot.
    mission_active = false;
    mav_mission_current_dirty = true;
    flightLand();
    return;
  }
  MissionItem *it = &mission_items[mission_current];
  switch (it->command) {
    case MAV_CMD_NAV_WAYPOINT: {
      float x = 0.0f, y = 0.0f;
      missionToLocal(it->lat_e7, it->lon_e7, &x, &y);
      tgt_x = x;
      tgt_y = y;
      tgt_valid = true;
      tgt_last_ms = now;
      float rad = it->p2 > MISSION_RADIUS_MIN_M ? it->p2 : MISSION_STOP_RADIUS_M;
      float dx = tgt_x - pos_x, dy = tgt_y - pos_y;
      if (dx * dx + dy * dy <= rad * rad) {
        mission_current++;                  // arrived — next item // đến nơi — mục kế
        missionSendCurrent();
      }
      break;
    }
    case MAV_CMD_NAV_RTL:
      // Chase the launch point, land on arrival.
      // Bám điểm cất cánh, hạ cánh khi tới nơi.
      tgt_x = 0.0f;
      tgt_y = 0.0f;
      tgt_valid = true;
      tgt_last_ms = now;
      if (pos_x * pos_x + pos_y * pos_y <=
          MISSION_STOP_RADIUS_M * MISSION_STOP_RADIUS_M) {
        mission_active = false;
        mav_mission_current_dirty = true;
        flightLand();
      }
      break;
    case MAV_CMD_NAV_LAND:
      mission_active = false;
      mav_mission_current_dirty = true;
      flightLand();
      break;
    default:
      mission_current++;                    // stored but unflown (§12) // lưu nhưng không bay
      missionSendCurrent();
      break;
  }
}

// The mission and the home point persist in NVS with SAVE / LOAD (§5, disarmed
// only) and survive a reboot.
// Nhiệm vụ và điểm home nằm trong NVS qua SAVE / LOAD (§5, chỉ khi đã disarm)
// và sống sót qua khởi động lại.
void missionSave() {
  struct { double lat, lon; uint8_t valid; } home = { home_lat, home_lon, home_valid };
  nvs.putBytes("mhome", &home, sizeof(home));
  nvs.putUShort("mcount", mission_count);
  if (mission_count) nvs.putBytes("msn", mission_items, sizeof(MissionItem) * mission_count);
  else nvs.remove("msn");
}

void missionLoad() {
  struct { double lat, lon; uint8_t valid; } home;
  if (nvs.getBytes("mhome", &home, sizeof(home)) == sizeof(home)) {
    home_lat = home.lat;
    home_lon = home.lon;
    home_valid = home.valid != 0;
    mav_home_dirty = true;
  }
  uint16_t n = nvs.getUShort("mcount", 0);
  if (n > MISSION_MAX_ITEMS) n = MISSION_MAX_ITEMS;
  if (n && nvs.getBytes("msn", mission_items, sizeof(MissionItem) * n) ==
           sizeof(MissionItem) * n) {
    mission_count = n;
    mission_current = 0;
  } else {
    mission_count = 0;
    mission_current = -1;
  }
  mission_active = false;
  memset(jump_exec_count, 0, sizeof(jump_exec_count));
  mav_mission_current_dirty = true;
}

// Port 7, channel 2 — the MISSION_* handshake (PROTOCOL.md §12). The client
// re-sends its last message every 500 ms (§2), so every path here is
// idempotent: COUNT restarts the upload, a repeated ITEM overwrites that seq.
// Cổng 7, kênh 2 — bắt tay MISSION_* (PROTOCOL.md §12). Client gửi lại thông
// điệp cuối mỗi 500 ms (§2), nên mọi nhánh ở đây đều idempotent: COUNT tải lại
// từ đầu, ITEM lặp ghi đè seq đó.
void crtpHandleMission(const uint8_t *p, size_t len) {
  if (len < 2) return;
  uint8_t cmd = p[1];

  switch (cmd) {
    case MSN_CMD_COUNT: {
      if (len < 4) return;
      uint16_t count = 0;
      memcpy(&count, p + 2, 2);
      int res = missionUploadBegin(count);
      if (res == MAV_MISSION_ACCEPTED && count > 0) {
        uint8_t out[4] = { MSN_HDR, MSN_CMD_REQUEST, 0, 0 };
        memcpy(out + 2, &mission_stage_seq, 2);
        crtpSend(out, sizeof(out));
      } else {
        // ACCEPTED with count 0 = the mission is cleared.
        // ACCEPTED với count 0 = nhiệm vụ đã bị xóa.
        uint8_t out[3] = { MSN_HDR, MSN_CMD_ACK, (uint8_t)res };
        crtpSend(out, sizeof(out));
      }
      break;
    }

    case MSN_CMD_ITEM: {
      // seq u16, command u16, p1 f32, p2 f32, lat_e7 i32, lon_e7 i32, alt f32
      if (len < 26) return;
      MissionItem it;
      memcpy(&it.seq,     p + 2,  2);
      memcpy(&it.command, p + 4,  2);
      memcpy(&it.p1,      p + 6,  4);
      memcpy(&it.p2,      p + 10, 4);
      it.p3 = it.p4 = 0.0f;   // the §12 wire note — 30-byte ceiling // ghi chú §12
      memcpy(&it.lat_e7,  p + 14, 4);
      memcpy(&it.lon_e7,  p + 18, 4);
      memcpy(&it.alt,     p + 22, 4);
      bool done = false;
      int res = missionUploadItem(&it, &done);
      if (res != MAV_MISSION_ACCEPTED) {
        uint8_t out[3] = { MSN_HDR, MSN_CMD_ACK, (uint8_t)res };
        crtpSend(out, sizeof(out));
      } else if (done) {
        uint8_t out[3] = { MSN_HDR, MSN_CMD_ACK, MAV_MISSION_ACCEPTED };
        crtpSend(out, sizeof(out));
      } else {
        uint8_t out[4] = { MSN_HDR, MSN_CMD_REQUEST, 0, 0 };
        memcpy(out + 2, &mission_stage_seq, 2);
        crtpSend(out, sizeof(out));
      }
      break;
    }

    case MSN_CMD_REQUEST_LIST: {
      uint8_t out[4] = { MSN_HDR, MSN_CMD_COUNT, 0, 0 };
      memcpy(out + 2, &mission_count, 2);
      crtpSend(out, sizeof(out));
      break;
    }

    case MSN_CMD_REQUEST: {
      if (len < 4) return;
      uint16_t seq = 0;
      memcpy(&seq, p + 2, 2);
      int idx = missionIndexForSeq(seq);
      if (idx < 0) {
        uint8_t out[3] = { MSN_HDR, MSN_CMD_ACK, MAV_MISSION_INVALID };
        crtpSend(out, sizeof(out));
        return;
      }
      uint8_t out[26];
      const MissionItem *it = &mission_items[idx];
      out[0] = MSN_HDR;
      out[1] = MSN_CMD_ITEM;
      memcpy(out + 2,  &it->seq,     2);
      memcpy(out + 4,  &it->command, 2);
      memcpy(out + 6,  &it->p1,      4);
      memcpy(out + 10, &it->p2,      4);
      memcpy(out + 14, &it->lat_e7,  4);
      memcpy(out + 18, &it->lon_e7,  4);
      memcpy(out + 22, &it->alt,     4);
      crtpSend(out, sizeof(out));
      break;
    }

    case MSN_CMD_SET_CURRENT: {
      uint8_t status = 0;
      if (len < 4) {
        status = 1;
      } else {
        uint16_t seq = 0;
        memcpy(&seq, p + 2, 2);
        int idx = missionIndexForSeq(seq);
        if (idx < 0) {
          status = 1;
        } else {
          mission_current = (int16_t)idx;
          memset(jump_exec_count, 0, sizeof(jump_exec_count));
          missionSendCurrent();
        }
      }
      uint8_t out[3] = { MSN_HDR, MSN_CMD_SET_CURRENT, status };
      crtpSend(out, sizeof(out));
      break;
    }

    case MSN_CMD_START: {
      uint8_t status = missionStart() ? 0 : 1;
      uint8_t out[3] = { MSN_HDR, MSN_CMD_START, status };
      crtpSend(out, sizeof(out));
      break;
    }

    default: {
      uint8_t out[3] = { MSN_HDR, cmd, 1 };
      crtpSend(out, sizeof(out));
      break;
    }
  }
}

// =============================================================================
//  MAVLINK BRIDGE  (PROTOCOL.md §13 — ArduPilot GCS interop)
//  MAVLink v2 on UDP 14550 — a second transport for the same flight commands
//  (§5) and the same mission store (§12). sysid 1 / compid 1,
//  MAV_TYPE_QUADROTOR (2), MAV_AUTOPILOT_GENERIC (0): claiming ARDUPILOTMEGA
//  would make Mission Planner expect the full ArduPilot parameter set.
//  CẦU NỐI MAVLINK  (PROTOCOL.md §13 — tương thích GCS ArduPilot)
//  MAVLink v2 trên UDP 14550 — transport thứ hai cho cùng lệnh bay (§5) và cùng
//  kho nhiệm vụ (§12). sysid 1 / compid 1, MAV_TYPE_QUADROTOR (2),
//  MAV_AUTOPILOT_GENERIC (0): khai báo ARDUPILOTMEGA sẽ khiến Mission Planner
//  đòi đủ bộ tham số ArduPilot.
// =============================================================================

// crc_extra per message id (PROTOCOL.md §13; generated from pymavlink's common
// dialect and pinned by test/mavlink_test.cpp).
// crc_extra theo id thông điệp (PROTOCOL.md §13; sinh từ dialect common của
// pymavlink và được test/mavlink_test.cpp cố định).
const uint32_t MAV_CRC_IDS[]    = { 0, 1, 30, 33, 39, 40, 41, 42, 43,
                                    44, 45, 47, 51, 73, 76, 77, 242, 253 };
const uint8_t  MAV_CRC_EXTRAS[] = { 50, 124, 39, 104, 254, 230, 28, 28, 132,
                                    221, 232, 153, 196, 38, 152, 143, 104, 83 };

uint8_t mavCrcExtra(uint32_t id) {
  for (size_t i = 0; i < sizeof(MAV_CRC_IDS) / sizeof(MAV_CRC_IDS[0]); i++)
    if (MAV_CRC_IDS[i] == id) return MAV_CRC_EXTRAS[i];
  return 0;
}

// CRC-16/MCRF4XX, MAVLink's accumulator (seed 0xFFFF).
uint16_t mavCrcAccumulate(uint8_t data, uint16_t crc) {
  uint8_t tmp = data ^ (uint8_t)(crc & 0xff);
  tmp ^= (uint8_t)(tmp << 4);
  return (uint16_t)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
}

// Frame a v2 packet. Trailing zeros are trimmed from the payload and the length
// field (MAVLink 2 zero-truncation); the CRC covers every byte from the length
// byte on — never the STX — then one final accumulate of crc_extra.
// Đóng gói gói v2. Các byte 0 cuối bị lược khỏi payload và trường length (cắt
// số 0 của MAVLink 2); CRC phủ mọi byte từ byte length trở đi — không bao giờ
// STX — rồi cộng dồn thêm một lần crc_extra cuối.
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

void mavSendTo(const IPAddress &ip, uint16_t port, uint32_t msgid,
               const uint8_t *payload, size_t len) {
  uint8_t buf[128];
  if (10 + len + 2 > sizeof(buf)) return;
  size_t n = mavFrame(msgid, payload, len, buf);
  MavUdp.beginPacket(ip, port);
  MavUdp.write(buf, n);
  MavUdp.endPacket();
}

void mavSend(uint32_t msgid, const uint8_t *payload, size_t len) {
  if (!mav_gcs_known) return;
  mavSendTo(mav_gcs_ip, mav_gcs_port, msgid, payload, len);
}

// Compass bearing of the nose now: ALIGN gave the bearing at launch, the gyro
// gives the rotation since. Without a mag this drifts — same as the heading
// hold itself (§5).
// Phương la bàn của mũi hiện tại: ALIGN cho hướng tại cất cánh, con quay cho
// lượng quay từ đó. Không có từ kế nên giá trị trôi — giống bản thân việc giữ
// hướng (§5).
float mavCompassDeg() {
  float c = align_alpha_deg + (heading_deg - heading_ref_deg);
  c = fmodf(c, 360.0f);
  if (c < 0.0f) c += 360.0f;
  return c;
}

void mavSendHeartbeat() {
  uint8_t p[9] = {0};   // custom_mode unused — the mode lives in base_mode
  uint8_t base = 0x01;                          // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
  if (armed) base |= 0x80;                      // MAV_MODE_FLAG_SAFETY_ARMED
  if (flight_mode == MODE_AUTO) base |= 0x04;   // MAV_MODE_FLAG_AUTO_ENABLED
  else if (flight_mode != MODE_MANUAL) base |= 0x08;  // GUIDED_ENABLED
  p[4] = 2;                                     // MAV_TYPE_QUADROTOR
  p[5] = 0;                                     // MAV_AUTOPILOT_GENERIC
  p[6] = base;
  p[7] = armed ? 4 : 3;                         // MAV_STATE_ACTIVE / STANDBY
  p[8] = 3;                                     // MAVLink protocol version
  mavSend(0, p, sizeof(p));
}

void mavSendSysStatus() {
  uint8_t p[19] = {0};
  // 1S — the cell IS the battery. 3.3 V empty, 4.2 V full.
  // 1S — pin chính là ô đó. 3.3 V hết, 4.2 V đầy.
  uint16_t mv = (uint16_t)(vbat * 1000.0f);
  int8_t remaining =
      (int8_t)constrain((vbat - 3.3f) / 0.9f * 100.0f, 0.0f, 100.0f);
  memcpy(p + 14, &mv, 2);
  p[16] = 0xFF;                                 // current_battery i16 = -1 (unknown)
  p[17] = 0xFF;
  p[18] = (uint8_t)remaining;
  mavSend(1, p, sizeof(p));
}

void mavSendAttitude(uint32_t now) {
  uint8_t p[28] = {0};
  uint32_t t = now;
  // MAVLink attitude is NED — roll positive right-wing-down. This firmware
  // calls left-wing-down +roll, so roll flips sign; pitch (nose up) and yaw
  // carry straight over.
  // Tư thế MAVLink theo NED — roll dương khi cánh phải hạ. Firmware này gọi
  // roll dương khi cánh TRÁI hạ, nên roll đảo dấu; pitch (mũi ngẩng) và yaw
  // giữ nguyên.
  float roll  = -angle_roll_deg  * DEG_TO_RAD;
  float pitch =  angle_pitch_deg * DEG_TO_RAD;
  float yaw   =  mavCompassDeg() * DEG_TO_RAD;
  memcpy(p,      &t,     4);
  memcpy(p + 4,  &roll,  4);
  memcpy(p + 8,  &pitch, 4);
  memcpy(p + 12, &yaw,   4);
  mavSend(30, p, sizeof(p));
}

void mavSendGlobalPosition(uint32_t now) {
  uint8_t p[28] = {0};
  int32_t lat = 0, lon = 0;
  if (home_valid) localToMission(pos_x, pos_y, &lat, &lon);
  int32_t alt = 0, rel = 0;                     // no altitude sensor (§5)
  // vel_* are launch-frame; the same rotation turns them into north/east.
  // vel_* theo khung cất cánh; phép quay tương tự đưa chúng sang bắc/đông.
  const float a = align_alpha_deg * DEG_TO_RAD;
  const float ca = cosf(a), sa = sinf(a);
  int16_t vx = (int16_t)(( vel_x * ca + vel_y * sa) * 100.0f);   // cm/s north
  int16_t vy = (int16_t)(( vel_x * sa - vel_y * ca) * 100.0f);   // cm/s east
  int16_t vz = 0;
  uint16_t hdg = (uint16_t)(mavCompassDeg() * 100.0f);            // cdeg, 0..35999
  uint32_t t = now;
  memcpy(p,      &t,   4);
  memcpy(p + 4,  &lat, 4);
  memcpy(p + 8,  &lon, 4);
  memcpy(p + 12, &alt, 4);
  memcpy(p + 16, &rel, 4);
  memcpy(p + 20, &vx,  2);
  memcpy(p + 22, &vy,  2);
  memcpy(p + 24, &vz,  2);
  memcpy(p + 26, &hdg, 2);
  mavSend(33, p, sizeof(p));
}

void mavSendHomePosition() {
  uint8_t p[60] = {0};
  int32_t lat = (int32_t)llround(home_lat * 1e7);
  int32_t lon = (int32_t)llround(home_lon * 1e7);
  int32_t alt = 0;
  memcpy(p,     &lat, 4);
  memcpy(p + 4, &lon, 4);
  memcpy(p + 8, &alt, 4);
  // x/y/z, q[4], approach xyz: not modelled — zeros.
  // x/y/z, q[4], approach xyz: không mô hình hóa — để 0.
  mavSend(242, p, sizeof(p));
  mav_home_dirty = false;
}

void mavSendStatusText(uint8_t severity, const char *text) {
  uint8_t p[51] = {0};
  p[0] = severity;                              // MAV_SEVERITY
  strncpy((char *)p + 1, text, 49);
  mavSend(253, p, sizeof(p));
}

void mavSendCommandAck(uint16_t command, uint8_t result) {
  uint8_t p[4] = {0};
  memcpy(p, &command, 2);
  p[2] = result;                                // MAV_RESULT
  mavSend(77, p, sizeof(p));
}

void mavSendMissionCount(uint16_t count) {
  uint8_t p[5] = {0};
  memcpy(p, &count, 2);
  p[4] = 0;                                     // mission_type: MISSION
  mavSend(44, p, sizeof(p));
}

void mavSendMissionRequest(uint16_t seq) {
  uint8_t p[5] = {0};
  memcpy(p, &seq, 2);
  p[4] = 0;
  mavSend(40, p, sizeof(p));                    // MISSION_REQUEST
}

void mavSendMissionItemInt(const MissionItem *it) {
  uint8_t p[38] = {0};
  memcpy(p,       &it->p1,      4);
  memcpy(p + 4,   &it->p2,      4);
  memcpy(p + 8,   &it->p3,      4);
  memcpy(p + 12,  &it->p4,      4);
  memcpy(p + 16,  &it->lat_e7,  4);
  memcpy(p + 20,  &it->lon_e7,  4);
  memcpy(p + 24,  &it->alt,     4);
  memcpy(p + 28,  &it->seq,     2);
  memcpy(p + 30,  &it->command, 2);
  p[37] = 0;                                    // mission_type ext
  mavSend(73, p, sizeof(p));
}

void mavSendMissionAck(uint8_t type) {
  uint8_t p[4] = {0};
  p[2] = type;                                  // MAV_MISSION_RESULT
  mavSend(47, p, sizeof(p));
}

void mavSendMissionCurrent() {
  uint8_t p[6] = {0};
  uint16_t seq = (mission_current >= 0 && mission_current < (int)mission_count)
               ? mission_items[mission_current].seq : 0;
  uint16_t total = mission_count;
  memcpy(p, &seq, 2);
  memcpy(p + 2, &total, 2);                     // ext: total
  mavSend(42, p, sizeof(p));
  mav_mission_current_dirty = false;
}

// COMMAND_LONG (§13) — the same flight commands the CRTP port-7 channel
// carries (§5), plus MISSION_START and GET_HOME_POSITION.
// COMMAND_LONG (§13) — cùng các lệnh bay mà kênh CRTP cổng 7 mang (§5), cộng
// thêm MISSION_START và GET_HOME_POSITION.
void mavHandleCommand(uint16_t command, float param1, float param2,
                      float param5, float param6, float param7) {
  bool ok = true;
  bool supported = true;
  switch (command) {
    case 400:   // COMPONENT_ARM_DISARM — the CRTP ARM/DISARM rules (§5)
      if (param1 > 0.5f) {
        if (!armed) {
          bool from_idle = (flight_mode == MODE_MANUAL && cmd_thrust <= 500) ||
                           (flight_mode == MODE_AUTO && auto_state == AUTO_IDLE);
          if (!from_idle || !armChecksOk("MAV ARM")) {
            ok = false;
            mavSendStatusText(4, "arm refused");
          } else {
            armed = true;
            pidReset();
          }
        }
      } else {
        armed = false;
        mission_active = false;
        motor_test_active = false;
        motorStop();
        pidReset();
      }
      break;

    case 22:    // NAV_TAKEOFF — the AUTO takeoff ramp // đoạn bám cất cánh AUTO
      if (!flightTakeoff(hover_thrust)) ok = false;
      break;

    case 21:    // NAV_LAND
      mission_active = false;
      flightLand();
      break;

    case 20:    // NAV_RETURN_TO_LAUNCH
      mission_active = false;
      flightSetMode(MODE_RTH);
      break;

    case 300:   // MISSION_START — arm + AUTO + fly the mission (§12)
      if (!missionStart()) {
        ok = false;
        mavSendStatusText(4, "mission start refused");
      }
      break;

    case 176:   // DO_SET_MODE — param2 = custom mode (our 0..4) // param2 = chế độ
      { uint8_t mode = (uint8_t)param2;
        if (mode > MODE_RTH || (armed && vbat_crit)) ok = false;
        else flightSetMode(mode); }
      break;

    case 410:   // GET_HOME_POSITION — HOME_POSITION on request (§13)
      if (home_valid) mavSendHomePosition();
      else ok = false;
      break;

    default:
      supported = false;
      break;
  }
  // MAV_RESULT: 0 accepted, 2 denied, 3 unsupported.
  mavSendCommandAck(command, supported ? (ok ? 0 : 2) : 3);
  (void)param1; (void)param2; (void)param5; (void)param6; (void)param7;
}

// The §12 store over MAVLink — identical handshake, _INT packing on the wire
// (MISSION_ITEM floats accepted too and converted).
// Kho §12 qua MAVLink — cùng bắt tay, đóng gói _INT trên dây (MISSION_ITEM dạng
// float cũng được chấp nhận và chuyển đổi).
void mavHandleMission(uint32_t msgid, const uint8_t *p, size_t len) {
  switch (msgid) {
    case 44: {  // MISSION_COUNT
      if (len < 4) return;
      uint16_t count = 0;
      memcpy(&count, p, 2);
      int res = missionUploadBegin(count);
      if (res == MAV_MISSION_ACCEPTED && count > 0) mavSendMissionRequest(0);
      else mavSendMissionAck((uint8_t)res);
      break;
    }

    case 39:    // MISSION_ITEM (float lat/lon)
    case 73: {  // MISSION_ITEM_INT (lat/lon x 1e7)
      MissionItem it;
      memset(&it, 0, sizeof(it));
      memcpy(&it.p1,     p,      4);
      memcpy(&it.p2,     p + 4,  4);
      memcpy(&it.p3,     p + 8,  4);
      memcpy(&it.p4,     p + 12, 4);
      if (msgid == 73) {
        memcpy(&it.lat_e7, p + 16, 4);
        memcpy(&it.lon_e7, p + 20, 4);
      } else {
        float latf = 0.0f, lonf = 0.0f;
        memcpy(&latf, p + 16, 4);
        memcpy(&lonf, p + 20, 4);
        it.lat_e7 = (int32_t)llround((double)latf * 1e7);
        it.lon_e7 = (int32_t)llround((double)lonf * 1e7);
      }
      memcpy(&it.alt,     p + 24, 4);
      memcpy(&it.seq,     p + 28, 2);
      memcpy(&it.command, p + 30, 2);
      bool done = false;
      int res = missionUploadItem(&it, &done);
      if (res != MAV_MISSION_ACCEPTED) mavSendMissionAck((uint8_t)res);
      else if (done) mavSendMissionAck(MAV_MISSION_ACCEPTED);
      else mavSendMissionRequest(mission_stage_seq);
      break;
    }

    case 40:    // MISSION_REQUEST
    case 51: {  // MISSION_REQUEST_INT — both answered with ITEM_INT
      if (len < 2) return;
      uint16_t seq = 0;
      memcpy(&seq, p, 2);
      int idx = missionIndexForSeq(seq);
      if (idx < 0) {
        mavSendMissionAck(MAV_MISSION_INVALID);
        return;
      }
      mavSendMissionItemInt(&mission_items[idx]);
      break;
    }

    case 43:    // MISSION_REQUEST_LIST
      mavSendMissionCount(mission_count);
      break;

    case 41: {  // MISSION_SET_CURRENT
      if (len < 2) return;
      uint16_t seq = 0;
      memcpy(&seq, p, 2);
      int idx = missionIndexForSeq(seq);
      if (idx < 0) {
        mavSendMissionAck(MAV_MISSION_INVALID);
        return;
      }
      mission_current = (int16_t)idx;
      memset(jump_exec_count, 0, sizeof(jump_exec_count));
      missionSendCurrent();
      break;
    }

    case 45:    // MISSION_CLEAR_ALL
      missionClear();
      mavSendMissionAck(MAV_MISSION_ACCEPTED);
      break;

    default:
      break;
  }
}

// Parse one datagram — the GCS sends one frame per packet. v1 (0xFE) and v2
// (0xFD) both accepted; v2 signed frames are ignored. Returns true when a
// valid frame has been handled.
// Parse một datagram — GCS gửi mỗi gói một frame. Chấp nhận cả v1 (0xFE) lẫn
// v2 (0xFD); frame v2 có chữ ký bị bỏ qua. Trả về true khi đã xử lý một frame hợp lệ.
bool mavHandle(const uint8_t *b, size_t n) {
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

  const uint8_t *p = b + payload_off;
  switch (msgid) {
    case 0:     // HEARTBEAT — presence only // chỉ để biết còn sống
      break;

    case 76: {  // COMMAND_LONG
      if (len < 33) break;
      float param1 = 0.0f, param2 = 0.0f, param5 = 0.0f, param6 = 0.0f, param7 = 0.0f;
      uint16_t command = 0;
      memcpy(&param1, p,      4);
      memcpy(&param2, p + 4,  4);
      memcpy(&param5, p + 16, 4);
      memcpy(&param6, p + 20, 4);
      memcpy(&param7, p + 24, 4);
      memcpy(&command, p + 28, 2);
      mavHandleCommand(command, param1, param2, param5, param6, param7);
      break;
    }

    case 44: case 39: case 73: case 40: case 51: case 43: case 41: case 45:
      mavHandleMission(msgid, p, len);
      break;

    default:
      break;                                    // known but unused — ignored
  }
  return true;
}

// Called every loop. Drain the MAVLink socket, then keep the GCS fed:
// 2 Hz heartbeat + system status, 5 Hz attitude + position, and the event-
// driven HOME_POSITION / MISSION_CURRENT.
// Gọi mỗi vòng lặp. Rút socket MAVLink, rồi cấp liệu cho GCS: heartbeat +
// system status 2 Hz, attitude + vị trí 5 Hz, và HOME_POSITION / MISSION_CURRENT
// theo sự kiện.
void mavlinkPoll() {
  int packetSize = MavUdp.parsePacket();
  while (packetSize > 0) {
    uint8_t buf[192];
    int n = MavUdp.read(buf, sizeof(buf));
    if (n > 0) {
      if (!mav_gcs_known || MavUdp.remoteIP() != mav_gcs_ip) {
        // Learn the GCS the way the CRTP side learns the app — last sender wins.
        // Học GCS như phía CRTP học app — người gửi cuối thắng.
        mav_gcs_ip = MavUdp.remoteIP();
        mav_gcs_port = MavUdp.remotePort();
        mav_gcs_known = true;
        mav_home_dirty = true;
        mav_mission_current_dirty = true;
        Serial.printf("[MAV] GCS %s:%u\n", mav_gcs_ip.toString().c_str(), mav_gcs_port);
      }
      if (mavHandle(buf, (size_t)n)) {
        // A MAVLink packet proves the GCS link alive exactly like a CRTP
        // packet (§5) — a GCS flies a mission without ever speaking CRTP.
        // Gói MAVLink chứng minh liên kết GCS sống y như gói CRTP (§5) —
        // GCS bay nhiệm vụ mà không bao giờ nói CRTP.
        crtp_last_rx_ms = millis();
      }
    }
    packetSize = MavUdp.parsePacket();
  }

  if (!mav_gcs_known) return;
  uint32_t now = millis();
  if ((int32_t)(now - mav_heartbeat_ms) >= 500) {
    mav_heartbeat_ms = now;
    mavSendHeartbeat();
    mavSendSysStatus();
  }
  if ((int32_t)(now - mav_pos_ms) >= 200) {
    mav_pos_ms = now;
    mavSendAttitude(now);
    mavSendGlobalPosition(now);
    if (mav_home_dirty && home_valid) mavSendHomePosition();
    if (mav_mission_current_dirty) mavSendMissionCurrent();
  }
}

// =============================================================================
//  CRTP TOC / PARAMETERS / LOGGING
//
//  Byte layouts are dictated by the Bitcraze "crazyflie" Java library the
//  Android client is built from, and are pinned to these files:
//    toc/TocFetcher.java  channel 0: CMD_TOC_ELEMENT=0, CMD_TOC_INFO=1
//                         info    -> <cmd><uint8 nItems><uint32 crc LE>
//                         element -> <cmd><uint8 id><type|access><group\0><name\0>
//    toc/TocElement.java  read-only when (type|access) & 0x40
//    param/Param.java     ch 1 read : <id>          -> <id><value LE>
//                         ch 2 write: <id><value LE> -> <id><value LE>
//    log/Logg.java        ch 1 ctrl : <cmd><id>...   -> <cmd><id><err>
//                         ch 2 data : <id><ts24 LE><value...>
//
//  The client keeps a 500 ms resend queue, so every handler here must be
//  idempotent: a retransmitted CREATE/START/DELETE has to re-ack success
//  rather than report "already exists" / "no such id".
//  Bố cục byte bị chi phối bởi thư viện Java "crazyflie" của Bitcraze mà client
//  Android được xây dựng từ đó, và được neo theo các tệp được liệt kê ở trên.
//
//  Client giữ một hàng đợi gửi lại 500 ms, nên mọi handler ở đây phải idempotent:
//  một CREATE/START/DELETE được gửi lại phải ack lại thành công thay vì báo
//  "already exists" / "no such id".
// =============================================================================

// The command codes and the TocEntry/LogBlock types are defined up with the
// other #defines at the top of the file; see the note there about the Arduino
// preprocessor's generated prototypes.
// Các mã lệnh và các kiểu TocEntry/LogBlock được định nghĩa cùng các #define ở
// đầu tệp; xem ghi chú ở đó về các prototype do bộ tiền xử lý Arduino sinh ra.

// The three parameters MainPresenter.java probes at connect time. This board
// has no buzzer deck, zRanger or LED ring, so all three report 0 — which is
// what keeps the matching buttons disabled instead of silently dead.
// Ba tham số mà MainPresenter.java dò khi kết nối. Bo này không có buzzer deck,
// zRanger hay LED ring, nên cả ba đều báo 0 — nhờ vậy các nút tương ứng bị mờ đi
// thay vì chết lặng lẽ.
static uint32_t p_cpu_flash      = 0;
static uint8_t  p_deck_bczranger = 0;
static uint8_t  p_ring_neffect   = 0;

static const TocEntry PARAM_TOC[] = {
  { "cpu",  "flash",     PT_UINT32, 4, TOC_ACCESS_RO, &p_cpu_flash      },
  { "deck", "bcZRanger", PT_UINT8,  1, TOC_ACCESS_RO, &p_deck_bczranger },
  { "ring", "neffect",   PT_UINT8,  1, TOC_ACCESS_RO, &p_ring_neffect   },
  // PID gains are exposed read/write so they can be tuned from the phone.
  // Các gain PID được phơi dạng đọc/ghi để có thể tinh chỉnh từ điện thoại.
  { "pid",  "roll_kp",   PT_FLOAT,  4, TOC_ACCESS_RW, &pid_p_gain_roll, 0.0f, 100.0f },
  { "pid",  "roll_ki",   PT_FLOAT,  4, TOC_ACCESS_RW, &pid_i_gain_roll, 0.0f, 20.0f },
  { "pid",  "roll_kd",   PT_FLOAT,  4, TOC_ACCESS_RW, &pid_d_gain_roll, 0.0f, 200.0f },
  { "pid",  "pitch_kp",  PT_FLOAT,  4, TOC_ACCESS_RW, &pid_p_gain_pitch, 0.0f, 100.0f },
  { "pid",  "pitch_ki",  PT_FLOAT,  4, TOC_ACCESS_RW, &pid_i_gain_pitch, 0.0f, 20.0f },
  { "pid",  "pitch_kd",  PT_FLOAT,  4, TOC_ACCESS_RW, &pid_d_gain_pitch, 0.0f, 200.0f },
  { "pid",  "yaw_kp",    PT_FLOAT,  4, TOC_ACCESS_RW, &pid_p_gain_yaw, 0.0f, 100.0f },
  { "pid",  "yaw_ki",    PT_FLOAT,  4, TOC_ACCESS_RW, &pid_i_gain_yaw, 0.0f, 20.0f },
  { "pid",  "yaw_kd",    PT_FLOAT,  4, TOC_ACCESS_RW, &pid_d_gain_yaw, 0.0f, 200.0f },
  { "flight", "hover_thrust", PT_UINT16, 2, TOC_ACCESS_RW, &hover_thrust, 8000.0f, 55000.0f },
  { "flight", "takeoff_ms",   PT_UINT16, 2, TOC_ACCESS_RW, &takeoff_ms, 100.0f, 20000.0f },
  { "flight", "land_ms",      PT_UINT16, 2, TOC_ACCESS_RW, &land_ms, 100.0f, 20000.0f },
  { "nav",    "kp",           PT_FLOAT,  4, TOC_ACCESS_RW, &nav_kp, 0.0f, 20.0f },
};
#define PARAM_TOC_COUNT ((uint8_t)(sizeof(PARAM_TOC) / sizeof(PARAM_TOC[0])))

// Telemetry the app can subscribe to. pm.vbat is the one MainPresenter's
// "Standard" config actually requests; the rest are here for flight-data views.
// There is no magnetometer, so stabilizer.yaw is the yaw RATE in deg/s.
// Dữ liệu từ xa mà app có thể đăng ký. pm.vbat là thứ cấu hình "Standard" của
// MainPresenter thực sự yêu cầu; phần còn lại phục vụ các màn hình dữ liệu bay.
// Không có từ kế, nên stabilizer.yaw là TỐC ĐỘ GÓC yaw theo deg/s.
static const TocEntry LOG_TOC[] = {
  { "pm",         "vbat",  LT_FLOAT,  4, TOC_ACCESS_RO, &vbat            },
  { "stabilizer", "roll",  LT_FLOAT,  4, TOC_ACCESS_RO, &angle_roll_deg  },
  { "stabilizer", "pitch", LT_FLOAT,  4, TOC_ACCESS_RO, &angle_pitch_deg },
  { "stabilizer", "yaw",   LT_FLOAT,  4, TOC_ACCESS_RO, &gyro_yaw        },
  { "controller", "roll",  LT_FLOAT,  4, TOC_ACCESS_RO, &pid_output_roll  },
  { "controller", "pitch", LT_FLOAT,  4, TOC_ACCESS_RO, &pid_output_pitch },
  { "controller", "yaw",   LT_FLOAT,  4, TOC_ACCESS_RO, &pid_output_yaw   },
  { "motor",      "m1",    LT_UINT16, 2, TOC_ACCESS_RO, &motor_rf         },
  { "motor",      "m2",    LT_UINT16, 2, TOC_ACCESS_RO, &motor_rr         },
  { "motor",      "m3",    LT_UINT16, 2, TOC_ACCESS_RO, &motor_lr         },
  { "motor",      "m4",    LT_UINT16, 2, TOC_ACCESS_RO, &motor_lf         },

  // Flight state, so the phone can show mode / sequence / armed without a
  // second protocol. `armed` is a bool, read back as one little-endian byte.
  // Trạng thái bay, để điện thoại hiển thị chế độ / chuỗi / armed mà không cần
  // giao thức thứ hai. `armed` là giá trị bool, đọc lại thành một byte little-endian.
  { "flight", "mode",       LT_UINT8,  1, TOC_ACCESS_RO, &flight_mode  },
  { "flight", "auto_state", LT_UINT8,  1, TOC_ACCESS_RO, &auto_state   },
  { "flight", "armed",      LT_UINT8,  1, TOC_ACCESS_RO, &armed        },
  { "flight", "heading",    LT_FLOAT,  4, TOC_ACCESS_RO, &heading_deg  },

  // Dead-reckoned position. Rough — see the NAVIGATION section. Surfaced so
  // the app can draw it and say plainly that it drifts.
  // Vị trí tự dẫn đường quán tính. Xấp xỉ — xem phần NAVIGATION. Được phơi ra
  // để app vẽ được và nói rõ rằng nó bị trôi.
  { "nav", "pos_x", LT_FLOAT, 4, TOC_ACCESS_RO, &pos_x },
  { "nav", "pos_y", LT_FLOAT, 4, TOC_ACCESS_RO, &pos_y },
  { "nav", "vel_x", LT_FLOAT, 4, TOC_ACCESS_RO, &vel_x },
  { "nav", "vel_y", LT_FLOAT, 4, TOC_ACCESS_RO, &vel_y },
  { "nav", "tgt_x", LT_FLOAT, 4, TOC_ACCESS_RO, &tgt_x },
  { "nav", "tgt_y", LT_FLOAT, 4, TOC_ACCESS_RO, &tgt_y },

  // Per-term breakdown of the PID output. Charting these is how you see which
  // loop is doing the work — a D term spiking against P is a different fix
  // from I windup, and the summed output hides the difference.
  // Phân rã từng thành phần của output PID. Vẽ các giá trị này là cách thấy vòng
  // nào đang làm việc — D gai lên chống lại P cần cách xử lý khác với I windup,
  // còn output tổng thì che mất sự khác biệt đó.
  { "pid", "roll_p",  LT_FLOAT, 4, TOC_ACCESS_RO, &pid_p_roll  },
  { "pid", "roll_i",  LT_FLOAT, 4, TOC_ACCESS_RO, &pid_i_roll  },
  { "pid", "roll_d",  LT_FLOAT, 4, TOC_ACCESS_RO, &pid_d_roll  },
  { "pid", "pitch_p", LT_FLOAT, 4, TOC_ACCESS_RO, &pid_p_pitch },
  { "pid", "pitch_i", LT_FLOAT, 4, TOC_ACCESS_RO, &pid_i_pitch },
  { "pid", "pitch_d", LT_FLOAT, 4, TOC_ACCESS_RO, &pid_d_pitch },
  { "pid", "yaw_p",   LT_FLOAT, 4, TOC_ACCESS_RO, &pid_p_yaw   },
  { "pid", "yaw_i",   LT_FLOAT, 4, TOC_ACCESS_RO, &pid_i_yaw   },
  { "pid", "yaw_d",   LT_FLOAT, 4, TOC_ACCESS_RO, &pid_d_yaw   },

  // Raw IMU, for bench work: gyro body rates (deg/s), accel in g, and the
  // accel-derived tilt angles (deg) before the complementary filter blends
  // them with the gyro. A vibration spike shows up here first, long before
  // it shows up as a wobble in stabilizer.roll.
  // IMU thô, cho việc thử trên bàn: tốc độ góc body theo deg/s, gia tốc theo g,
  // và các góc nghiêng suy ra từ gia tốc (độ) trước khi bộ lọc bổ sung trộn
  // chúng với gyro. Gai rung hiện ra ở đây trước tiên, lâu trước khi hiện
  // thành rung lắc trong stabilizer.roll.
  { "gyro", "roll",  LT_FLOAT, 4, TOC_ACCESS_RO, &gyro_roll     },
  { "gyro", "pitch", LT_FLOAT, 4, TOC_ACCESS_RO, &gyro_pitch    },
  { "gyro", "yaw",   LT_FLOAT, 4, TOC_ACCESS_RO, &gyro_yaw      },
  { "acc",  "x",     LT_FLOAT, 4, TOC_ACCESS_RO, &acc_x_g       },
  { "acc",  "y",     LT_FLOAT, 4, TOC_ACCESS_RO, &acc_y_g       },
  { "acc",  "z",     LT_FLOAT, 4, TOC_ACCESS_RO, &acc_z_g       },
  { "acc",  "roll",  LT_FLOAT, 4, TOC_ACCESS_RO, &angle_roll_acc  },
  { "acc",  "pitch", LT_FLOAT, 4, TOC_ACCESS_RO, &angle_pitch_acc },
};
#define LOG_TOC_COUNT ((uint8_t)(sizeof(LOG_TOC) / sizeof(LOG_TOC[0])))

static LogBlock log_blocks[LOG_MAX_BLOCKS];

// CRC-32 (IEEE) over the TOC contents. The client only compares this against
// its own cache key, so it has to be stable and content-derived; it does not
// need to match any external implementation.
// CRC-32 (IEEE) trên nội dung TOC. Client chỉ so giá trị này với khóa cache của
// chính nó, nên nó phải ổn định và suy từ nội dung; không cần khớp với triển
// khai bên ngoài nào.
uint32_t crc32Byte(uint32_t crc, uint8_t b) {
  crc ^= b;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
  return crc;
}

uint32_t tocCrc(const TocEntry *toc, uint8_t n) {
  uint32_t crc = 0xFFFFFFFFu;
  for (uint8_t i = 0; i < n; i++) {
    crc = crc32Byte(crc, i);
    for (const char *s = toc[i].group; *s; s++) crc = crc32Byte(crc, (uint8_t)*s);
    crc = crc32Byte(crc, 0);
    for (const char *s = toc[i].name; *s; s++) crc = crc32Byte(crc, (uint8_t)*s);
    crc = crc32Byte(crc, 0);
  }
  return ~crc;
}

// <cmd><nItems><crc32 LE> on the TOC channel of the given port.
// <cmd><nItems><crc32 LE> trên kênh TOC của port được chỉ định.
void tocSendInfo(uint8_t port, const TocEntry *toc, uint8_t n) {
  uint8_t buf[CRTP_MAX_SIZE];
  buf[0] = (uint8_t)((port << 4) | LOG_CHAN_TOC);
  buf[1] = TOC_CMD_GET_INFO;
  buf[2] = n;
  uint32_t crc = tocCrc(toc, n);
  memcpy(buf + 3, &crc, 4);
  crtpSend(buf, 7);
}

// <cmd><id><type|access><group\0><name\0>. Refusing to send rather than
// truncating matters: TocElement.java needs exactly two NUL-separated fields,
// so a clipped name would silently drop the element and desync the TOC.
// <cmd><id><type|access><group\0><name\0>. Từ chối gửi thay vì cắt bớt là điều
// quan trọng: TocElement.java cần đúng hai trường ngăn bởi NUL, nên một tên bị
// cắt sẽ lặng lẽ loại bỏ phần tử và làm lệch đồng bộ TOC.
void tocSendElement(uint8_t port, const TocEntry *toc, uint8_t n, uint8_t id) {
  if (id >= n) return;
  const TocEntry &e = toc[id];
  size_t need = 4 + strlen(e.group) + 1 + strlen(e.name) + 1;
  if (need > CRTP_MAX_SIZE) return;

  uint8_t buf[CRTP_MAX_SIZE];
  buf[0] = (uint8_t)((port << 4) | LOG_CHAN_TOC);
  buf[1] = TOC_CMD_GET_ELEMENT;
  buf[2] = id;
  buf[3] = e.ctype | e.access;
  size_t pos = 4;
  for (const char *s = e.group; *s; s++) buf[pos++] = (uint8_t)*s;
  buf[pos++] = 0;
  for (const char *s = e.name; *s; s++) buf[pos++] = (uint8_t)*s;
  buf[pos++] = 0;
  crtpSend(buf, pos);
}

// Store a value written from the phone. Floats must be finite, and every
// numeric value is clamped into the entry's [lo, hi]. Without this a NaN gain,
// or a takeoff_ms of 0 (a divide-by-zero in the ramp), reached the motors.
// Lưu giá trị được ghi từ điện thoại. Float phải hữu hạn, và mọi giá trị số bị
// kẹp trong khoảng [lo, hi] của mục đó. Không có bước này thì một gain NaN, hoặc
// takeoff_ms bằng 0 (chia cho 0 trong đoạn bám), sẽ tới được động cơ.
static float paramClampF(const TocEntry &e, float v) {
  if (v < e.lo) return e.lo;
  if (v > e.hi) return e.hi;
  return v;
}

void paramStore(const TocEntry &e, const uint8_t *src) {
  if (e.ctype == PT_FLOAT) {
    float v;
    memcpy(&v, src, 4);
    if (!isfinite(v)) return;          // refused -- the echo shows the old value
                                       // từ chối -- giá trị echo sẽ là giá trị cũ
    v = paramClampF(e, v);
    memcpy(e.value, &v, 4);
  } else if (e.ctype == PT_UINT16) {
    uint16_t v;
    memcpy(&v, src, 2);
    v = (uint16_t)paramClampF(e, (float)v);
    memcpy(e.value, &v, 2);
  } else if (src != e.value) {
    memcpy(e.value, src, e.size);      // no RW entries of other types today
                                       // hiện chưa có mục RW loại nào khác
  }
}

// After loading from NVS: a non-finite stored gain restores all defaults, then
// every RW entry is clamped into its range.
// Sau khi tải từ NVS: một gain đã lưu không hữu hạn sẽ khôi phục toàn bộ mặc định,
// rồi mọi mục RW được kẹp vào khoảng cho phép của nó.
void paramSanitizeAll() {
  bool bad = false;
  for (uint8_t i = 0; i < PARAM_TOC_COUNT; i++) {
    const TocEntry &e = PARAM_TOC[i];
    if (e.access != TOC_ACCESS_RW || e.ctype != PT_FLOAT) continue;
    float v;
    memcpy(&v, e.value, 4);
    if (!isfinite(v)) bad = true;
  }
  if (bad) {
    flightConfigDefaults();
    Serial.println(F("[CFG] stored gains were not finite -- defaults restored"));
  }
  for (uint8_t i = 0; i < PARAM_TOC_COUNT; i++) {
    const TocEntry &e = PARAM_TOC[i];
    if (e.access == TOC_ACCESS_RW) paramStore(e, (const uint8_t *)e.value);
  }
}

// ---- CRTP port 2: parameters ---------------------------------------------
void crtpHandleParams(const uint8_t *p, size_t len) {
  if (len < 2) return;
  uint8_t ch = p[0] & 0x03;

  if (ch == PARAM_CHAN_TOC) {
    if (p[1] == TOC_CMD_GET_INFO) {
      tocSendInfo(CRTP_PORT_PARAMS, PARAM_TOC, PARAM_TOC_COUNT);
    } else if (p[1] == TOC_CMD_GET_ELEMENT && len >= 3) {
      tocSendElement(CRTP_PORT_PARAMS, PARAM_TOC, PARAM_TOC_COUNT, p[2]);
    }
    return;
  }

  // Value access — <id><value LE> in both directions.
  // Truy cập giá trị — <id><value LE> ở cả hai chiều.
  uint8_t id = p[1];
  if (id >= PARAM_TOC_COUNT) return;
  const TocEntry &e = PARAM_TOC[id];

  if (ch == PARAM_CHAN_READ) {
    uint8_t buf[CRTP_MAX_SIZE];
    buf[0] = (uint8_t)((CRTP_PORT_PARAMS << 4) | PARAM_CHAN_READ);
    buf[1] = id;
    memcpy(buf + 2, e.value, e.size);
    crtpSend(buf, 2 + e.size);
  } else if (ch == PARAM_CHAN_WRITE) {
    if (len < (size_t)(2 + e.size)) return;
    // Read-only entries are not stored, but silence is the wrong answer: the
    // client's resend queue retries an unmatched write every 500 ms and never
    // gives up. Ack with whatever is actually in the variable so the matcher
    // clears; the app's read-back then shows the true value.
    // Mục chỉ đọc không được lưu, nhưng im lặng không phải câu trả lời đúng: hàng
    // đợi gửi lại của client thử lại một lần ghi chưa khớp mỗi 500 ms và không bao
    // giờ bỏ cuộc. Hãy ack với đúng giá trị đang có trong biến để bộ so khớp được
    // xóa; sau đó phần đọc lại của app sẽ hiện giá trị thật.
    if (e.access == TOC_ACCESS_RW) paramStore(e, p + 2);
    uint8_t buf[CRTP_MAX_SIZE];
    buf[0] = (uint8_t)((CRTP_PORT_PARAMS << 4) | PARAM_CHAN_WRITE);
    buf[1] = id;
    memcpy(buf + 2, e.value, e.size);           // echo what is actually stored
                                                // echo đúng giá trị đang được lưu
    crtpSend(buf, 2 + e.size);
  }
}

// ---- CRTP port 5: logging ------------------------------------------------
LogBlock *logFind(uint8_t id) {
  for (uint8_t i = 0; i < LOG_MAX_BLOCKS; i++)
    if (log_blocks[i].used && log_blocks[i].id == id) return &log_blocks[i];
  return NULL;
}

LogBlock *logAlloc() {
  for (uint8_t i = 0; i < LOG_MAX_BLOCKS; i++)
    if (!log_blocks[i].used) return &log_blocks[i];
  return NULL;
}

// Every control reply is <cmd><id><err>. Logg.java reads all three bytes
// unconditionally, so a shorter reply would throw there.
// Mọi phản hồi điều khiển là <cmd><id><err>. Logg.java đọc cả ba byte không có
// điều kiện, nên một phản hồi ngắn hơn sẽ gây lỗi ở đó.
void logReply(uint8_t cmd, uint8_t id, uint8_t err) {
  uint8_t buf[CRTP_MAX_SIZE];
  buf[0] = (uint8_t)((CRTP_PORT_LOGGING << 4) | LOG_CHAN_SETTINGS);
  buf[1] = cmd;
  buf[2] = id;
  buf[3] = err;
  crtpSend(buf, 4);
}

// <id><ts24 LE><value...> on the log-data channel. Values are read straight out
// of the live variables so one packet is internally consistent.
// <id><ts24 LE><value...> trên kênh dữ liệu log. Giá trị được đọc thẳng từ các
// biến trực tiếp để một gói luôn nhất quán nội bộ.
void logSendData(const LogBlock &b) {
  uint8_t buf[CRTP_MAX_SIZE];
  buf[0] = (uint8_t)((CRTP_PORT_LOGGING << 4) | LOG_CHAN_LOGDATA);
  buf[1] = b.id;

  uint32_t ts = (millis() - b.start_ms) & 0xFFFFFFu;
  buf[2] = (uint8_t)( ts        & 0xFF);
  buf[3] = (uint8_t)((ts >> 8)  & 0xFF);
  buf[4] = (uint8_t)((ts >> 16) & 0xFF);

  size_t pos = LOG_DATA_OVERHEAD;
  for (uint8_t i = 0; i < b.nvars; i++) {
    const TocEntry &e = LOG_TOC[b.var_id[i]];
    memcpy(buf + pos, e.value, e.size);
    pos += e.size;
  }
  crtpSend(buf, pos);
}

// Push each log block whose period has elapsed. period is in 10 ms units.
// Đẩy mỗi khối log có chu kỳ đã hết. period tính bằng đơn vị 10 ms.
void logUpdate() {
  if (!link_ready) return;      // linkSend() handles "no phone yet" for each transport
                                // linkSend() tự xử lý "chưa có điện thoại" cho từng tầng truyền
  uint32_t now = millis();
  for (uint8_t i = 0; i < LOG_MAX_BLOCKS; i++) {
    LogBlock &b = log_blocks[i];
    if (!b.used || !b.started) continue;
    if (now - b.last_ms < (uint32_t)b.period * 10u) continue;
    b.last_ms = now;
    logSendData(b);
  }
}

void crtpHandleLogging(const uint8_t *p, size_t len) {
  if (len < 2) return;
  uint8_t ch = p[0] & 0x03;

  if (ch == LOG_CHAN_TOC) {
    if (p[1] == TOC_CMD_GET_INFO) {
      tocSendInfo(CRTP_PORT_LOGGING, LOG_TOC, LOG_TOC_COUNT);
    } else if (p[1] == TOC_CMD_GET_ELEMENT && len >= 3) {
      tocSendElement(CRTP_PORT_LOGGING, LOG_TOC, LOG_TOC_COUNT, p[2]);
    }
    return;
  }
  if (ch != LOG_CHAN_SETTINGS) return;   // channel 2 is our own output
                                         // channel 2 là kênh phát ra của chính ta

  const uint8_t cmd = p[1];
  const uint8_t id  = (len >= 3) ? p[2] : 0;

  switch (cmd) {

    case LOG_CMD_RESET:
      for (uint8_t i = 0; i < LOG_MAX_BLOCKS; i++) {
        log_blocks[i].used = false;
        log_blocks[i].started = false;
        log_blocks[i].nvars = 0;
      }
      logReply(cmd, 0, LOG_ERR_OK);
      break;

    case LOG_CMD_CREATE:
    case LOG_CMD_APPEND: {
      // Payload after <cmd><id> is a flat list of <type><tocId> pairs. The type
      // byte is informational — the TOC id already fixes type and width — so
      // only the ids are kept.
      //
      // The whole block is validated before anything is committed: Logg.java
      // unpacks a log packet against its own variable list, so accepting a
      // subset would desynchronise the payload and read garbage.
      // Phần dữ liệu sau <cmd><id> là một danh sách phẳng các cặp <type><tocId>.
      // Byte type chỉ mang tính thông tin — tocId đã quyết định kiểu và độ rộng —
      // nên chỉ các id được giữ lại.
      //
      // Toàn bộ khối được kiểm tra trước khi ghi nhận gì: Logg.java giải nén một
      // gói log theo danh sách biến của chính nó, nên chấp nhận một tập con sẽ làm
      // lệch phần dữ liệu và đọc phải rác.
      uint8_t  ids[LOG_MAX_VARS];
      uint8_t  n     = 0;
      uint16_t total = 0;
      bool     ok    = true;

      LogBlock *b = logFind(id);
      if (cmd == LOG_CMD_APPEND) {
        if (!b) { logReply(cmd, id, LOG_ERR_NO_ID); break; }
        for (uint8_t i = 0; i < b->nvars; i++) {
          ids[i] = b->var_id[i];
          total += LOG_TOC[b->var_id[i]].size;
        }
        n = b->nvars;
      }

      for (size_t i = 3; i + 1 < len; i += 2) {
        // Logg.create() builds this request in a fixed 31-byte ByteBuffer and
        // sends the entire backing array (CrtpPacket does not trim), so the
        // real <type><tocId> pairs are followed by zero padding. Log type codes
        // start at 1 (see Toc.java), so a zero type byte cannot be a real
        // element and ends the list. Without this the padding would parse as
        // repeated element 0 and overflow the block.
        // Logg.create() dựng yêu cầu này trong một ByteBuffer cố định 31 byte và
        // gửi toàn bộ mảng nền (CrtpPacket không cắt bớt), nên các cặp
        // <type><tocId> thật theo sau là phần đệm bằng 0. Mã kiểu log bắt đầu từ
        // 1 (xem Toc.java), nên byte type bằng 0 không thể là phần tử thật và sẽ
        // kết thúc danh sách. Không có bước này thì phần đệm sẽ bị phân tích thành
        // phần tử 0 lặp lại và tràn khối.
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

      if (!ok) { logReply(cmd, id, LOG_ERR_TOO_LARGE); break; }

      if (cmd == LOG_CMD_CREATE && b) {
        // A retransmitted CREATE has to re-ack OK. The client resends every
        // 500 ms until something matches its expectation, and LOG_ERR_EXISTS
        // would surface to the user as a failure for what is really just a
        // lost ack. Only a genuinely different variable list is a conflict.
        // Một CREATE được gửi lại phải được ack OK lần nữa. Client gửi lại mỗi
        // 500 ms cho tới khi có gì khớp kỳ vọng, và LOG_ERR_EXISTS sẽ hiện với
        // người dùng như một thất bại cho việc vốn chỉ là một ack bị mất. Chỉ một
        // danh sách biến thật sự khác mới là xung đột.
        if (b->nvars == n && memcmp(b->var_id, ids, n) == 0) {
          logReply(cmd, id, LOG_ERR_OK);
        } else {
          logReply(cmd, id, LOG_ERR_EXISTS);
        }
        break;
      }

      if (!b) {
        b = logAlloc();
        if (!b) { logReply(cmd, id, LOG_ERR_NO_MEMORY); break; }
        memset(b, 0, sizeof(*b));
      }
      for (uint8_t i = 0; i < n; i++) b->var_id[i] = ids[i];
      b->nvars = n;
      b->used  = true;
      b->id    = id;
      logReply(cmd, id, LOG_ERR_OK);
      break;
    }

    case LOG_CMD_DELETE:
      // Deleting an absent block still acks: a lost ack would otherwise make
      // the client's retransmission look like a failed delete.
      // Xóa một khối không tồn tại vẫn phải ack: nếu không, một ack bị mất sẽ làm
      // lần gửi lại của client trông như một lần xóa thất bại.
      if (LogBlock *b = logFind(id)) {
        b->used = false;
        b->started = false;
        b->nvars = 0;
      }
      logReply(cmd, id, LOG_ERR_OK);
      break;

    case LOG_CMD_START: {
      LogBlock *b = logFind(id);
      if (!b) { logReply(cmd, id, LOG_ERR_NO_ID); break; }
      if (len >= 4 && p[3] > 0) b->period = p[3];
      if (!b->started) {
        b->started = true;
        b->start_ms = millis();
        b->last_ms  = millis();
      }
      logReply(cmd, id, LOG_ERR_OK);
      break;
    }

    case LOG_CMD_STOP:
      if (LogBlock *b = logFind(id)) b->started = false;
      logReply(cmd, id, LOG_ERR_OK);
      break;

    default:
      logReply(cmd, id, LOG_ERR_NO_CMD);
      break;
  }
}

// Dispatch one full CRTP packet (checksum already verified).
// Phân phối một gói CRTP hoàn chỉnh (checksum đã được kiểm tra).
void crtpDispatch(const uint8_t *p, size_t len) {
  uint8_t port = (p[0] >> 4) & 0x0F;
  switch (port) {
    case CRTP_PORT_COMMANDER:
      crtpHandleCommander(p, len);
      break;
    case CRTP_PORT_PARAMS:
      crtpHandleParams(p, len);
      break;
    case CRTP_PORT_LOGGING:
      crtpHandleLogging(p, len);
      break;
    case CRTP_PORT_GENERIC:
      switch (p[0] & 0x03) {
        case 0: crtpHandleFlightControl(p, len); break;   // flight control
        case 1: crtpHandleNav(p, len);          break;   // navigation
        case 2: crtpHandleMission(p, len);      break;   // mission (§12)
        default: break;
      }
      break;
    default:
      break;                             // unknown ports are ignored
  }
}

// (wifiUdpInit() / crtpPoll() moved into the TRANSPORT section above.)
// (wifiUdpInit() / crtpPoll() đã được chuyển vào phần TRANSPORT ở trên.)

// =============================================================================
//  Motor PWM  (LEDC, brushed MOSFET drive — duty cycle, not servo pulse)
//  20 kHz keeps switching above audible range for 8520 coreless motors.
//  Motor PWM  (LEDC, điều khiển MOSFET chổi than — duty cycle, không phải xung servo)
//  20 kHz giữ tần số chuyển mạch trên ngưỡng nghe được của động cơ coreless 8520.
// =============================================================================
#define MOTOR_PWM_FREQ_HZ  20000
#define MOTOR_PWM_BITS     10          // 0..1023 duty
#define MOTOR_PWM_MAX      ((1 << MOTOR_PWM_BITS) - 1)

void motorInit() {
  // ESP32 Arduino core 2.x: ledcSetup / ledcAttachPin  —  3.x: ledcAttach
  const uint8_t pins[4] = { PIN_MOTOR_RF, PIN_MOTOR_RR, PIN_MOTOR_LR, PIN_MOTOR_LF };
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(pins[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
    ledcWrite(pins[i], 0);
#else
    ledcSetup(i, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
    ledcAttachPin(pins[i], i);
    ledcWrite(i, 0);
#endif
  }
}

// Convert the internal 1000..2000 µs convention to a 0..1023 duty
// Chuyển quy ước nội bộ 1000..2000 µs thành duty 0..1023
inline uint16_t usToDuty(int16_t us) {
  if (us <= 1000) return 0;
  if (us >= 2000) return MOTOR_PWM_MAX;
  return (uint16_t)(((uint32_t)(us - 1000) * MOTOR_PWM_MAX) / 1000);
}

void motorWrite(int16_t rf, int16_t rr, int16_t lr, int16_t lf) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_MOTOR_RF, usToDuty(rf));
  ledcWrite(PIN_MOTOR_RR, usToDuty(rr));
  ledcWrite(PIN_MOTOR_LR, usToDuty(lr));
  ledcWrite(PIN_MOTOR_LF, usToDuty(lf));
#else
  ledcWrite(0, usToDuty(rf));
  ledcWrite(1, usToDuty(rr));
  ledcWrite(2, usToDuty(lr));
  ledcWrite(3, usToDuty(lf));
#endif
}

void motorStop() {
  motorWrite(1000, 1000, 1000, 1000);
}

// =============================================================================
//  Battery monitor
// =============================================================================
void batteryUpdate() {
  // analogReadMilliVolts() uses the factory Vref calibration
  // analogReadMilliVolts() dùng hiệu chỉnh Vref của nhà máy
  uint32_t mv = analogReadMilliVolts(PIN_VBAT);
  float sample = (mv / 1000.0f) * VBAT_DIVIDER;
  // Smooth it: one ADC read is noisy and sags with every throttle punch.
  // Làm mượt: một lần đọc ADC rất nhiễu và sụt theo mỗi cú đẩy throttle.
  if (vbat <= 0.0f) vbat = sample;
  else              vbat += VBAT_FILTER_ALPHA * (sample - vbat);
  vbat_warn = (vbat < VBAT_CELLS * VBAT_CELL_WARN);

  uint32_t now = millis();
  if (vbat < VBAT_CELLS * VBAT_CELL_CRIT) {
    if (vbat_low_since_ms == 0) vbat_low_since_ms = now;
    vbat_crit = (now - vbat_low_since_ms) >= VBAT_CRIT_HOLD_MS;
  } else {
    vbat_low_since_ms = 0;
    vbat_crit = false;
  }
}

// Pre-arm checks shared by every arming path (SBUS, CRTP thrust-up, ARM,
// TAKEOFF). The message is rate-limited so a held stick or a streaming thrust
// value cannot flood the console.
// Các bước kiểm tra trước khi kích hoạt, dùng chung cho mọi đường kích hoạt
// (SBUS, CRTP lực đẩy-up, ARM, TAKEOFF). Thông báo bị giới hạn tần suất để một
// cần gạt giữ nguyên hoặc một luồng giá trị lực đẩy không thể làm ngập console.
bool armChecksOk(const char *who) {
#if VBAT_ARM_CHECK
  if (vbat < VBAT_CELLS * VBAT_CELL_CRIT) {
    static uint32_t last_msg_ms = 0;
    uint32_t now = millis();
    if (now - last_msg_ms > 1000) {
      last_msg_ms = now;
      Serial.printf("[ARM] %s refused: battery %.2f V is below %.2f V\n",
                    who, vbat, VBAT_CELLS * VBAT_CELL_CRIT);
      crtpConsole("arm refused: low battery");
    }
    return false;
  }
#endif
  (void)who;
  return true;
}

// =============================================================================
//  PID  (rate loop; stick gives rate setpoint, accelerometer levels the frame)
//  PID  (vòng lặp tốc độ góc; cần gạt cho setpoint tốc độ góc, gia tốc kế làm cân bằng khung)
// =============================================================================
void pidCompute() {
  float err;

  // ---- ROLL ----
  err = gyro_roll - roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * err;
  pid_i_mem_roll = constrain(pid_i_mem_roll, -I_TERM_LIMIT, I_TERM_LIMIT);
  pid_p_roll = pid_p_gain_roll * err;
  pid_i_roll = pid_i_mem_roll;
  pid_d_roll = pid_d_gain_roll * (err - pid_last_roll_err);
  pid_output_roll = pid_p_roll + pid_i_roll + pid_d_roll;
  pid_last_roll_err = err;
  pid_output_roll = constrain(pid_output_roll, -PID_LIMIT, PID_LIMIT);

  // ---- PITCH ----
  err = gyro_pitch - pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * err;
  pid_i_mem_pitch = constrain(pid_i_mem_pitch, -I_TERM_LIMIT, I_TERM_LIMIT);
  pid_p_pitch = pid_p_gain_pitch * err;
  pid_i_pitch = pid_i_mem_pitch;
  pid_d_pitch = pid_d_gain_pitch * (err - pid_last_pitch_err);
  pid_output_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  pid_last_pitch_err = err;
  pid_output_pitch = constrain(pid_output_pitch, -PID_LIMIT, PID_LIMIT);

  // ---- YAW ----
  err = gyro_yaw - yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * err;
  pid_i_mem_yaw = constrain(pid_i_mem_yaw, -I_TERM_LIMIT, I_TERM_LIMIT);
  pid_p_yaw = pid_p_gain_yaw * err;
  pid_i_yaw = pid_i_mem_yaw;
  pid_d_yaw = pid_d_gain_yaw * (err - pid_last_yaw_err);
  pid_output_yaw = pid_p_yaw + pid_i_yaw + pid_d_yaw;
  pid_last_yaw_err = err;
  pid_output_yaw = constrain(pid_output_yaw, -PID_LIMIT, PID_LIMIT);
}

void pidReset() {
  pid_i_mem_roll = pid_i_mem_pitch = pid_i_mem_yaw = 0;
  pid_last_roll_err = pid_last_pitch_err = pid_last_yaw_err = 0;
  pid_output_roll = pid_output_pitch = pid_output_yaw = 0;
}

// =============================================================================
//  Attitude  (complementary filter, 250 Hz)
//  gyro_pitch/gyro_roll/gyro_yaw are RATES in deg/s (fed to the PID).
//  angle_pitch_deg/angle_roll_deg are filtered ANGLES in deg (used to self-level).
//  Tư thế  (bộ lọc bổ sung, 250 Hz)
//  gyro_pitch/gyro_roll/gyro_yaw là TỐC ĐỘ GÓC theo deg/s (đưa vào PID).
//  angle_pitch_deg/angle_roll_deg là GÓC đã lọc theo độ (dùng để tự cân bằng).
// =============================================================================

void attitudeUpdate() {
  // raw minus bias, with configurable axis signs
  // giá trị thô trừ sai lệch bias, với dấu trục có thể cấu hình
  const float gx = (raw_gx - gyro_x_cal) * GYRO_X_SIGN / 65.5f;   // deg/s
  const float gy = (raw_gy - gyro_y_cal) * GYRO_Y_SIGN / 65.5f;
  const float gz = (raw_gz - gyro_z_cal) * GYRO_Z_SIGN / 65.5f;

  gyro_pitch = gx;    // rate to PID
  gyro_roll  = gy;
  gyro_yaw   = gz;

  // ---- accelerometer tilt angles ----
  const float ax = raw_ax * (float)ACC_X_SIGN;
  const float ay = raw_ay * (float)ACC_Y_SIGN;
  const float az = raw_az;

  // g-scaled accel, for the IMU log. NAV_ACC_LSB_PER_G matches the MPU6500
  // ±8 g config; it must stay in step with MPU_REG_ACCEL_CFG.
  // Gia tốc kế theo g, cho khối log IMU. NAV_ACC_LSB_PER_G khớp cấu hình ±8 g
  // của MPU6500; phải luôn khớp với MPU_REG_ACCEL_CFG.
  acc_x_g = ax / NAV_ACC_LSB_PER_G;
  acc_y_g = ay / NAV_ACC_LSB_PER_G;
  acc_z_g = az / NAV_ACC_LSB_PER_G;

  float acc_total = sqrtf(ax * ax + ay * ay + az * az);
  if (acc_total < 1.0f) acc_total = 1.0f;

  // only trust asin argument while |axis| < |vector|
  // chỉ tin đối số asin khi |trục| < |vectơ|
  if (fabsf(ay) < acc_total) angle_pitch_acc = asinf(ay / acc_total) * RAD_TO_DEG;
  if (fabsf(ax) < acc_total) angle_roll_acc  = asinf(-ax / acc_total) * RAD_TO_DEG;

  const float dt = LOOP_TIME_US / 1000000.0f;
  if (!gyro_angles_set) {
    angle_pitch_deg = angle_pitch_acc;
    angle_roll_deg  = angle_roll_acc;
    gyro_angles_set = true;
  } else {
    angle_pitch_deg += gx * dt;
    angle_roll_deg  += gy * dt;
    angle_pitch_deg = angle_pitch_deg * 0.996f + angle_pitch_acc * 0.004f;
    angle_roll_deg  = angle_roll_deg  * 0.996f + angle_roll_acc  * 0.004f;
  }

  // Heading is integrated from the yaw rate. There is no magnetometer on this
  // board, so heading hold is only as good as the gyro bias over a flight —
  // it drifts slowly. That is a real limit, not something to paper over.
  // Hướng mũi được tích phân từ tốc độ góc yaw. Bo này không có từ kế, nên việc
  // giữ hướng chỉ tốt bằng sai lệch bias của gyro qua một chuyến bay — nó trôi
  // chậm. Đó là giới hạn thật, không phải thứ để che đậy.
  heading_deg += gz * dt;
  if      (heading_deg >= 360.0f) heading_deg -= 360.0f;
  else if (heading_deg < 0.0f)    heading_deg += 360.0f;
}

// =============================================================================
//  NAVIGATION  — accelerometer dead reckoning toward a phone-supplied target
//
//  This board has no GPS, no magnetometer and no rangefinder, so the drone
//  cannot measure where it is. What it can do is integrate its own
//  accelerometer and fly toward an offset the phone computed from the phone's
//  own GPS. That estimate drifts — metres within a minute — because MEMS
//  accelerometer bias is integrated twice. The mitigations here are a leaky
//  velocity integrator, a velocity zero whenever the drone is holding still,
//  and a position reset at launch and landing. Treat the result as "roughly
//  toward the pilot", never as a tracked position.
//  DẪN ĐƯỜNG — tự dẫn đường quán tính bằng gia tốc kế về mục tiêu do điện thoại cung cấp
//
//  Bo này không có GPS, không có từ kế và không có cảm biến khoảng cách, nên
//  drone không thể đo nó đang ở đâu. Việc nó có thể làm là tích phân gia tốc kế
//  của chính nó và bay về phía một độ lệch do điện thoại tính từ GPS của điện
//  thoại. Ước lượng đó bị trôi — vài mét trong một phút — vì sai lệch gia tốc kế
//  MEMS bị tích phân hai lần. Các biện pháp khắc phục ở đây là bộ tích phân vận tốc
//  có rò rỉ, đặt vận tốc về 0 mỗi khi drone đứng yên, và đặt lại vị trí khi cất
//  cánh và hạ cánh. Hãy coi kết quả là "đại khái về phía người lái", đừng bao giờ
//  coi đó là vị trí được theo dõi.
// =============================================================================
void navReset() {
  pos_x = pos_y = 0.0f;
  vel_x = vel_y = 0.0f;
  heading_ref_deg = heading_deg;
  tgt_valid = false;
}

void navUpdate(float dt, bool holding) {
  const float G = NAV_GRAVITY;

  // MPU6500 specific force -> m/s^2. It reads +1 g on the vertical axis at
  // rest, so subtracting the attitude-derived gravity leaves linear accel.
  // Lực riêng của MPU6500 -> m/s^2. Khi nghỉ nó đọc +1 g trên trục thẳng đứng,
  // nên trừ trọng lực suy từ tư thế đi sẽ còn gia tốc tuyến tính.
  const float ax_b = raw_ax / NAV_ACC_LSB_PER_G * G;
  const float ay_b = raw_ay / NAV_ACC_LSB_PER_G * G;

  const float roll_rad  = angle_roll_deg  * DEG_TO_RAD;
  const float pitch_rad = angle_pitch_deg * DEG_TO_RAD;
  const float sg = sinf(roll_rad),  cg = cosf(roll_rad);
  const float st = sinf(pitch_rad), ct = cosf(pitch_rad);

  // Gravity in the sensor frame. Matches the firmware's own tilt convention
  // (pitch from +ay, roll from -ax), so the unit vector is consistent.
  // Trọng lực trong khung cảm biến. Khớp quy ước nghiêng của chính firmware
  // (pitch từ +ay, roll từ -ax), nên vectơ đơn vị luôn nhất quán.
  const float gx_g = -G * sg * ct;
  const float gy_g = +G * st;

  // Linear acceleration along the two horizontal sensor axes.
  // Gia tốc tuyến tính dọc hai trục cảm biến nằm ngang.
  float fwd  = (ay_b - gy_g) * (float)NAV_FWD_SIGN;
  float left = (ax_b - gx_g) * (float)NAV_LEFT_SIGN;

  // Rotate into the launch frame using the yaw since launch.
  // Quay vào khung cất cánh bằng yaw kể từ lúc cất cánh.
  const float yaw_rel = (heading_deg - heading_ref_deg) * (float)NAV_HEADING_SIGN * DEG_TO_RAD;
  const float cy = cosf(yaw_rel), sy = sinf(yaw_rel);
  const float ax_w = fwd * cy - left * sy;
  const float ay_w = fwd * sy + left * cy;

  vel_x += ax_w * dt;
  vel_y += ay_w * dt;

  // Leaky integrator: without this, bias integrates into a velocity error that
  // grows without bound. The leak costs a little of any sustained move, which
  // is the right trade for a craft that mostly hovers.
  // Bộ tích phân có rò rỉ: không có nó, bias tích thành sai số vận tốc tăng vô hạn.
  // Khoản rò làm mất một chút khi di chuyển liên tục, nhưng đó là đánh đổi đúng cho
  // một vật thể phần lớn thời gian ở trạng thái hover.
  const float leak = powf(NAV_VEL_LEAK_PER_S, dt);
  vel_x *= leak;
  vel_y *= leak;

  // Holding still is the one thing we know for certain, so use it.
  // Đứng yên là điều duy nhất ta biết chắc, nên hãy tận dụng nó.
  if (holding) {
    vel_x = 0.0f;
    vel_y = 0.0f;
  }

  pos_x += vel_x * dt;
  pos_y += vel_y * dt;
}

// Position controller: turn an offset error into a tilt angle toward it.
// Fills sp_roll_deg / sp_pitch_deg; yaw is left to the heading hold.
// Bộ điều khiển vị trí: biến sai số độ lệch thành góc nghiêng về phía mục tiêu.
// Điền sp_roll_deg / sp_pitch_deg; yaw để cho phần giữ hướng đảm nhiệm.
void navSetpoints() {
  float ex = tgt_x - pos_x;
  float ey = tgt_y - pos_y;
  float dist = sqrtf(ex * ex + ey * ey);

  if (dist < NAV_STOP_BAND_M || !tgt_valid) {
    // close enough (or no target): hold where we are
    // đã đủ gần (hoặc không có mục tiêu): giữ nguyên vị trí
    return;
  }

  // Command a tilt toward the target, magnitude proportional to distance and
  // capped. nav_kp is already deg-of-tilt per metre, so mag is in degrees.
  // Ra lệnh nghiêng về phía mục tiêu, độ lớn tỷ lệ với khoảng cách và bị chặn.
  // nav_kp đã tính bằng độ nghiêng trên mỗi mét, nên mag tính bằng độ.
  float mag = constrain(dist * nav_kp, 0.0f, (float)NAV_MAX_TILT_DEG);
  float ux = ex / dist * mag;             // toward the target, launch frame
  float uy = ey / dist * mag;

  const float yaw_rel = (heading_deg - heading_ref_deg) * (float)NAV_HEADING_SIGN * DEG_TO_RAD;
  const float cy = cosf(yaw_rel), sy = sinf(yaw_rel);
  float body_fwd   =  ux * cy + uy * sy;
  float body_left  = -ux * sy + uy * cy;

  // +x body = nose, and a nose-up pitch is +pitch; +y body = left, and a
  // left-wing-down roll is +roll in this firmware's convention.
  // +x thân = mũi, và pitch mũi hướng lên là +pitch; +y thân = trái, và roll
  // cánh trái hướng xuống là +roll theo quy ước của firmware này.
  sp_pitch_deg = constrain(body_fwd,  -MAX_TILT_DEG, MAX_TILT_DEG);
  sp_roll_deg  = constrain(body_left, -MAX_TILT_DEG, MAX_TILT_DEG);
}

// =============================================================================
//  FLIGHT MODES  — manual / hold / auto(takeoff -> hold -> land)
//
//  This board has ONLY an MPU6500: no barometer, no ToF/rangefinder, no GPS.
//  So "hold" holds attitude level + heading locked + a held THRUST, and "auto"
//  is a takeoff / hover / land sequence. Neither one claims an altitude —
//  hover height is whatever hover_thrust produces and drifts as the battery
//  sags. The phone UI says so instead of drawing a fake altitude hold.
//
//  Modes apply to the CRTP (phone) path only. SBUS stays rate-mode manual
//  always: the physical RC is the safety net and must never be overridden.
//  CÁC CHẾ ĐỘ BAY — manual / hold / auto(cất cánh -> giữ -> hạ cánh)
//
//  Bo này CHỈ có MPU6500: không áp kế, không ToF/không cảm biến khoảng cách,
//  không GPS. Nên "hold" giữ tư thế cân bằng + khóa hướng mũi + giữ mức LỰC ĐẨU,
//  còn "auto" là chuỗi cất cánh / hover / hạ cánh. Không chế độ nào cam kết giữ
//  độ cao — độ cao khi hover là thứ hover_thrust tạo ra và bị trôi khi pin yếu đi.
//  Giao diện điện thoại nói rõ điều đó thay vì vẽ một mức giữ độ cao giả.
//
//  Các chế độ chỉ áp dụng cho đường CRTP (điện thoại). SBUS luôn ở chế độ rate
//  manual: RC vật lý là lưới an toàn và không bao giờ được ghi đè.
// =============================================================================
void flightConfigDefaults() {
  pid_p_gain_roll  = PID_ROLL_KP;   pid_i_gain_roll  = PID_ROLL_KI;   pid_d_gain_roll  = PID_ROLL_KD;
  pid_p_gain_pitch = PID_PITCH_KP;  pid_i_gain_pitch = PID_PITCH_KI;  pid_d_gain_pitch = PID_PITCH_KD;
  pid_p_gain_yaw   = PID_YAW_KP;    pid_i_gain_yaw   = PID_YAW_KI;    pid_d_gain_yaw   = PID_YAW_KD;
  hover_thrust = HOVER_THRUST_DEFAULT;
  takeoff_ms   = TAKEOFF_MS_DEFAULT;
  land_ms      = LAND_MS_DEFAULT;
}

// The nine gains plus the flight config are what the phone's tuning screen
// persists. Stored as raw floats so a read-back is byte-identical to what was
// written — no rounding between the editor and the loop.
// Chín gain cùng cấu hình bay là những gì màn hình tinh chỉnh của điện thoại lưu
// lại. Được lưu dạng float thô để khi đọc lại giống hệt từng byte với giá trị đã
// ghi — không có làm tròn giữa trình biên tập và vòng lặp.
void cfgSave() {
  float g[9] = {
    pid_p_gain_roll,  pid_i_gain_roll,  pid_d_gain_roll,
    pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch,
    pid_p_gain_yaw,   pid_i_gain_yaw,   pid_d_gain_yaw,
  };
  uint16_t f[3] = { hover_thrust, takeoff_ms, land_ms };
  nvs.putBytes("pid",  g, sizeof(g));
  nvs.putBytes("fcfg", f, sizeof(f));
  missionSave();          // mission + home origin too (§12) // thêm nhiệm vụ + điểm gốc
  Serial.println(F("[CFG] gains + flight config saved to NVS"));
}

void cfgLoad() {
  float g[9];
  if (nvs.getBytes("pid", g, sizeof(g)) == sizeof(g)) {
    pid_p_gain_roll  = g[0]; pid_i_gain_roll  = g[1]; pid_d_gain_roll  = g[2];
    pid_p_gain_pitch = g[3]; pid_i_gain_pitch = g[4]; pid_d_gain_pitch = g[5];
    pid_p_gain_yaw   = g[6]; pid_i_gain_yaw   = g[7]; pid_d_gain_yaw   = g[8];
    Serial.println(F("[CFG] gains loaded from NVS"));
  }
  uint16_t f[3];
  if (nvs.getBytes("fcfg", f, sizeof(f)) == sizeof(f)) {
    hover_thrust = constrain(f[0], HOVER_THRUST_MIN, HOVER_THRUST_MAX);
    takeoff_ms = f[1];
    land_ms    = f[2];
    Serial.println(F("[CFG] flight config loaded from NVS"));
  }
  missionLoad();          // mission + home origin (§12) // nhiệm vụ + điểm gốc
  paramSanitizeAll();
}

void flightSetMode(uint8_t mode) {
  if (mode > MODE_RTH) return;
  if (mode != MODE_AUTO) auto_state = AUTO_IDLE;   // FOLLOW/RTH are not a sequence
                                                   // FOLLOW/RTH không phải một chuỗi
  if (mode != MODE_AUTO) missionStop();   // leaving AUTO stops the runner (§12)
                                          // rời AUTO là dừng runner (§12)
  if (mode == MODE_HOLD) hold_heading_deg = heading_deg;   // lock where we are
                                                           // khóa hướng theo vị trí hiện tại
  flight_mode = mode;
}

bool flightTakeoff(uint16_t thrust) {
  hover_thrust = constrain(thrust, HOVER_THRUST_MIN, HOVER_THRUST_MAX);
  // A retransmitted TAKEOFF (the client resends every 500 ms until it sees the
  // ack) must not restart the ramp from zero while climbing or hovering --
  // that would drop the drone. Treat it as already done.
  // Một TAKEOFF được gửi lại (client gửi lại mỗi 500 ms cho tới khi thấy ack) không
  // được khởi động lại đoạn bám từ 0 khi đang lên hoặc đang hover --
  // làm vậy sẽ làm rơi drone. Hãy coi như đã hoàn tất.
  if (armed && flight_mode == MODE_AUTO &&
      (auto_state == AUTO_TAKEOFF || auto_state == AUTO_HOLD ||
       auto_state == AUTO_MISSION)) return true;
  // Flying in another mode: a ramp from zero thrust is never what is wanted.
  // Đang bay ở chế độ khác: một đoạn bám từ lực đẩy 0 không bao giờ là điều muốn.
  bool spinning = motor_rf > 1000 || motor_rr > 1000 || motor_lr > 1000 || motor_lf > 1000;
  if (armed && spinning) return false;
  if (!armed && !armChecksOk("TAKEOFF")) return false;
  hold_heading_deg = heading_deg;
  navReset();                        // this is the launch point
                                     // đây là điểm cất cánh
  flight_mode = MODE_AUTO;
  auto_state  = AUTO_TAKEOFF;
  auto_phase_start_ms = millis();
  armed = true;
  pidReset();
  Serial.printf("[FLIGHT] takeoff -> hover thrust %u\n", hover_thrust);
  return true;
}

void flightLand() {
  missionStop();          // landing ends any mission (§12) // hạ cánh kết thúc nhiệm vụ
  if (flight_mode != MODE_AUTO) {
    flight_mode = MODE_AUTO;
    hold_heading_deg = heading_deg;
  }
  auto_state = AUTO_LAND;
  auto_phase_start_ms = millis();
  Serial.println(F("[FLIGHT] landing"));
}

// Always honoured, whatever the mode: props off, now.
// Luôn được thực hiện, bất kể chế độ nào: tắt cánh quạt ngay lập tức.
void flightEmergency() {
  armed = false;
  motor_test_active = false;   // EMERGENCY cuts a bench test too
                               // EMERGENCY cũng cắt cả thử bàn
  missionStop();               // and any mission // và mọi nhiệm vụ
  motorStop();
  pidReset();
  flight_mode = MODE_MANUAL;
  auto_state  = AUTO_IDLE;
  Serial.println(F("[FLIGHT] EMERGENCY — motors cut"));
}

// Fill sp_* from the active mode. Called once per loop, before pidCompute().
// Điền sp_* theo chế độ đang hoạt động. Gọi một lần mỗi vòng lặp, trước pidCompute().
void flightSetpoints(uint32_t now, bool use_crtp, bool sbus_ok) {
  sp_roll_deg = sp_pitch_deg = 0.0f;
  sp_roll_rate = sp_pitch_rate = sp_yaw_rate = 0.0f;
  sp_throttle_us = 1000;

  if (use_crtp && flight_mode != MODE_MANUAL) {
    // ---- HOLD / AUTO: the state machine owns every setpoint ----
    // ---- HOLD / AUTO: máy trạng thái làm chủ mọi setpoint ----
    float thrust = (float)hover_thrust;

    if (flight_mode == MODE_AUTO) {
      switch (auto_state) {
        case AUTO_TAKEOFF: {
          float t = (float)(now - auto_phase_start_ms) / (float)(takeoff_ms ? takeoff_ms : 1);
          if (t >= 1.0f) {
            t = 1.0f;
            auto_state = AUTO_HOLD;
            auto_phase_start_ms = now;
            Serial.println(F("[FLIGHT] takeoff complete -> hover"));
          }
          thrust = t * (float)hover_thrust;
          break;
        }
        case AUTO_HOLD:
          thrust = (float)hover_thrust;
          // A started mission (§12) takes over from plain hover.
          // Nhiệm vụ đã bắt đầu (§12) tiếp quản từ trạng thái lơ lửng.
          if (mission_active) {
            auto_state = AUTO_MISSION;
            auto_phase_start_ms = now;
            Serial.println(F("[MSN] mission legs"));
          }
          break;
        case AUTO_MISSION:
          thrust = (float)hover_thrust;
          missionTick(now);       // sets tgt_* — navSetpoints() chases them below
                                  // đặt tgt_* — navSetpoints() bám theo ở dưới
          break;
        case AUTO_LAND: {
          float t = (float)(now - auto_phase_start_ms) / (float)(land_ms ? land_ms : 1);
          if (t >= 1.0f) {
            auto_state   = AUTO_IDLE;
            flight_mode  = MODE_MANUAL;
            armed        = false;
            motorStop();
            pidReset();
            Serial.println(F("[FLIGHT] landed, disarmed"));
            return;
          }
          thrust = (1.0f - t) * (float)hover_thrust;
          break;
        }
        default:
          thrust = 0.0f;
          break;
      }
    }

    // Heading hold: turn a heading error into a yaw rate setpoint.
    // Giữ hướng mũi: biến sai số hướng thành setpoint tốc độ góc yaw.
    float herr = hold_heading_deg - heading_deg;
    if      (herr >  180.0f) herr -= 360.0f;
    else if (herr < -180.0f) herr += 360.0f;
    sp_yaw_rate = constrain(herr * HEADING_KP, -MAX_YAW_RATE, MAX_YAW_RATE);

    // Attitude target is level, with no rate feed-forward — that is the
    // whole point of hold: the pilot is not flying the axes.
    // Mục tiêu tư thế là cân bằng, không có feed-forward tốc độ góc — đó là
    // toàn bộ ý nghĩa của hold: người lái không điều khiển các trục.
    sp_roll_deg = sp_pitch_deg = 0.0f;
    sp_roll_rate = sp_pitch_rate = 0.0f;

    // FOLLOW/RTH tilt toward the target instead of holding level. The
    // controller works in the body frame, so the drone slides toward the
    // target whichever way it happens to face; the phone can still command
    // heading separately if it wants the nose pointed at the pilot.
    // FOLLOW/RTH nghiêng về phía mục tiêu thay vì giữ cân bằng. Bộ điều khiển
    // làm việc trong khung thân, nên drone trượt về phía mục tiêu bất kể nó đang
    // quay mặt hướng nào; điện thoại vẫn có thể ra lệnh hướng riêng nếu muốn mũi
    // chỉ về phía người lái.
    // A mission leg steers exactly like FOLLOW/RTH — missionTick() owns tgt_*.
    // Chân nhiệm vụ lái y như FOLLOW/RTH — missionTick() làm chủ tgt_*.
    if (flight_mode == MODE_FOLLOW || flight_mode == MODE_RTH ||
        (flight_mode == MODE_AUTO && auto_state == AUTO_MISSION)) {
      if (flight_mode == MODE_RTH) { tgt_x = 0.0f; tgt_y = 0.0f; tgt_valid = true; }
      navSetpoints();
    }

    sp_throttle_us = (int16_t)(1000.0f + (thrust / 65535.0f) * 1000.0f);
    return;
  }

  if (use_crtp) {
    // ---- MANUAL over CRTP: a CommanderPacket commands tilt ANGLES ----
    // ---- MANUAL qua CRTP: CommanderPacket ra lệnh GÓC nghiêng ----
    sp_roll_deg    = constrain(cmd_roll_deg,  -MAX_TILT_DEG, MAX_TILT_DEG);
    sp_pitch_deg   = constrain(cmd_pitch_deg, -MAX_TILT_DEG, MAX_TILT_DEG);
    sp_roll_rate   = 0.0f;
    sp_pitch_rate  = 0.0f;
    sp_yaw_rate    = constrain(cmd_yaw_dps,   -MAX_YAW_RATE, MAX_YAW_RATE);
    sp_throttle_us = (int16_t)(1000.0f + (cmd_thrust / 65535.0f) * 1000.0f);
    return;
  }

  if (!sbus_ok) return;

  // ---- SBUS: rate mode. Sticks command body RATES and the frame self-levels,
  //      so the attitude target stays 0 and the stick feeds the rate term. ----
  // ---- SBUS: chế độ rate. Cần gạt ra lệnh TỐC ĐỘ GÓC thân và khung tự cân bằng,
  //      nên mục tiêu tư thế giữ ở 0 và cần gạt cấp vào thành phần tốc độ góc. ----
  if (rc[CH_ROLL] > 1508)       sp_roll_rate  = (rc[CH_ROLL]  - 1508) / SETPOINT_DIVISOR;
  else if (rc[CH_ROLL] < 1492)  sp_roll_rate  = (rc[CH_ROLL]  - 1492) / SETPOINT_DIVISOR;

  if (rc[CH_PITCH] > 1508)      sp_pitch_rate = (rc[CH_PITCH] - 1508) / SETPOINT_DIVISOR;
  else if (rc[CH_PITCH] < 1492) sp_pitch_rate = (rc[CH_PITCH] - 1492) / SETPOINT_DIVISOR;

  if (rc[CH_THROTTLE] > 1050) {
    if (rc[CH_YAW] > 1508)       sp_yaw_rate = (rc[CH_YAW] - 1508) / SETPOINT_DIVISOR;
    else if (rc[CH_YAW] < 1492)  sp_yaw_rate = (rc[CH_YAW] - 1492) / SETPOINT_DIVISOR;
  }
  sp_throttle_us = rc[CH_THROTTLE];
}

// =============================================================================
//  Arm / disarm
//  CRTP path (phone app): manual mode auto-arms on throttle-up and auto-disarms
//  when thrust falls back to zero. HOLD/AUTO arm and disarm explicitly, driven
//  by the flight-control commands (takeoff arms; landing end / emergency cuts).
//  SBUS path (RC transmitter): throttle low + yaw gesture / CH5 switch.
//  Kích hoạt / ngắt kích hoạt
//  Đường CRTP (app điện thoại): chế độ manual tự kích hoạt khi tăng throttle và tự
//  ngắt khi lực đẩy về không. HOLD/AUTO kích hoạt và ngắt rõ ràng, theo các lệnh
//  điều khiển bay (cất cánh thì kích hoạt; hết hạ cánh / emergency thì cắt).
//  Đường SBUS (tay phát RC): throttle thấp + thao tác yaw / công tắc CH5.
// =============================================================================
void armUpdate(uint32_t now, bool use_crtp) {
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
        Serial.println(F("[ARM] Armed (CRTP thrust-up) — props will spin!"));
      }
    } else {
      // disarm the moment the app drops thrust to (near) zero
      // ngắt kích hoạt ngay khi app hạ lực đẩy về (gần) 0
      if (cmd_thrust <= 500) {
        armed = false;
        motorStop();
        pidReset();
        Serial.println(F("[ARM] Disarmed (CRTP thrust-down)."));
      }
    }
    return;
  }

  bool throttle_low = (rc[CH_THROTTLE] < THROTTLE_ARM_MAX);
  bool stick_arm    = (rc[CH_YAW] > YAW_ARM_THRESHOLD);
  bool stick_disarm = (rc[CH_YAW] < YAW_DISARM_THR);
  bool switch_arm   = (rc[CH_ARM] > 1500);

  if (!armed) {
    disarm_stick_ms = 0;
    if (throttle_low && (stick_arm || switch_arm)) {
      if (arm_stick_ms == 0) arm_stick_ms = now;
      else if (now - arm_stick_ms >= ARM_HOLD_MS && armChecksOk("SBUS")) {
        armed = true;
        pidReset();
        arm_stick_ms = 0;
        Serial.println(F("[ARM] Armed — props will spin!"));
      }
    } else {
      arm_stick_ms = 0;
    }
  } else {
    arm_stick_ms = 0;
    // Disarm: throttle low + yaw-left held, OR arm switch dropped (if one is used)
    // Ngắt kích hoạt: throttle thấp + giữ yaw trái, HOẶC công tắc kích hoạt hạ xuống (nếu có dùng)
    bool switch_disarm = arm_switch_seen && !switch_arm;
    if ((throttle_low && stick_disarm) || switch_disarm) {
      if (disarm_stick_ms == 0) disarm_stick_ms = now;
      else if (now - disarm_stick_ms >= ARM_HOLD_MS) {
        armed = false;
        disarm_stick_ms = 0;
        motorStop();
        Serial.println(F("[ARM] Disarmed."));
      }
    } else {
      disarm_stick_ms = 0;
    }
  }

  if (switch_arm) arm_switch_seen = true;   // remember a switch is in use
                                            // ghi nhớ rằng có dùng công tắc
}

// =============================================================================
//  Motor mixing  (X-quad, matches schematic labels)
//  Trộn tín hiệu động cơ  (X-quad, khớp các nhãn trên sơ đồ)
//    RF = throttle - pitch + roll - yaw
//    RR = throttle + pitch + roll + yaw
//    LR = throttle + pitch - roll - yaw
//    LF = throttle - pitch - roll + yaw
// =============================================================================
void motorMix(int16_t throttle) {
  int16_t t = constrain(throttle, RC_MIN, THROTTLE_MAX);

  if (t > THROTTLE_ARM_MAX) {
    motor_rf = t - (int16_t)pid_output_pitch + (int16_t)pid_output_roll - (int16_t)(pid_output_yaw * YAW_DIRECTION);
    motor_rr = t + (int16_t)pid_output_pitch + (int16_t)pid_output_roll + (int16_t)(pid_output_yaw * YAW_DIRECTION);
    motor_lr = t + (int16_t)pid_output_pitch - (int16_t)pid_output_roll - (int16_t)(pid_output_yaw * YAW_DIRECTION);
    motor_lf = t - (int16_t)pid_output_pitch - (int16_t)pid_output_roll + (int16_t)(pid_output_yaw * YAW_DIRECTION);

    motor_rf = constrain(motor_rf, 1100, 2000);
    motor_rr = constrain(motor_rr, 1100, 2000);
    motor_lr = constrain(motor_lr, 1100, 2000);
    motor_lf = constrain(motor_lf, 1100, 2000);
  } else {
    motor_rf = motor_rr = motor_lr = motor_lf = 1000;
    pidReset();   // avoid I-term windup while disarmed / at idle
                  // tránh I-term windup khi đã ngắt kích hoạt / đang nghỉ
  }
}

// =============================================================================
//  Telemetry (1 Hz, human-readable)
// =============================================================================
void telemetry(uint32_t now) {
  if (now - last_telemetry_ms < 1000) return;
  last_telemetry_ms = now;
  Serial.printf("%s | VBAT:%.2fV | RC r:%d p:%d t:%d y:%d | ang p:%+.1f r:%+.1f | "
                "mot RF:%d RR:%d LR:%d LF:%d\n",
                armed ? "ARMED  " : "disarm ",
                vbat,
                rc[CH_ROLL], rc[CH_PITCH], rc[CH_THROTTLE], rc[CH_YAW],
                angle_pitch_deg, angle_roll_deg,
                motor_rf, motor_rr, motor_lr, motor_lf);
}

// =============================================================================
//  Serial console commands — bench use; the phone uses CRTP instead.
//    link <n>           switch transport profile (applied after a reboot)
//    sta <ssid> <pass>  set station-mode credentials (saved to NVS)
//    save | load | defaults
//    status
//  Lệnh console serial — dùng khi thử trên bàn; điện thoại dùng CRTP thay thế.
//    link <n>           đổi hồ sơ tầng truyền (áp dụng sau khi khởi động lại)
//    sta <ssid> <pass>  đặt thông tin đăng nhập chế độ station (lưu vào NVS)
//    save | load | defaults
//    status
// =============================================================================
void serialCommandPoll() {
  static char  line[96];
  static size_t len = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (len + 1 < sizeof(line)) line[len++] = c;
      continue;
    }
    line[len] = '\0';
    len = 0;
    if (line[0] == '\0') continue;

    char *argv[4];
    int   argc = 0;
    char *tok = strtok(line, " ");
    while (tok != NULL && argc < 4) { argv[argc++] = tok; tok = strtok(NULL, " "); }
    if (argc == 0) continue;

    if (armed && (strcmp(argv[0], "link") == 0 || strcmp(argv[0], "save") == 0 ||
                  strcmp(argv[0], "sta") == 0)) {
      Serial.println(F("refused while armed -- disarm first"));
    } else if (strcmp(argv[0], "link") == 0 && argc >= 2) {
      long m = atol(argv[1]);
      if (m >= 0 && m < LINK_MODE_COUNT) {
        nvs.putUChar("link", (uint8_t)m);
        pending_link_mode = (uint8_t)m;
        reboot_at_ms = millis() + 500;
        Serial.printf("link: profile %ld after reboot\n", m);
      } else {
        Serial.println(F("link: profile out of range"));
      }
    } else if (strcmp(argv[0], "sta") == 0 && argc >= 2) {
      strncpy(sta_ssid, argv[1], sizeof(sta_ssid) - 1);
      sta_ssid[sizeof(sta_ssid) - 1] = '\0';
      if (argc >= 3) {
        strncpy(sta_pass, argv[2], sizeof(sta_pass) - 1);
        sta_pass[sizeof(sta_pass) - 1] = '\0';
      }
      nvs.putString("ssid", sta_ssid);
      nvs.putString("pass", sta_pass);
      Serial.printf("sta: saved '%s' — now \"link 1\" for station mode\n", sta_ssid);
    } else if (strcmp(argv[0], "save") == 0) {
      cfgSave();
    } else if (strcmp(argv[0], "load") == 0) {
      cfgLoad();
    } else if (strcmp(argv[0], "defaults") == 0) {
      flightConfigDefaults();
      Serial.println(F("cfg: defaults restored (not saved)"));
    } else if (strcmp(argv[0], "status") == 0) {
      Serial.printf("link %u | mode %u | auto %u | armed %u | hover %u | heading %.1f | vbat %.2f\n",
                    link_mode, flight_mode, auto_state, armed ? 1 : 0,
                    hover_thrust, heading_deg, vbat);
    } else {
      Serial.println(F("commands: link <n> | sta <ssid> <pass> | save | load | defaults | status"));
    }
  }
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("=== XW_Z1_V2.0  ESP32 Drone Flight Controller ==="));

  motorInit();
  motorStop();

  if (!mpuInit()) {
    Serial.println(F("[MPU] WHO_AM_I mismatch — check SPI wiring (SCK18/MISO19/MOSI23/CS5)"));
    while (true) { delay(1000); }
  }
  Serial.println(F("[MPU] MPU6500 detected, SPI OK."));

  analogSetAttenuation(ADC_11db);          // full 0-3.3 V range on GPIO36
                                           // dải đầy đủ 0-3.3 V trên GPIO36
  pinMode(PIN_VBAT, INPUT);

  sbusInit();
  Serial.println(F("[SBUS] UART2 listening on GPIO4 (100000 8E2, inverted)."));

  nvs.begin("drone", false);
  cfgLoad();
  nvs.getString("ssid", sta_ssid, sizeof(sta_ssid));
  nvs.getString("pass", sta_pass, sizeof(sta_pass));
  linkInit();
  MavUdp.begin(MAVLINK_UDP_PORT);   // ArduPilot GCS socket (§13)
                                    // socket GCS ArduPilot (§13)

  mpuCalibrateGyro();
  batteryUpdate();
  Serial.printf("[PWR] VBAT = %.2f V\n", vbat);

  Serial.println(F("[ARM] SBUS : throttle LOW + yaw RIGHT (or CH5 high) 1 s to arm;"));
  Serial.println(F("            throttle LOW + yaw LEFT 1 s to disarm."));
  Serial.println(F("[ARM] CRTP : thrust at 0, then up, arms; thrust back to 0 disarms."));
  Serial.println(F("[MODE] manual | hold | auto(takeoff->hover->land) over CRTP port 7."));
  Serial.println(F("[MAV ] MAVLink v2 on UDP 14550 — Mission Planner / QGC / MAVProxy."));
  Serial.println(F("[NOTE] IMU only — hold keeps a thrust level, not an altitude."));
  Serial.println(F("=================================================="));

  loop_timer = micros();
}

// =============================================================================
//  MAIN LOOP  (250 Hz hard-timed)
// =============================================================================
void loop() {
  uint32_t now = millis();

  // ---- receivers ----
  sbusPoll();
  if (link_ready) linkPoll();
  mavlinkPoll();          // MAVLink GCS socket (§13) // socket GCS MAVLink

  // Ages are compared SIGNED. `now` is read before the receivers are polled, so
  // a packet handled this very loop can carry a timestamp 1 ms newer than `now`.
  // Unsigned, that wrapped to ~49 days old and tripped the failsafe for one
  // loop -- enough to cut the motors mid-flight.
  // Tuổi gói tin được so theo dấu SIGNED. `now` được đọc trước khi các bộ thu
  // được poll, nên một gói xử lý ngay trong vòng lặp này có thể mang mốc thời gian
  // mới hơn `now` 1 ms. Nếu so unsigned, giá trị đó quấn thành ~49 ngày tuổi và
  // kích hoạt failsafe trong một vòng lặp -- đủ để cắt động cơ giữa chuyến bay.
  bool sbus_ok = sbus_frame_ok && (int32_t)(now - sbus_last_frame_ms) < FAILSAFE_MS;
  bool cmd_ok  = cmd_valid     && (int32_t)(now - cmd_last_ms)        < FAILSAFE_MS;
  // Any CRTP packet proves the phone link is alive (PROTOCOL.md "Failsafe").
  // HOLD/AUTO/FOLLOW/RTH are driven by port-7 commands, not CommanderPackets,
  // so any traffic keeps them alive. MANUAL still needs a live CommanderPacket
  // stream, because that stream IS the setpoint -- other traffic alone would
  // leave the last thrust latched.
  // Bất kỳ gói CRTP nào cũng chứng minh liên kết điện thoại còn sống (PROTOCOL.md
  // "Failsafe"). HOLD/AUTO/FOLLOW/RTH được điều khiển bằng lệnh port-7, không phải
  // CommanderPacket, nên bất kỳ lưu lượng nào cũng giữ cho chúng sống. MANUAL vẫn
  // cần một luồng CommanderPacket trực tiếp, vì luồng đó CHÍNH LÀ setpoint -- chỉ
  // lưu lượng khác thôi sẽ để lực đẩy cuối cùng bị khóa giữ.
  bool crtp_ok  = (crtp_last_rx_ms != 0) &&
                  (int32_t)(now - crtp_last_rx_ms) < FAILSAFE_MS;
  bool phone_ok = (flight_mode == MODE_MANUAL) ? cmd_ok : crtp_ok;
  if (!cmd_ok) crtp_thrust_was_low = false;   // a reconnect must show zero thrust again
                                              // khi kết nối lại phải thấy lực đẩy bằng 0 lần nữa

  // The phone wins whenever it is live; otherwise fall back to the SBUS receiver
  // Điện thoại được ưu tiên khi còn kết nối; nếu không thì dùng lại bộ thu SBUS
  bool rx_ok     = phone_ok || sbus_ok;
  bool use_crtp  = phone_ok;

  // ---- failsafe ----
  if (!rx_ok && armed) {
    armed = false;
    missionStop();          // a dead link ends a mission too (§12)
                            // mất liên kết cũng kết thúc nhiệm vụ (§12)
    motorStop();
    pidReset();
    flight_mode = MODE_MANUAL;
    auto_state  = AUTO_IDLE;
    Serial.println(F("[FAILSAFE] RC signal lost — motors stopped."));
  }

  // ---- arm state ----
  if (rx_ok) armUpdate(now, use_crtp);

  // ---- critical battery: land the phone-flown modes, warn an RC/manual pilot ----
  // ---- pin nguy kịch: hạ cánh các chế độ bay bằng điện thoại, cảnh báo người lái RC/manual ----
  if (armed && vbat_crit) {
    if (use_crtp && flight_mode != MODE_MANUAL) {
      if (!(flight_mode == MODE_AUTO && auto_state == AUTO_LAND)) {
        Serial.println(F("[PWR] battery critical -- auto-landing"));
        crtpConsole("battery critical: landing");
        flightLand();
      }
    } else {
      static uint32_t last_low_msg_ms = 0;
      if (now - last_low_msg_ms > 2000) {
        last_low_msg_ms = now;
        Serial.printf("[PWR] battery critical %.2f V -- land now\n", vbat);
        crtpConsole("battery critical: land now");
      }
    }
  }

  // ---- attitude ----
  mpuReadRaw();
  attitudeUpdate();

  // ---- dead reckoning (feeds follow / return-to-launch) ----
  // "Holding" is the one state where we know the drone is not translating, so
  // it is the only anchor the velocity leak has. FOLLOW/RTH with no live target
  // is just as stationary as HOLD -- navSetpoints() leaves the tilt level -- so
  // it belongs in the same bucket, otherwise the estimate keeps drifting for no
  // reason while the drone sits there.
  // ---- tự dẫn đường quán tính (cấp liệu cho follow / return-to-launch) ----
  // "Holding" là trạng thái duy nhất ta biết drone không dịch chuyển, nên nó là
  // mỏ neo duy nhất cho khoản rò vận tốc. FOLLOW/RTH không có mục tiêu trực tiếp
  // cũng đứng yên như HOLD -- navSetpoints() để tư thế cân bằng -- nên thuộc cùng
  // nhóm, nếu không ước lượng sẽ cứ trôi vô cớ trong khi drone nằm yên ở đó.
  bool holding = (flight_mode == MODE_HOLD) ||
                 (flight_mode == MODE_AUTO && auto_state == AUTO_HOLD) ||
                 ((flight_mode == MODE_FOLLOW || flight_mode == MODE_RTH) && !tgt_valid);
  navUpdate(LOOP_TIME_US / 1000000.0f, holding && armed);
  if (tgt_valid && (int32_t)(now - tgt_last_ms) > NAV_TARGET_TIMEOUT_MS) {
    tgt_valid = false;          // phone reference went stale — stop chasing
                                // tham chiếu từ điện thoại đã cũ — ngừng đuổi theo
  }

  // ---- setpoints (manual / hold / auto / follow / return) ----
  flightSetpoints(now, use_crtp, sbus_ok);

  // ---- self-level: drive angle error to zero, then add the rate term ----
  // ---- tự cân bằng: đẩy sai số góc về 0, rồi cộng thành phần tốc độ góc ----
  float roll_err_deg  = angle_roll_deg  - sp_roll_deg;
  float pitch_err_deg = angle_pitch_deg - sp_pitch_deg;

  roll_level_adjust  = constrain(roll_err_deg  * LEVEL_SENSITIVITY, -200.0f, 200.0f);
  pitch_level_adjust = constrain(pitch_err_deg * LEVEL_SENSITIVITY, -200.0f, 200.0f);

  // The level term is SUBTRACTED. Tilted +5 deg past the target, the rate loop
  // must be asked for a NEGATIVE rate to come back. Adding it (the old code)
  // asked for more tilt in the same direction, and the drone flipped.
  // test/level_test.cpp pins this.
  // Thành phần cân bằng bị TRỪ đi. Khi nghiêng quá mục tiêu +5 độ, vòng tốc độ góc
  // phải được yêu cầu một tốc độ góc ÂM để quay về. Cộng nó vào (code cũ) lại đòi
  // nghiêng thêm cùng chiều, làm drone lật. test/level_test.cpp cố định điều này.
  roll_setpoint  = sp_roll_rate  - roll_level_adjust;
  pitch_setpoint = sp_pitch_rate - pitch_level_adjust;
  yaw_setpoint   = sp_yaw_rate;

  // ---- PID + mix ----
  pidCompute();

  if (armed && rx_ok) {
    motorMix(sp_throttle_us);
  } else {
    motor_rf = motor_rr = motor_lr = motor_lf = 1000;
    if (!armed) pidReset();
  }

  // ---- bench motor-test override ----
  // When a motor test is active it owns the four outputs, bypassing the mixer
  // entirely. Two ways it ends by itself: someone arms (the flight loop takes
  // over), or the GUI stops re-sending and the watchdog trips.
  // ---- phần ghi đè thử động cơ trên bàn ----
  // Khi một thử động cơ đang hoạt động, nó làm chủ bốn output, bỏ qua hoàn toàn
  // bộ trộn. Hai cách tự kết thúc: có người armed (vòng bay tiếp quản), hoặc
  // GUI ngừng gửi lại và watchdog kích hoạt.
  if (motor_test_active) {
    if (armed || (int32_t)(now - motor_test_last_ms) > MOTOR_TEST_TIMEOUT_MS) {
      motor_test_active = false;
      motorStop();
    } else {
      motor_rf = (int16_t)motor_test_m[0];
      motor_rr = (int16_t)motor_test_m[1];
      motor_lr = (int16_t)motor_test_m[2];
      motor_lf = (int16_t)motor_test_m[3];
    }
  }
  motorWrite(motor_rf, motor_rr, motor_lr, motor_lf);

  // ---- housekeeping ----
  batteryUpdate();
  logUpdate();          // stream any CRTP log block the app has started
                        // phát các khối log CRTP mà app đã khởi động
  telemetry(now);
  serialCommandPoll();

  // A link-profile change was requested over CRTP: give the reply time to
  // leave, then reboot so the radios come up in the new configuration.
  // Một thay đổi hồ sơ liên kết được yêu cầu qua CRTP: cho phản hồi kịp rời đi,
  // rồi khởi động lại để các radio lên với cấu hình mới.
  if (reboot_at_ms != 0 && (int32_t)(now - reboot_at_ms) >= 0) {
    Serial.printf("[LINK] switching to profile %u, rebooting\n", pending_link_mode);
    Serial.flush();
    ESP.restart();
  }

  // ---- hard 250 Hz pacing ----
  while (micros() - loop_timer < LOOP_TIME_US) { /* spin */ }
  loop_timer = micros();
}
