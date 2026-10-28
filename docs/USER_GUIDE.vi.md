# Hướng dẫn bay

*[English version](USER_GUIDE.md)*

Cách bay drone ESP32 XW_Z1 bằng ứng dụng Android và iOS: từng màn hình làm
gì, các tính năng an toàn hoạt động ra sao, và danh sách kiểm tra cho chuyến
bay đầu tiên.

Mới bắt đầu? Hãy cài firmware và ứng dụng trước — xem
**[INSTALL.md](INSTALL.md)** (tiếng Anh). Giao thức truyền dữ liệu nằm trong
**[../PROTOCOL.md](../PROTOCOL.md)**.

> **Trước hết:** bo mạch này **chỉ có một cảm biến chuyển động (MPU6500)**.
> Không có cảm biến áp suất (barometer), không có GPS, không có la bàn. Vì
> vậy **Hold giữ một mức lực đẩy, không giữ độ cao**, và vị trí của drone trên
> bản đồ chỉ là **ước tính**, sai lệch vài mét mỗi phút. Ứng dụng cũng ghi rõ
> điều này trên màn hình.

> **Về tên nút:** ứng dụng dùng tiếng Anh, nên tài liệu này giữ nguyên tên nút
> bằng tiếng Anh (in **đậm**) và giải thích bằng tiếng Việt bên cạnh.

---

## Mục lục

1. [Quy tắc an toàn](#s1)
2. [Bố cục màn hình](#s2)
3. [Kết nối](#s3)
4. [Màn hình Fly (bay)](#s4)
5. [CUT MOTORS (tắt động cơ)](#s5)
6. [Arm và disarm (mở khoá / khoá động cơ)](#s6)
7. [Những gì drone tự làm](#s7)
8. [Mission (bay theo GPS)](#s8)
9. [Indoor Map (bản đồ trong nhà)](#s9)
10. [Tune (chỉnh PID)](#s10)
11. [Bàn thử — giao diện bộ chỉnh PID trên Python](#s11)
12. [Setup (cài đặt)](#s12)
13. [Danh sách kiểm tra chuyến bay đầu tiên](#s13)
14. [Xử lý sự cố](#s14)
15. [Giới hạn đã biết](#s15)

---

<a id="s1"></a>
## 1. Quy tắc an toàn

1. **Biết nút CUT MOTORS ở đâu trước khi arm.** Đó là nút đỏ ở góc trên bên
   phải của mọi màn hình. Nút này tắt động cơ ngay lập tức — drone sẽ rơi.
2. **Những lần bay đầu: tháo cánh quạt trước, sau đó lắp cánh nhưng giữ hoặc
   buộc chặt drone.**
3. **Bay trên bãi cỏ, tránh xa người**, và luôn nhìn thấy drone.
4. **Theo dõi pin trên thanh tiêu đề.** Dưới 3,50 V chữ chuyển màu hổ phách,
   dưới 3,30 V chuyển màu đỏ. Hạ cánh khi thấy màu hổ phách.
5. **Không dựa vào Hold, Follow hay Return home để giữ drone đứng yên.** Chúng
   bị trôi (xem khung ở trên).
6. **Đổi mật khẩu Wi-Fi** (`WIFI_AP_PASS` trong sketch) trước khi bay ở nơi
   công cộng — bất kỳ ai kết nối vào mạng đều có thể điều khiển drone.

---

<a id="s2"></a>
## 2. Bố cục màn hình

Mọi màn hình đều có cùng một thanh tiêu đề, nên những thứ quan trọng không
bao giờ đổi chỗ.

![Màn hình Fly, đã kết nối, chưa arm](images/fly-connected.png)

*Bản thiết kế mẫu của màn hình Fly. Ứng dụng Android và iOS theo bố cục này;
một vài chi tiết nhỏ khác nhau tuỳ nền tảng.*

| Mục trên thanh tiêu đề | Ý nghĩa |
|---|---|
| **Fly · Mission · Indoor · Tune · Setup** | Năm mục chính. Mục đang mở có nền trắng. |
| **Wi-Fi / Link** | Màu xanh lá khi đã kết nối, kèm cường độ tín hiệu. Màu xám = chưa kết nối. |
| **Pin (vôn)** | Điện áp thật của cell pin. Màu chữ thường = ổn, **hổ phách dưới 3,50 V**, **đỏ dưới 3,30 V**. |
| **Ô chế độ** | MANUAL, HOLD, AUTO, FOLLOW hoặc RTH — chế độ drone đang báo về. |
| **ARMED / DISARMED** | **"● ARMED" màu hổ phách nghĩa là động cơ có thể quay.** Viền xám = an toàn. |
| **CUT MOTORS** | Nút dừng động cơ khẩn cấp. Xem [mục 5](#s5). |

Ô chế độ và ô ARMED xuất hiện khoảng một giây sau khi kết nối, khi drone gửi
báo cáo trạng thái đầu tiên.

**Chuyển giữa các mục**

- **Fly** là màn hình chính. Mission, Indoor Map và Tune được mở từ Fly; đóng
  chúng (hoặc chạm *Fly*) sẽ quay về Fly.
- **Không thể chuyển mục khi đang bay một nhiệm vụ (mission) hoặc khi
  auto-tune đang chạy** — ứng dụng sẽ báo lý do. Hãy hạ cánh, kết thúc nhiệm
  vụ, dừng bộ chỉnh, hoặc dùng CUT MOTORS trước. Điều này giúp bạn không vô
  tình đóng màn hình đang điều khiển drone.

---

<a id="s3"></a>
## 3. Kết nối

![Chưa kết nối](images/fly-not-connected.png)

Khi chưa kết nối, màn hình Fly hiển thị việc cần làm tiếp theo:

1. **Bật nguồn drone.**
2. **Wi-Fi hotspot (mặc định):** trong cài đặt điện thoại, kết nối vào mạng
   Wi-Fi **ESP-Drone** (mật khẩu `12345678`).
   **Bluetooth LE:** bật Bluetooth trên điện thoại.
3. **Chạm Connect (kết nối).**

Thẻ này cho biết ứng dụng sẽ dùng kiểu kết nối nào. Muốn đổi, hãy mở
**Setup**. Nút CUT MOTORS chuyển màu xám khi chưa kết nối — chưa có gì để
dừng.

| Kiểu kết nối | Android | iOS | Ghi chú |
|---|---|---|---|
| Wi-Fi hotspot (drone phát mạng `ESP-Drone`) | ✅ | ✅ | Đơn giản nhất. Mặc định. |
| Mạng Wi-Fi (drone kết nối vào router của bạn) | ✅ | ✅ | Cài đặt qua cổng serial: `sta <ssid> <pass>` |
| Bluetooth LE | ✅ | ✅ | |
| Bluetooth Classic | ✅ | ❌ | iOS chỉ cho phép với phụ kiện MFi |

Cách đổi kiểu kết nối phía drone nằm trong
[INSTALL.md § Connect](INSTALL.md#4-connect).

---

<a id="s4"></a>
## 4. Màn hình Fly (bay)

Đây là màn hình bạn cầm khi bay. Cần trái = ga (throttle) và xoay đầu (yaw),
cần phải = chúi (pitch) và nghiêng (roll) (kiểu Mode 2). Ở giữa:

| Thành phần | Chức năng |
|---|---|
| **Flight mode** | Manual · Hold · Auto. Follow và Return home được chạy từ màn hình Mission; khi một trong hai đang bật, không nút nào trong ba nút này sáng và ô chế độ sẽ hiển thị nó. |
| **Nút chính** | **Takeoff** (cất cánh) khi ở dưới đất, **Land** (hạ cánh) khi đã arm. Nút tự đổi. |
| **Dòng chú thích** | Nút chính sẽ làm gì, trong một dòng. |
| **ROLL · PITCH** | Độ nghiêng của drone (độ), đo bằng cảm biến của chính nó. |
| **SEQUENCE** | Chuỗi Auto đang ở bước nào: idle (chờ), takeoff (cất cánh), hold (giữ), land (hạ cánh). |
| **FLIGHT** | Thời gian từ lúc drone arm (phút:giây). Dùng như đồng hồ báo pin. |
| **More** | Những thứ không cần dùng liên tục (xem bên dưới). |

### Các chế độ bay

| Chế độ | Cần điều khiển | Drone làm gì |
|---|---|---|
| **Manual** (thủ công) | Điều khiển | Bạn tự bay. Đẩy ga từ 0 lên sẽ arm. |
| **Hold** (giữ) | Bị bỏ qua | Giữ thăng bằng, giữ hướng đầu, giữ một **mức lực đẩy** (không phải độ cao). |
| **Auto** (tự động) | Bị bỏ qua | Tăng dần lực đẩy để cất cánh → giữ → giảm dần để hạ cánh → disarm. |
| Follow (bám theo) | Bị bỏ qua | Nghiêng về phía mục tiêu mà điện thoại gửi (màn hình Mission). Vị trí là ước tính. |
| Return home (về nhà) | Bị bỏ qua | Nghiêng về lại điểm cất cánh. Vị trí là ước tính. |

**Takeoff** chuyển sang Auto: drone arm, tăng lực đẩy lên mức lực đẩy treo
(hover thrust) trong khoảng 2 giây, rồi giữ thăng bằng. **Land** giảm lực đẩy
trong khoảng 2,5 giây rồi disarm. Bấm Takeoff lần nữa khi đang lên hoặc đang
treo sẽ không làm gì — nó không bao giờ bắt đầu lại từ lực đẩy 0.

### Cần điều khiển mờ đi khi không có tác dụng

![Màn hình Fly, đã arm, chế độ Hold](images/fly-armed.png)

Ở chế độ Hold và Auto, hai cần điều khiển bị làm mờ vì drone bỏ qua chúng.
Chuyển sang **Manual** để tự điều khiển.

### Bảng More

![Bảng More](images/fly-more.png)

| Mục | Android | iOS |
|---|---|---|
| Thanh trượt **Hover thrust** (8 000–55 000) — dùng cho Takeoff và Hold | ✅ | ✅ |
| **Tilt control** — điều khiển pitch/roll bằng cách nghiêng điện thoại. Tắt cho đến khi bạn bật | ✅ | — |
| **Yaw on the left stick** — tắt nghĩa là cần yaw bị bỏ qua | — | ✅ |
| Nút **Arm / Disarm** (dùng khi thử trên bàn) | ✅ | qua *All flight modes…* |
| **All flight modes and sequence…** — danh sách đầy đủ các chế độ, gồm Follow / RTH | — | ✅ |
| **Stick output** — giá trị cần điều khiển đang gửi | ✅ | — |
| **Drone log** — thông báo từ drone | ✅ | ✅ |
| **Disconnect** (ngắt kết nối) | ✅ | — |
| Đèn LED ring / đèn pha / còi | chỉ khi drone có phần cứng này (bo này không có) | — |

Hover thrust là **lực đẩy**, không phải độ cao. Bắt đầu thấp (khoảng 30 000)
và tăng dần cho đến khi Takeoff nhấc drone lên gọn gàng. Giá trị phù hợp sẽ
thay đổi khi pin yếu dần.

---

<a id="s5"></a>
## 5. CUT MOTORS (tắt động cơ)

Nút đỏ ở góc trên bên phải của mọi màn hình. **Nút này dừng cả bốn động cơ
ngay lập tức và disarm. Drone sẽ rơi.** Dùng khi có bất cứ điều gì bất thường.

| Màn hình | Cách dùng |
|---|---|
| **Fly** | **Chạm hai lần trong vòng 3 giây.** Lần chạm đầu nút chuyển màu cam và hiện *TAP AGAIN* (chạm lần nữa); lần chạm thứ hai tắt động cơ. Nếu không chạm lần nữa, nút trở lại màu đỏ. |
| **Mission, Tune** | **Chạm một lần** là tắt ngay. Ở màn hình Tune, nút này cũng dừng bộ tự chỉnh (auto-tuner). |

Màn hình Fly cần chạm hai lần vì ngón tay bạn đang đặt trên cần điều khiển
ngay cạnh nút; một cú chạm nhầm không được làm rơi drone. Mission và Tune được
dùng khi drone ở xa hơn, nơi tốc độ quan trọng hơn.

> Nút trông giống nhau trên mọi màn hình nhưng cách xác nhận khác nhau. Hãy
> tập dùng trên bàn (đã tháo cánh) ở từng màn hình trước khi bay.

---

<a id="s6"></a>
## 6. Arm và disarm (mở khoá / khoá động cơ)

**Armed** nghĩa là động cơ có thể quay. Thanh tiêu đề hiện **● ARMED** màu hổ
phách.

### Các cách arm

| Cách | Khi nào hoạt động |
|---|---|
| Nút **Takeoff** | Pin ổn. Arm và cất cánh. |
| **Đẩy ga từ 0 lên** (điện thoại, Manual) | Ứng dụng phải thấy ga ở mức 0 trước. Điện thoại kết nối khi ga đang cao sẽ **không** làm cánh quạt quay. |
| Nút **Arm** | Chỉ khi đang nghỉ: Manual với ga ở 0, hoặc Auto trước khi cất cánh. Bị từ chối ở Hold, Follow và RTH, vì động cơ sẽ nhảy thẳng lên lực đẩy treo. |
| **Tay điều khiển RC** | Ga thấp + yaw sang phải trong 1 giây, hoặc công tắc arm (CH5) bật. |

**Mọi cách đều bị từ chối nếu pin dưới 3,30 V.** Nhật ký drone sẽ ghi
*"arm refused: low battery"* (từ chối arm: pin yếu).

### Các cách disarm

- **Land** — giảm dần rồi disarm.
- **Kéo ga về 0** (điện thoại, Manual).
- Nút **Disarm** (bảng More trên Android, *All flight modes* trên iOS).
- **Tay điều khiển RC:** ga thấp + yaw sang trái trong 1 giây, hoặc tắt công
  tắc arm.
- **CUT MOTORS** — tức thì.

---

<a id="s7"></a>
## 7. Những gì drone tự làm

### Mất kết nối (failsafe)

Nếu drone không nhận được gì trong **0,5 giây** khi đang arm, nó sẽ **tắt
động cơ**, disarm và trở về Manual.

- Ở **Manual**, "nhận được gì đó" nghĩa là luồng tín hiệu cần điều khiển từ
  điện thoại (hoặc tín hiệu RC hợp lệ). Các tin nhắn khác không tính, vì chính
  luồng cần điều khiển là lệnh điều khiển.
- Ở **Hold, Auto, Follow và RTH**, mọi tin nhắn từ điện thoại đều được tính.
- Với **bộ thu RC**, các khung dữ liệu mà bộ thu tự đánh dấu là *failsafe*
  hoặc *lost* (mất) sẽ bị bỏ qua, nên khi bộ thu mất tín hiệu từ tay điều
  khiển, drone cũng tắt động cơ.

### Pin yếu

Nếu pin ở dưới **3,30 V liên tục trong 3 giây**:

- Ở **Hold, Auto, Follow, RTH** (điện thoại điều khiển): drone **tự hạ cánh**,
  và từ chối đổi chế độ cho đến khi đã disarm.
- Ở **Manual** hoặc khi dùng tay điều khiển RC: nhật ký drone cảnh báo
  *"battery critical: land now"* (pin cạn: hạ cánh ngay) mỗi 2 giây. Hãy hạ
  cánh.

Điện áp tụt nhanh trong chốc lát khi bạn đẩy ga mạnh sẽ không kích hoạt việc
này — điện áp được làm mượt và phải ở mức thấp liên tục 3 giây.

### Bị từ chối khi đang arm

Drone sẽ không **lưu cài đặt vào bộ nhớ** hoặc **đổi kiểu kết nối** khi đang
arm — cả hai đều làm bo mạch tạm dừng hoặc khởi động lại.

---

<a id="s8"></a>
## 8. Mission (bay theo GPS)

![Thiết kế màn hình Mission](images/mission.png)

*Bản thiết kế tham khảo. Cách sắp xếp nút hơi khác trên mỗi nền tảng — xem
các bảng bên dưới.*

GPS và la bàn của **điện thoại** xác định điểm cất cánh và hướng. Drone không
có GPS; nó **ước tính** vị trí của mình từ cảm biến chuyển động, và ước tính
này bị trôi. Hãy coi Follow và Return home là "đi về phía đó một cách tương
đối", không bao giờ là chính xác.

**Trước khi cất cánh:** đứng tại điểm cất cánh khi đã có tín hiệu GPS tốt,
hướng điện thoại theo hướng mũi drone, rồi mới cất cánh. Ứng dụng sẽ từ chối
cất cánh nếu không có tín hiệu GPS hoặc la bàn.

### Android

| Nút | Chức năng |
|---|---|
| **Launch** | Ghi lại điểm và hướng cất cánh, arm, cất cánh. |
| **Mission** | Bay lần lượt theo danh sách điểm đến (waypoint). |
| **Go to here** | Bay đến điểm bạn đã chạm (chế độ chạm: go-to). |
| **Pause / Resume** | Ngừng gửi mục tiêu; drone đứng giữ. / Tiếp tục. |
| **End mission** | Ngừng điều hướng; drone **giữ** tại chỗ. |
| **Return home** | Bay về phía điểm cất cánh. |
| **Land** | Hạ cánh và disarm. |
| **Map: …** | Đổi bản đồ vệ tinh / đường phố / địa hình. |
| **Tap: …** | Chạm vào bản đồ để thêm waypoint hay để đặt điểm go-to. |
| ↑ ↓ 🗑 (biểu tượng) | Di chuyển lên/xuống hoặc xoá waypoint đang chọn. |
| 📁 💾 (biểu tượng) | Mở / lưu tệp nhiệm vụ (`.waypoints` của Mission Planner). |

### iOS

| Điều khiển | Chức năng |
|---|---|
| **MISSION · GUIDED · RTL** | Kiểu điều hướng mà *Start mission* sẽ chạy. |
| **Arm + ALIGN** | Ghi lại điểm và hướng cất cánh, arm. |
| **Start mission** | Bắt đầu kiểu điều hướng đã chọn. |
| **Pause / Resume** | Ngừng gửi mục tiêu; drone đứng giữ. / Tiếp tục. |
| **End mission** | Ngừng điều hướng và trả quyền điều khiển về cần điều khiển. |
| Biểu tượng thư mục | Mở tệp `.waypoints`. |
| Biểu tượng thùng rác | Xoá kế hoạch bay. |

Nhấn giữ các nút chỉ có biểu tượng (Android) hoặc dùng VoiceOver (iOS) để
nghe tên của chúng.

**Các vùng cấm bay và hành lang được vẽ trên bản đồ này, nhưng không sửa ở
đây.** Bản đồ chỉ đọc với chúng để kế hoạch bay không thể đổi giữa chuyến bay;
trình sửa nằm ở trang [Indoor Map](#s9). Nếu một waypoint bị từ chối ("inside
keep-out zone") hoặc một lần giữ bắt đầu ("keep-out zone entered - holding"),
hình màu đỏ bạn thấy chính là lý do.

---

<a id="s9"></a>
## 9. Indoor Map (bản đồ trong nhà)

![Màn hình Indoor Map](android-app.svg)

*Màn hình Indoor Map, dạng sơ đồ. Ứng dụng iOS
([ios-app.svg](ios-app.svg)) có các công cụ tương tự theo bố cục riêng.*

Mục **Indoor** là trình sửa chú thích bản đồ — nơi bạn cho drone biết những
thứ bản đồ không tự biết: chỗ nào không được bay, chỗ nào được đi xuyên qua
vùng cấm bay, và chỗ nào không khí không ổn định. Mọi thứ bạn vẽ ở đây được
chia sẻ với trang [Mission](#s8): các vùng sửa ở đây chính là các vùng mà trình
chạy nhiệm vụ tuân theo.

### Các chế độ

Mỗi lúc chỉ một chế độ được bật (chế độ hiện tại có nền tô):

| Chế độ | Cách vẽ | Tạo ra |
|---|---|---|
| **Select** (chọn) | Chạm vào hình để chọn; kéo các tay cầm hoặc thân hình | Sửa những gì đã có trên bản đồ |
| **Keep-out** (cấm bay) | Kéo trên bản đồ để vẽ hình chữ nhật | Vùng màu đỏ drone sẽ không bay vào |
| **Wind** (gió) | Kéo trên bản đồ để vẽ hình chữ nhật | Vùng cần xem xét kỹ khi đến (không khí không ổn định) |
| **Corridor** (hành lang) | Chạm một đầu, rồi chạm đầu kia | Cổng màu xanh lá drone được đi xuyên qua |
| **Go to** (đi đến) | Chạm một điểm | Mục tiêu cho chế độ GUIDED — khởi chạy từ trang Mission |

Chế độ vẽ là chỉ-vẽ: khi một chế độ vẽ đang bật, chạm vào bản đồ sẽ vẽ (hoặc
không làm gì) — không chọn hình và không thả waypoint. Hai ngón vẫn thu phóng.
Khi đặt xong, hình được làm gọn theo **góc 5° và số mét nguyên**.

### Sửa một hình

Chuyển sang **Select**, rồi chạm vào hình. Hình sẽ mang các tay cầm sửa:

| Tay cầm | Ở đâu | Làm gì |
|---|---|---|
| ✕ đỏ | góc đầu tiên | xoá hình (chạm, không kéo) |
| ✛ / ↻ | góc thứ hai | di chuyển đầu hành lang / xoay vùng |
| ↖↘ | góc thứ ba | phóng to / thu nhỏ vùng |
| thân hình | bất kỳ đâu trên hình | kéo cả hình đi |

**Delete selected** (xoá đang chọn), **Clear all** (xoá hết) và **Center**
(căn giữa) nằm phía trên hàng ô; hàng ô liệt kê mọi hình ("Keep-out 1",
"Corridor 1", …) và mọi waypoint ("WP 1", …) — chạm một ô để chọn và căn bản
đồ vào đó.

Sơ đồ màn hình của hai ứng dụng: [android-app.svg](android-app.svg) ·
[ios-app.svg](ios-app.svg).

---

<a id="s10"></a>
## 10. Tune (chỉnh PID)

Dùng để chỉnh bộ ổn định khi thử trên bàn. **Auto-tune làm cánh quạt quay.
Buộc drone vào giá đỡ hoặc giữ bằng dây.**

| Thành phần | Chức năng |
|---|---|
| **Axis (trục)** | Roll, pitch hoặc yaw — chỉnh từng trục một. |
| **Biểu đồ** | Góc thực tế so với đầu ra bộ điều khiển, và các thành phần P / I / D. Tạm dừng và xoá bằng các nút biểu tượng. |
| **Hệ số (gains)** | Sửa P, I và D. Drone giới hạn mọi giá trị trong khoảng an toàn và từ chối số không hợp lệ; ô nhập hiển thị giá trị drone thực sự đã lưu. |
| **Save / Load / Defaults** | Lưu hệ số vào bộ nhớ drone (chỉ khi chưa arm), đọc lại, hoặc khôi phục mặc định (chưa lưu cho đến khi bạn bấm Save). |
| **Start auto-tune** | **Nhấn và giữ** (2 giây trên Android, 1,5 giây trên iOS) để bắt đầu. |
| **Stop tuner** (Android) | Dừng sau lượt thử hiện tại. |
| **CUT MOTORS** | Tắt động cơ **và** dừng bộ tự chỉnh, chạm một lần. |

---

<a id="s11"></a>
## 11. Bàn thử — giao diện bộ chỉnh PID trên Python

![Giao diện bộ chỉnh PID Python](pid-tune-gui.svg)

*Cửa sổ trên laptop (`tools/pid-tune/tuner_gui.py`), dạng sơ đồ.*

Cùng các công cụ chỉnh PID, cộng thêm các bài thử trên bàn, chạy trên laptop
kết nối vào mạng của drone. Xem
[tools/pid-tune/README.md](../tools/pid-tune/README.md) để biết cách khởi chạy.

### Thử động cơ

Chạy từng động cơ một để kiểm tra dây nối và cánh quạt sau khi sửa chữa.
**Chỉ khi chưa arm, tránh xa cánh quạt** — cánh quạt đang quay có thể cắt
thương.

| Thành phần | Chức năng |
|---|---|
| **Speed** (tốc độ) | Dùng chung cho cả bốn động cơ. 1000 là chế độ chờ (idle); 1200 là quay nhẹ. |
| **Run** (chạy, từng động cơ) | Chạy **động cơ đó thôi** ở tốc độ đã đặt; các động cơ còn lại ở chế độ chờ. |
| **Stop** (dừng, từng động cơ) | Dừng động cơ đó; các động cơ khác giữ nguyên trạng thái. |
| **STOP ALL** | Kết thúc toàn bộ bài thử ngay lập tức. |

Giao diện gửi lại lệnh mỗi 150 ms khi có động cơ đang chạy — firmware tự dừng
bài thử sau 300 ms im lặng, và đóng cửa sổ cũng gửi lệnh tắt. EMERGENCY cũng
kết thúc nó. Nếu trạng thái báo bài thử không được xác nhận, drone đang arm —
hãy disarm trước.

---

<a id="s12"></a>
## 12. Setup (cài đặt)

| | Android | iOS |
|---|---|---|
| Mở ra | Màn hình cài đặt: kiểu kết nối, điều khiển, độ nhạy | Một menu: **Link — Wi-Fi or Bluetooth** (kết nối), hoặc **Sticks and sensitivity** (cần điều khiển và độ nhạy) |

Sau khi đổi kiểu kết nối, hãy kết nối lại. Để đổi kiểu kết nối ở **phía
drone**, dùng cổng serial (`link <n>`) — xem
[INSTALL.md](INSTALL.md#verify-the-flash).

---

<a id="s13"></a>
## 13. Danh sách kiểm tra chuyến bay đầu tiên

Làm đủ mọi bước. Làm bước 1–6 **khi đã tháo cánh quạt.**

1. **Sạc đầy pin.** Thanh tiêu đề sẽ hiện khoảng 4,1 V khi kết nối.
2. **Kết nối.** Ô chế độ và DISARMED xuất hiện trong một, hai giây.
3. **Nghiêng drone bằng tay.** ROLL và PITCH phải thay đổi theo: hạ bên phải
   xuống → roll thay đổi; chúi mũi xuống → pitch thay đổi. Nếu một số thay đổi
   sai chiều, **dừng lại** và sửa thiết lập dấu trong sketch (`GYRO_*_SIGN`,
   `ACC_*_SIGN`) trước khi làm tiếp.
4. **Arm, rồi CUT.** Chạm Takeoff (hoặc đẩy ga từ 0 lên), thấy **● ARMED**,
   rồi dùng CUT MOTORS. DISARMED phải hiện lại ngay. Làm lại trên màn hình
   Mission và Tune để làm quen với kiểu chạm một lần.
5. **Thử mất kết nối.** Arm, rồi tắt Wi-Fi trên điện thoại. Trong vòng nửa
   giây, động cơ phải dừng.
6. **Chiều quay động cơ.** Đã tháo cánh, arm, tăng ga một chút: RF và LR quay
   **theo chiều kim đồng hồ**, RR và LF **ngược chiều kim đồng hồ** (nhìn từ
   trên xuống).
7. **Lắp cánh, giữ hoặc buộc chặt drone.** Tăng ga từ từ. Nghiêng drone bằng
   tay — nó phải đẩy ngược về thăng bằng, không nghiêng thêm. Nếu nó đẩy *ra
   xa* vị trí thăng bằng, hãy CUT và dừng.
8. **Lần treo đầu tiên.** Ở nơi thoáng: Takeoff với hover thrust thấp, một
   ngón tay sẵn trên CUT MOTORS. Tăng hover thrust từng chút (bảng More) cho
   đến khi drone nhấc lên gọn gàng. Hạ cánh.

Các hệ số PID và hai thiết lập dấu điều hướng (`NAV_FWD_SIGN`,
`NAV_LEFT_SIGN`) vẫn là giá trị mặc định. Hãy chuẩn bị chỉnh trước khi bay
mượt, và chỉ thử Follow / Return home sau khi đã treo ổn định.

---

<a id="s14"></a>
## 14. Xử lý sự cố

| Hiện tượng | Nguyên nhân có thể và cách xử lý |
|---|---|
| Ô chế độ / ARMED không bao giờ hiện | Báo cáo trạng thái không đến. Kết nối lại; kiểm tra firmware đã là bản mới. |
| Pin hiện "— V" | Chưa kết nối, hoặc chưa có báo cáo. |
| Nhật ký ghi *arm refused: low battery* | Pin dưới 3,30 V (hoặc bo đang cắm USB mà không có pin — đặt `VBAT_ARM_CHECK` bằng 0 khi thử trên bàn). |
| Đẩy ga lên không arm | Kéo ga hoàn toàn về 0 trước, rồi mới đẩy lên. |
| Nút Arm bị từ chối | Chỉ hoạt động khi đang nghỉ — Manual với ga ở 0, hoặc Auto trước khi cất cánh. Dùng Takeoff thay thế. |
| Takeoff bị từ chối | Pin yếu, hoặc drone đang bay ở chế độ khác. |
| Động cơ đột ngột tắt khi đang bay | Mất kết nối 0,5 giây, hoặc bộ thu RC báo failsafe. Đứng gần hơn; xem tín hiệu Wi-Fi trên thanh tiêu đề. |
| Drone tự hạ cánh | Pin dưới 3,30 V trong 3 giây. |
| Không chuyển được mục | Một nhiệm vụ hoặc auto-tune đang chạy — kết thúc nó, hạ cánh, hoặc CUT trước. |
| Lưu / đổi kết nối bị từ chối | Drone đang arm. Disarm trước. |
| Drone nghiêng thêm khi bị đẩy (đã lắp cánh) | Thiết lập dấu cảm biến sai. Dừng lại và sửa trước khi bay. |
| Hold bay lên hoặc tụt xuống | Bình thường — Hold là một mức lực đẩy, không phải độ cao. Chỉnh hover thrust. |
| Follow / Return home đi sai hướng | Ước tính vị trí bị trôi, hoặc `NAV_FWD_SIGN` / `NAV_LEFT_SIGN` sai. Thử ở độ cao thấp. |
| Không thấy Bluetooth Classic trên iPhone | Bình thường — dùng Wi-Fi hoặc Bluetooth LE. |

---

<a id="s15"></a>
## 15. Giới hạn đã biết

- **Không có độ cao, không có vị trí.** Chỉ có cảm biến chuyển động. Xem khung
  ở đầu trang.
- **CUT MOTORS xác nhận khác nhau** ở Fly (chạm hai lần) và Mission / Tune
  (chạm một lần). Xem [mục 5](#s5).
- **Setup chưa được thiết kế lại.** Nó vẫn mở các màn hình cài đặt cũ.
- **Các màn hình ứng dụng thiết kế lại còn mới.** Hãy build chúng trong
  Android Studio và Xcode và kiểm tra trên điện thoại trước khi tin dùng khi
  bay.
- **Hệ số PID và thiết lập dấu điều hướng là mặc định.** Cần chỉnh từ các
  chuyến bay thật.
