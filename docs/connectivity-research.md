# ESP32-WROOM-32 connectivity: transports, boot selection, platform limits

Research report for this firmware and its two phone clients. Surveyed 2026-10-28
against primary vendor and platform documentation; every claim below carries a
source and a confidence rating. Where nothing survived verification, it says so
rather than guessing.

The question was: how do the four transports compare for a 50 Hz bidirectional
control stream, how should the firmware's boot-time `link <n>` selector be made
robust, and what do Android and iOS actually permit? **Control latency and link
robustness are the driving constraint** — a delayed or dropped commander packet
trips the 500 ms failsafe and the drone falls out of the sky, so this is a
flight-safety question, not a quality-of-service one.

---

## 1. The headline

**All four transports share one radio.** The ESP32-WROOM-32E/32UE has a single
RF chain (ESP32-D0WD-V3 → RF matching → one antenna) serving WiFi
802.11b/g/n, Bluetooth BR/EDR 4.2 and BLE. No transport gets a private radio;
everything below follows from that.

**One premise in the original brief was wrong.** WiFi and Bluetooth Classic are
*not* mutually exclusive on the ESP32. They coexist by time-division
multiplexing that one RF path (`CONFIG_ESP_COEX_SW_COEXIST_ENABLE`). A `link`
selector may legitimately advertise both — the cost is airtime, not impossibility.

**Mode 4 is on a path Espressif itself rates unstable.** Every
SOFTAP + BT/BLE connected combination is rated `C1` "supported but the
performance is unstable" in the vendor coexistence matrix. STA + BT/BLE is rated
stable (`Y`). Since mode 4 is `WIFI_AP_BLE`, it is the one combination to warn
about in the UI; mode 5 (`WIFI_STA_BLE`) is not.

---

## 2. Transport ranking for a 50 Hz control link

| rank | mode | why |
| --- | --- | --- |
| 1 | `WIFI_AP` (0), `WIFI_STA` (1) | full radio to WiFi, best link budget, no coexistence tax |
| 2 | `BT_SPP` (2) | full radio to BT, no coexistence tax; weaker link budget; **unusable on iOS without MFi** |
| 3 | `BLE GATT` (3) | weakest link budget, 19-byte segmentation overhead; the only iOS-capable BT option |
| 4 | `WIFI_AP_BLE` (4), `WIFI_STA_BLE` (5) | coexistence halves airtime on >100 ms superframes; mode 4's pair is vendor-rated `C1` |

**Confidence: medium.** This ranking is a synthesis from verified RF, link-budget
and coexistence specs. It is *not* backed by measured RTT/jitter/loss figures —
no measured-latency claim survived verification in this survey. Measure it on
this hardware before treating the order as settled (see §6).

### 2.1 Link budget (high confidence)

From the module datasheet, at the antenna port:

| | TX | RX sensitivity | link budget |
| --- | --- | --- | --- |
| WiFi 802.11b | 19.5 dBm | −97 dBm | **~116.5 dB** |
| BLE (max TX) | +9 dBm | −93 dBm | ~102 dB |
| BLE (default 0 dBm) | 0 dBm | −93 dBm | **~93 dB** |
| BR/EDR | −8..+9 dBm | −89 dBm | ~93–98 dB |

WiFi leads by roughly 9.5–18.5 dB, more at Bluetooth's default TX power. The
qualification that matters: at high WiFi MCS rates (g 54 Mbps, n MCS7) the
picture partially inverts (~89 dB). But a 50 Hz ~16-byte commander stream lives
in the low-rate regime, where WiFi's advantage holds.

For a link that must not drop a packet at 50 Hz, ~20 dB of extra budget is the
single biggest lever available — bigger than any protocol choice.

### 2.2 Coexistence cost (high confidence)

When WiFi and BLE are both *connected* (mode 5's pair), each gets **50 % of a
coexistence period that is TBTT-anchored and longer than 100 ms**. A 20 ms
control packet therefore shares half the airtime on ~100 ms superframes.

Espressif's own ESP-FAQ calls degradation under concurrent load *"expected
behavior… services with strict real-time requirements experience performance
degradation."* It is not a configuration bug and cannot be tuned away.

Practical consequence: **modes 4 and 5 inherently tax the control stream**, and
mode 4 additionally sits in the vendor's unstable matrix cell. If the firmware
offers them, the app should say what it costs.

### 2.3 Hardware limits

One RF path, one antenna, shared across all three radios. Nothing in software
creates a second one. Antenna detuning from the airframe and battery is the
usual field failure and is not modelled here.

---

## 3. Boot-time transport selection (the NVS `link` key)

**Confidence: high.** This is what NVS is documented to be for — "a moderate,
relatively stable set of small values — such as device configuration,
calibration data or state flags."

### 3.1 The pattern

```
namespace "link", key "mode", uint8
pre-initialise the output to 0 (WIFI_AP) BEFORE the nvs_get_* call
nvs_get_u8(...) -> keep 0 on ESP_ERR_NVS_NOT_FOUND
validate the range 0..5 on read; anything else falls back to 0
```

`nvs_get_*` **never modifies `out_value` on error**, so pre-initialising to the
safe default and ignoring a `NOT_FOUND` return is correct and complete. The
default is soft-AP, because that is the only mode a phone can reach with no
prior configuration.

### 3.2 The failure that bites on firmware updates

`nvs_flash_init()` can return `ESP_ERR_NVS_NO_FREE_PAGES` or
`ESP_ERR_NVS_NEW_VERSION_FOUND` after a partition-table or format change. The
documented recovery is to **erase the NVS partition and re-init** — which
destroys the saved `link` key.

So: **never assume the value survives a firmware update.** The fallback path is
not defensive programming, it is the normal path across an OTA or a reflash.

### 3.3 Reboot-to-apply is correct

The Bluetooth controller's mode is one of exactly three — 1 BLE-only,
2 Classic-only, 3 Dual (needed to offer both SPP and BLE) — fixed at
`esp_bt_controller_init()`, and **cannot be changed by re-enabling**. The
documented workaround is `esp_bt_controller_disable()` then enable with the new
mode, and ESP-IDF issues report runtime switching failing even then.

Writing NVS and rebooting is therefore the robust design, not a shortcut. A hot
radio-down/radio-up path would need full controller deinit + reinit and is less
reliable than a reboot. Keep reboot-to-apply.

### 3.4 Discovery before any link exists

The phone cannot ask the drone which transport it is offering, because asking
requires a link. So the client must try in a fixed order and be explicit in the
UI about which it is attempting. The safe order is the ranking in §2, with BLE
placed where the platform requires it (see §4).

### 3.5 This firmware, checked against §3

`linkInit()` and the serial `link <n>` handler were read directly, which also
settles the `nvs_commit` question in §5:

| §3 requirement | `Esp32_drone.ino` | |
| --- | --- | --- |
| pre-init to mode 0 | `linkInit()`: `nvs.getUChar("link", LINK_MODE_WIFI_AP)` supplies the default | ✓ |
| validate the range | `if (link_mode >= LINK_MODE_COUNT) link_mode = LINK_MODE_WIFI_AP` | ✓ |
| persist the write | `nvs.putUChar("link", ...)` — the Arduino `Preferences` wrapper calls `nvs_set_u8` **then `nvs_commit` unconditionally** (`Preferences.cpp:125-136` in the ESP32 core) | ✓ |
| apply by reboot | `reboot_at_ms = millis() + 500` after the write | ✓ |
| | and the handler **refuses while armed** (`"refused while armed -- disarm first"`) | extra |

So the `nvs_commit` claim in §5 is moot here: the firmware never calls
`nvs_set_*` directly, and `Preferences::putUChar` commits internally. Nothing to
fix.

The gap this leaves is §3.4 — discovery — and §2's missing measurements.

---

## 4. Platform constraints

### 4.1 Android (high confidence)

**Bluetooth permissions, API 31+.** Access is split into three separately
runtime-approved permissions, each of which must be requested explicitly before
it works:

- `BLUETOOTH_SCAN` — to look for devices
- `BLUETOOTH_ADVERTISE` — to be discoverable
- `BLUETOOTH_CONNECT` — to talk to devices, including *new* BLE peripherals,
  not only already-paired ones

BLE scan/connect and SPP connect each need their own granted permission before
any link exists. This model has been stable since Android 12 (2021).

The app's `minSdk 21` also implies legacy `BLUETOOTH`/`BLUETOOTH_ADMIN`
handling on API 21–30 — **that legacy half could not be verified in this
survey** (see §6).

**WiFi control is closed, Android 10+.** An app targeting API 10+ cannot
enable or disable the WiFi radio — `WifiManager.setWifiEnabled()` always returns
false — and cannot list, add or modify configured networks
(`getConfiguredNetworks()` empty, `addNetwork()`/`updateNetwork()` return −1,
`enableNetwork()`/`reconnect()`/`disconnect()` return false).

So the app cannot turn WiFi on, and cannot silently force the phone onto the
drone's soft-AP. It must prompt the user, and drive the join with:

- `WifiNetworkSpecifier` via `NetworkRequest` — instant, local-only connection.
  **API 29+ only.**
- `WifiNetworkSuggestion` — where the network should be considered for internet.

On `minSdk 21` devices running API 21–28 the specifier class does not exist, so
that path needs a fallback UX. Carrier-app and Device Owner exemptions do not
apply to a third-party drone app.

**App Standby is not the risk.** An app with a foreground activity or foreground
service is not idle, so its network access is not subject to standby
restrictions. A 50 Hz stream is sustainable while the user is actively flying.
The risk window is *backgrounding*, not active use — and OEM battery skins can
be stricter than stock.

### 4.2 iOS — **not covered**

**The iOS angle of this survey produced no surviving verified claims.** Every
iOS question asked — MFi/ExternalAccessory as the bar on Bluetooth Classic SPP,
CoreBluetooth MTU/ATT negotiation against the 19-byte CRTPUP segmentation and
20-byte raw characteristic, BLE background modes and state restoration, the
local-network permission for UDP, mDNS/multicast discovery restrictions, iOS 15+
landscape controller behaviour — is **unverified**.

The only iOS fact on hand is the app's own on-screen statement that SPP is
unavailable. That is consistent with the well-known MFi requirement for the
ExternalAccessory framework, but it has **not** been confirmed against Apple
documentation in this run.

Do not treat §2's ranking as settled for iOS. The known structural facts are
that SPP is not an option there (so BLE GATT is the only Bluetooth path) and
that the client targets iOS 15+; everything else needs its own verification
pass before shipping.

---

## 5. What was refuted

Two claims were adversarially checked and **refuted**. Do not repeat them:

1. **"A `link <n>` handler that omits `nvs_commit()` silently loses the
   setting."** Refuted on a 1–2 vote. Do not assert either way without reading
   the commit call in the handler directly.
2. **"An app targeting API 29+ must hold `ACCESS_FINE_LOCATION` for BLE
   scanning."** Refuted on a 1–2 vote.

## 6. Open questions

These are the real gaps, in priority order:

1. **Measured latency, jitter and packet loss at 50 Hz** for all four transports
   on this hardware under realistic 2.4 GHz interference — including whether
   modes 4/5 measurably trip the 500 ms failsafe. The §2 ranking is medium
   confidence precisely because this is missing.
2. **The iOS constraints** (§4.2) — a whole platform's worth of unverified
   claims.
3. **Hot transport switch** — is full `esp_bt_controller` disable/deinit +
   reinit without a reboot reliable in practice? The documented known issue says
   no; field experience may differ.
4. **Doze and OEM battery skins** — how they affect a 50 Hz UDP/BLE stream if
   the screen blanks or the app is briefly backgrounded mid-flight. Both
   supporting claims were unverified this run.

---

## 7. Confidence and freshness

Verified claims rest on primary vendor and platform documentation — Espressif
datasheets, ESP-IDF and ESP-FAQ, Android developer docs — fetched as of
**2026-10-28**. The coexistence matrix, the link-budget tables and the NVS
behaviour are stable. **Permission and background-execution behaviour is
time-sensitive** and should be re-checked against current Android and iOS
releases before shipping.

Several verification passes failed on infrastructure errors (rate limiting), so
contradiction sweeps were done by direct source inspection rather than broad web
search. Four claims are neither confirmed nor refuted: `neverForLocation` on
`BLUETOOTH_SCAN` filtering some BLE beacons; the Android 10-and-earlier legacy
Bluetooth + `ACCESS_FINE_LOCATION` regime for `minSdk 21`; Doze suspending
network access for a 50 Hz stream; and the claim that a screen-on handheld
controller never enters Doze.

## Sources

Espressif

- [ESP32-WROOM-32E / 32UE datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf)
- [ESP-IDF RF Coexistence guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/coexist.html)
- [ESP-FAQ: coexistence](https://docs.espressif.com/projects/esp-faq/en/latest/software-framework/coexistence.html)
- [ESP-IDF NVS flash API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/nvs_flash.html)
- [ESP-IDF Bluetooth controller / VHCI](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/controller_vhci.html)
- [esp-idf#2619](https://github.com/espressif/esp-idf/issues/2619)

Android

- [Bluetooth permissions](https://developer.android.com/develop/connectivity/bluetooth/bt-permissions)
- [Doze and App Standby](https://developer.android.com/training/monitoring-device-state/doze-standby)
- [Android 10 privacy changes: WiFi](https://developer.android.com/about/versions/10/privacy/changes)
- [WifiNetworkSpecifier](https://developer.android.com/reference/android/net/wifi/WifiNetworkSpecifier)
- [Background execution limits](https://developer.android.com/about/versions/oreo/background)
- [Foreground services](https://developer.android.com/guide/components/foreground-services)

iOS (listed for the questions they would answer; **none verified in this run**)

- [ExternalAccessory](https://developer.apple.com/documentation/externalaccessory)
- [CoreBluetooth background processing](https://developer.apple.com/library/archive/documentation/NetworkingInternetWeb/Conceptual/CoreBluetooth_concepts/CoreBluetoothBackgroundProcessingForIOSApps/PerformingTasksWhileYourAppIsInTheBackground.html)
- [Apple: Bluetooth and third-party devices](https://support.apple.com/en-us/102229)

Background / measurement leads (unverified, for §6.1)

- [Hackaday: benchmarking latency across wireless links](https://hackaday.com/2024/02/11/benchmarking-latency-across-common-wireless-links-for-mcus/)
- [Electric UI: latency comparison](https://electricui.com/blog/latency-comparison)
- [Punch Through: larger ATT MTU](https://punchthrough.com/maximizing-ble-throughput-part-2-use-larger-att-mtu/)
