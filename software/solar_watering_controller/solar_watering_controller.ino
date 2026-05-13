/*
 * Solar Watering Controller - ESP8266 (ESP-12 Module)
 *
 * Wiring:
 *   D0 (GPIO16) → Relay IN (active-HIGH) → Van điện từ
 *   D1 (GPIO5)  → DS1307 SCL
 *   D2 (GPIO4)  → DS1307 SDA
 *   D5 (GPIO14) → Button (active-LOW, pull-up nội)
 *
 * WiFi:  Nhấn giữ nút GPIO14 5 giây → bật AP "SolarWater-XXXX" / 192.168.4.1
 *        Sau 5 phút không có ai kết nối → tắt AP tự động
 * Time:  DS1307 lưu giờ địa phương, đọc lúc boot; set qua Web UI → ghi DS1307
 *
 * Board: Generic ESP8266 Module
 * Core:  ESP8266 Arduino Core (Board Manager)
 * Libs:  RTClib by Adafruit (Library Manager)
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RTClib.h>
#include <time.h>

// ── Pin ───────────────────────────────────────────────────────────────────────
#define VALVE_PIN  16   // GPIO16 = D0, active-HIGH relay
#define BTN_PIN    14   // GPIO14 = D5, active-LOW, pull-up nội
#define LED_PIN     2   // GPIO2  = D4, active-LOW, LED xanh ESP-12
#define I2C_SDA     4   // GPIO4  = D2
#define I2C_SCL     5   // GPIO5  = D1

// ── EEPROM ────────────────────────────────────────────────────────────────────
#define EEPROM_SIZE  256
#define ADDR_MAGIC     0   // 1 byte
#define ADDR_TZ      129   // 1 byte (int8_t)
#define ADDR_SCHED   130   // sizeof(Schedule)
#define MAGIC_VAL   0xC7

// ── Hằng số ───────────────────────────────────────────────────────────────────
#define AP_PREFIX        "SolarWater"
#define AP_PASSWORD      "12345678"

#define AP_TX_POWER      0     // 0dBm — tầm phủ ~1-2m khi setup
#define AP_BEACON_MS   500

#define BTN_HOLD_MS   5000UL   // Giữ nút 5s để bật AP
#define AP_IDLE_MS  300000UL   // Tắt AP sau 5 phút không có client

#define SCHED_CHECK_MS   10000UL   // Kiểm tra lịch mỗi 10 giây
#define SCHED_WINDOW_SEC   60      // Tưới khi sai lệch ≤60s

#define RTC_SYNC_MS  3600000UL  // Đồng bộ lại từ DS1307 mỗi 1 giờ

// ── Cấu trúc lịch tưới ───────────────────────────────────────────────────────
struct Schedule {
  bool     enabled;
  uint8_t  hour;
  uint8_t  minute;
  uint16_t durationSec;
  uint8_t  mode;    // 0 = hàng ngày | 1 = hàng tuần
  uint8_t  days;    // bitmask: bit0=CN bit1=T2 … bit6=T7
};

// ── Biến toàn cục ─────────────────────────────────────────────────────────────
int8_t   g_tz   = 7;
Schedule g_sched;

RTC_DS1307 g_rtc;
bool       g_rtcOk = false;

ESP8266WebServer g_srv(80);

bool     g_wifiActive = false;
bool     g_apMode     = false;
bool     g_watering   = false;
uint32_t g_waterEnd   = 0;

uint32_t g_lastCheck        = 0;
bool     g_wateredThisCycle = false;

// Nút bấm
uint32_t g_btnPressedAt = 0;
bool     g_btnTriggered = false;

// AP idle timeout
uint32_t g_apIdleSince  = 0;

// RTC re-sync
uint32_t g_lastRtcSync  = 0;

// ── Thời gian ─────────────────────────────────────────────────────────────────
static time_t nowTs()    { return time(nullptr); }
static bool   timeValid(){ return nowTs() > 1609459200UL; }

static void applyTimezone() {
  configTime((long)g_tz * 3600, 0, "", "");
}

static String fmtTime(time_t t) {
  char buf[24];
  strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", localtime(&t));
  return String(buf);
}

// Đọc DS1307 → cập nhật đồng hồ hệ thống
// DS1307 lưu giờ địa phương; mktime() chuyển về UTC dựa trên TZ đã cấu hình.
static bool syncFromRTC() {
  if (!g_rtcOk) return false;
  DateTime dt = g_rtc.now();
  if (dt.year() < 2020) return false;

  struct tm t = {};
  t.tm_year  = dt.year() - 1900;
  t.tm_mon   = dt.month() - 1;
  t.tm_mday  = dt.day();
  t.tm_hour  = dt.hour();
  t.tm_min   = dt.minute();
  t.tm_sec   = dt.second();
  t.tm_isdst = -1;
  time_t utc = mktime(&t);

  timeval tv = {utc, 0};
  settimeofday(&tv, nullptr);
  return true;
}

// Ghi giờ địa phương hiện tại vào DS1307
static void syncToRTC() {
  if (!g_rtcOk) return;
  time_t n = nowTs();
  struct tm *lt = localtime(&n);
  g_rtc.adjust(DateTime(
    lt->tm_year + 1900,
    lt->tm_mon  + 1,
    lt->tm_mday,
    lt->tm_hour,
    lt->tm_min,
    lt->tm_sec
  ));
}

// ── EEPROM ────────────────────────────────────────────────────────────────────
static void cfgLoad() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(ADDR_MAGIC) != MAGIC_VAL) {
    g_tz    = 7;
    g_sched = {false, 6, 0, 300, 0, 0x7F};
    return;
  }
  EEPROM.get(ADDR_TZ,    g_tz);
  EEPROM.get(ADDR_SCHED, g_sched);
}

static void cfgSave() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(ADDR_MAGIC, MAGIC_VAL);
  EEPROM.put(ADDR_TZ,    g_tz);
  EEPROM.put(ADDR_SCHED, g_sched);
  EEPROM.commit();
}

// ── Lịch tưới ─────────────────────────────────────────────────────────────────
static bool dayMatches(const struct tm &t) {
  if (g_sched.mode == 0) return true;
  return (g_sched.days >> t.tm_wday) & 1;
}

static bool isWateringTime() {
  if (!timeValid()) return false;
  time_t n = nowTs();
  struct tm t = *localtime(&n);
  int nowSec   = t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
  int schedSec = g_sched.hour * 3600 + g_sched.minute * 60;
  int diff     = nowSec - schedSec;
  return (diff >= 0 && diff <= SCHED_WINDOW_SEC && dayMatches(t));
}

static String nextWaterStr() {
  if (!g_sched.enabled) return "Tắt";
  if (!timeValid())     return "Chưa set giờ";
  time_t n = nowTs();
  struct tm t = *localtime(&n);
  int nowSec   = t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
  int schedSec = g_sched.hour * 3600 + g_sched.minute * 60;
  for (int d = 0; d <= 7; d++) {
    time_t cand = n - nowSec + (time_t)d * 86400 + schedSec;
    if (cand <= n) continue;
    struct tm ct = *localtime(&cand);
    if (dayMatches(ct)) {
      char buf[20];
      strftime(buf, sizeof(buf), "%d/%m %H:%M", &ct);
      return String(buf);
    }
  }
  return "N/A";
}

// ── Van điện từ ───────────────────────────────────────────────────────────────
static void valveSet(bool open) {
  digitalWrite(VALVE_PIN, open ? HIGH : LOW);
}

static void waterStart(uint16_t sec) {
  if (g_watering) return;
  g_watering = true;
  g_waterEnd = millis() + (uint32_t)sec * 1000;
  valveSet(true);
  Serial.printf("[valve] Mở %ds\n", sec);
}

static void waterCheck() {
  if (g_watering && (int32_t)(millis() - g_waterEnd) >= 0) {
    g_watering = false;
    valveSet(false);
    g_wateredThisCycle = true;
    Serial.println("[valve] Đóng");
  }
}

// ── WiFi ──────────────────────────────────────────────────────────────────────
static void startAP() {
  if (g_apMode) return;
  char apName[32];
  snprintf(apName, sizeof(apName), "%s-%04X", AP_PREFIX, (uint16_t)ESP.getChipId());
  WiFi.mode(WIFI_AP);
  WiFi.setOutputPower(AP_TX_POWER);
  WiFi.softAP(apName, AP_PASSWORD);
  struct softap_config cfg;
  wifi_softap_get_config(&cfg);
  cfg.beacon_interval = AP_BEACON_MS;
  wifi_softap_set_config(&cfg);
  g_wifiActive  = true;
  g_apMode      = true;
  g_apIdleSince = 0;
  g_srv.begin();
  Serial.printf("[wifi] AP: %s  IP: 192.168.4.1\n", apName);
}

static void wifiOff() {
  if (!g_wifiActive) return;
  g_srv.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  g_wifiActive  = false;
  g_apMode      = false;
  g_apIdleSince = 0;
  Serial.println("[wifi] Off");
}

// ── Nút bấm ───────────────────────────────────────────────────────────────────
static void checkButton(uint32_t now_ms) {
  bool pressed = (digitalRead(BTN_PIN) == LOW);
  if (pressed) {
    if (g_btnPressedAt == 0) {
      g_btnPressedAt = now_ms ? now_ms : 1;
      g_btnTriggered = false;
    } else if (!g_btnTriggered && now_ms - g_btnPressedAt >= BTN_HOLD_MS) {
      g_btnTriggered = true;
      if (!g_apMode) {
        Serial.println("[btn] Giữ 5s → bật AP");
        startAP();
      }
    }
  } else {
    g_btnPressedAt = 0;
    g_btnTriggered = false;
  }
}

// ── AP idle timeout ───────────────────────────────────────────────────────────
static void checkApTimeout(uint32_t now_ms) {
  if (!g_apMode) return;
  uint8_t clients = WiFi.softAPgetStationNum();
  if (clients > 0) {
    g_apIdleSince = 0;
  } else {
    if (g_apIdleSince == 0) g_apIdleSince = now_ms ? now_ms : 1;
    if (now_ms - g_apIdleSince >= AP_IDLE_MS) {
      Serial.println("[wifi] AP: 5 phút không client → tắt");
      wifiOff();
    }
  }
}

// ── LED ───────────────────────────────────────────────────────────────────────
// AP mode    : chớp 2 lần rồi nghỉ (on200-off200-on200-off800, chu kỳ 1.4s)
// Nút đang giữ : sáng liên tục
// WiFi off   : tắt
static void updateLed(uint32_t now_ms) {
  bool on;
  if (g_apMode) {
    uint32_t t = now_ms % 1400;
    on = (t < 200) || (t >= 400 && t < 600);
  } else {
    on = (g_btnPressedAt != 0);
  }
  digitalWrite(LED_PIN, on ? LOW : HIGH);
}

// ── Web UI ────────────────────────────────────────────────────────────────────
static const char HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<link rel="icon" href="data:,">
<title>Solar Watering</title>
<style>
*{box-sizing:border-box}
body{font-family:Arial,sans-serif;max-width:500px;margin:20px auto;padding:0 14px;color:#222}
h1{color:#2e7d32;font-size:1.25em;margin-bottom:6px}
.stat{background:#f1f8e9;padding:12px 14px;border-radius:8px;margin-bottom:14px;line-height:1.9;font-size:.93em}
.warn{background:#fff3e0;border-left:4px solid #f57c00;padding:10px 14px;border-radius:4px;margin-bottom:14px;font-size:.9em}
.card{border:1px solid #ddd;border-radius:8px;padding:16px;margin-bottom:12px}
.card h3{margin:0 0 10px;font-size:1em}
label{display:block;margin-top:10px;font-weight:600;font-size:.88em;color:#555}
input[type=number],input[type=date],input[type=time],select{
  width:100%;padding:8px;border:1px solid #bbb;border-radius:4px;margin-top:3px;font-size:.95em}
.chk-row{display:flex;align-items:center;gap:8px;margin-top:10px}
.chk-row input{width:auto}
.days{display:flex;gap:6px;flex-wrap:wrap;margin-top:8px}
.days label{display:flex;flex-direction:column;align-items:center;font-weight:400;
  font-size:.85em;width:36px;cursor:pointer;background:#f5f5f5;border-radius:4px;
  padding:4px 2px;margin:0}
.days input{width:auto;margin:4px 0 0}
.row{display:flex;gap:8px}
.row>*{flex:1}
.btn{padding:10px 18px;border:none;border-radius:5px;cursor:pointer;
  font-size:.95em;font-weight:600;margin-top:8px}
.btn-save{background:#2e7d32;color:#fff;width:100%;margin-top:14px;padding:12px}
.btn-time{background:#6a1b9a;color:#fff;width:100%;margin-top:10px}
.btn-water{background:#1565c0;color:#fff}
.btn-stop{background:#e65100;color:#fff}
.btn-reset{background:#b71c1c;color:#fff;font-size:.8em;padding:7px 12px}
.hint{color:#888;font-size:.78em;margin-top:3px}
.watering{color:#1b5e20;font-weight:700}
.err{color:#c62828}
@keyframes blink{50%{opacity:0}}
.blink{animation:blink 1s step-end infinite}
</style>
</head>
<body>
<h1>&#127807; Solar Watering Controller</h1>

%WARN%

<div class="stat">
  <b>Thời gian:</b> <span id="clk">%TIME%</span><br>
  <b>RTC DS1307:</b> %RTC%<br>
  <b>Van điện từ:</b> %VALVE%<br>
  <b>Tưới tiếp theo:</b> %NEXT%<br>
  <b>WiFi:</b> %WIFI%
</div>

<div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:14px">
  <form method="post" action="/water">
    <button class="btn btn-water">&#128167; Tưới ngay (%DUR%s)</button>
  </form>
  %STOP_BTN%
</div>

<!-- Set giờ -->
<div class="card">
  <h3>&#8987; Thời gian &amp; Múi giờ</h3>
  <button class="btn btn-time" type="button" onclick="syncTime()" style="margin-top:0">
    &#128241; Đồng bộ giờ từ thiết bị này
  </button>
  <p class="hint" style="margin:6px 0 8px">Hoặc nhập thủ công bên dưới:</p>
  <form method="post" action="/settime">
    <div class="row">
      <div>
        <label>Ngày</label>
        <input type="date" name="date" value="%CUR_DATE%" required>
      </div>
      <div>
        <label>Giờ</label>
        <input type="time" name="t" value="%CUR_TIME%" required>
      </div>
    </div>
    <button class="btn btn-time" type="submit">&#10003; Cập nhật giờ</button>
  </form>
</div>

<!-- Cài đặt -->
<form method="post" action="/save">
<div class="card">
  <h3>&#127758; Múi giờ</h3>
  <label>UTC+</label>
  <input type="number" name="tz" min="-12" max="14" value="%TZ%">
  <div class="hint">Việt Nam = 7</div>
</div>

<!-- Lịch tưới -->
<div class="card">
  <h3>&#128336; Lịch tưới cây</h3>
  <div class="chk-row">
    <input type="checkbox" name="enabled" id="en" %CHK_EN%>
    <label for="en" style="margin:0;font-weight:400">Bật lịch tưới tự động</label>
  </div>

  <label>Chế độ</label>
  <select name="mode" id="modeSelect" onchange="toggleDays(this.value)">
    <option value="0" %SEL_DAILY%>Hàng ngày</option>
    <option value="1" %SEL_WEEKLY%>Hàng tuần</option>
  </select>

  <div id="daysRow" class="days" style="%DAYS_STYLE%">
    <label>CN<input type="checkbox" name="d0" %D0%></label>
    <label>T2<input type="checkbox" name="d1" %D1%></label>
    <label>T3<input type="checkbox" name="d2" %D2%></label>
    <label>T4<input type="checkbox" name="d3" %D3%></label>
    <label>T5<input type="checkbox" name="d4" %D4%></label>
    <label>T6<input type="checkbox" name="d5" %D5%></label>
    <label>T7<input type="checkbox" name="d6" %D6%></label>
  </div>

  <label>Giờ tưới (0–23)</label>
  <input type="number" name="hour" min="0" max="23" value="%HOUR%">
  <label>Phút (0–59)</label>
  <input type="number" name="minute" min="0" max="59" value="%MIN%">
  <label>Thời gian tưới (giây)</label>
  <input type="number" name="dur" min="5" max="7200" value="%DUR%">
  <div class="hint">300s = 5 phút &nbsp;|&nbsp; 600s = 10 phút</div>
</div>

<button class="btn btn-save" type="submit">&#128190; Lưu cài đặt</button>
</form>

<div style="margin-top:10px">
  <form method="post" action="/reset"
    onsubmit="return confirm('Xoá toàn bộ cài đặt?')">
    <button class="btn btn-reset" type="submit">&#9888; Factory Reset</button>
  </form>
</div>

<script>
function toggleDays(v){
  document.getElementById('daysRow').style.display=v=='1'?'flex':'none';
}
var clkEl=document.getElementById('clk');
var y,mo,d,h,m,s;
(function(){
  var txt=clkEl?clkEl.textContent:'';
  var r=txt.match(/(\d+)\/(\d+)\/(\d+) (\d+):(\d+):(\d+)/);
  if(r){d=+r[1];mo=+r[2];y=+r[3];h=+r[4];m=+r[5];s=+r[6];}
  else{var now=new Date();y=now.getFullYear();mo=now.getMonth()+1;d=now.getDate();h=now.getHours();m=now.getMinutes();s=now.getSeconds();}
})();
function syncTime(){
  var ts=Math.floor(new Date().getTime()/1000);
  fetch('/synctime?ts='+ts)
    .then(function(){alert('Đã đồng bộ giờ!');location.reload();})
    .catch(function(){alert('Lỗi đồng bộ!');});
}
function pad(n){return('0'+n).slice(-2)}
function tick(){
  if(!clkEl)return;
  s++;if(s>=60){s=0;m++;}if(m>=60){m=0;h++;}if(h>=24){h=0;}
  clkEl.textContent=pad(d)+'/'+pad(mo)+'/'+y+' '+pad(h)+':'+pad(m)+':'+pad(s);
}
setInterval(tick,1000);
</script>
</body>
</html>)HTML";

static String buildPage() {
  String html = FPSTR(HTML);
  time_t n = nowTs();

  html.replace("%WARN%", timeValid() ? "" :
    "<div class='warn'>&#9888; <b>Chưa có thời gian!</b> "
    "Vui lòng cập nhật giờ bên dưới để lịch tưới hoạt động.</div>");

  html.replace("%TIME%", timeValid() ? fmtTime(n) : "Đang đồng bộ...");
  html.replace("%RTC%",  g_rtcOk
    ? "<span style='color:#2e7d32'>Kết nối OK</span>"
    : "<span class='err'>Không tìm thấy!</span>");
  html.replace("%NEXT%", nextWaterStr());

  // Valve status
  if (g_watering) {
    uint32_t rem = (g_waterEnd > millis()) ? (g_waterEnd - millis()) / 1000 : 0;
    html.replace("%VALVE%",
      "<span class='watering blink'>&#128167; Đang tưới — còn " + String(rem) + "s</span>");
    html.replace("%STOP_BTN%",
      "<form method='post' action='/stop'>"
      "<button class='btn btn-stop'>&#9209; Dừng</button></form>");
  } else {
    html.replace("%VALVE%", "Đóng");
    html.replace("%STOP_BTN%", "");
  }

  // WiFi status
  if (g_apMode) {
    char apName[32];
    snprintf(apName, sizeof(apName), "%s-%04X", AP_PREFIX, (uint16_t)ESP.getChipId());
    uint8_t clients = WiFi.softAPgetStationNum();
    String wifiStr = "AP — <b>" + String(apName) + "</b> / 192.168.4.1";
    if (clients == 0 && g_apIdleSince) {
      uint32_t elapsed = (millis() - g_apIdleSince) / 1000;
      uint32_t remain  = elapsed < AP_IDLE_MS / 1000 ? AP_IDLE_MS / 1000 - elapsed : 0;
      wifiStr += " (tắt sau " + String(remain) + "s)";
    }
    html.replace("%WIFI%", wifiStr);
  } else {
    html.replace("%WIFI%", "Tắt — giữ nút 5s để bật");
  }

  // Pre-fill date/time inputs
  if (timeValid()) {
    char dateBuf[12], timeBuf[6];
    struct tm *lt = localtime(&n);
    strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", lt);
    strftime(timeBuf, sizeof(timeBuf), "%H:%M",    lt);
    html.replace("%CUR_DATE%", dateBuf);
    html.replace("%CUR_TIME%", timeBuf);
  } else {
    html.replace("%CUR_DATE%", "2026-01-01");
    html.replace("%CUR_TIME%", "06:00");
  }

  html.replace("%TZ%",         String(g_tz));
  html.replace("%CHK_EN%",     g_sched.enabled   ? "checked" : "");
  html.replace("%SEL_DAILY%",  g_sched.mode == 0 ? "selected" : "");
  html.replace("%SEL_WEEKLY%", g_sched.mode == 1 ? "selected" : "");
  html.replace("%DAYS_STYLE%", g_sched.mode == 1 ? "display:flex" : "display:none");
  for (int i = 0; i < 7; i++)
    html.replace("%D" + String(i) + "%", (g_sched.days >> i) & 1 ? "checked" : "");
  html.replace("%HOUR%", String(g_sched.hour));
  html.replace("%MIN%",  String(g_sched.minute));
  html.replace("%DUR%",  String(g_sched.durationSec));

  return html;
}

// ── HTTP Handlers ─────────────────────────────────────────────────────────────
static void handleSyncTime() {
  if (g_srv.hasArg("ts")) {
    time_t ts = (time_t)g_srv.arg("ts").toInt();
    if (ts > 1609459200UL) {
      timeval tv = {ts, 0};
      settimeofday(&tv, nullptr);
      syncToRTC();
      Serial.println("[time] Sync từ browser: " + fmtTime(nowTs()));
    }
  }
  g_srv.send(200, "text/plain", "ok");
}

static void handleRoot() {
  g_srv.sendHeader("Cache-Control", "no-cache, no-store");
  g_srv.send(200, "text/html; charset=utf-8", buildPage());
}

static void handleSetTime() {
  String dateStr = g_srv.arg("date");  // "2026-05-10"
  String timeStr = g_srv.arg("t");     // "06:00"

  int yr = 2026, mo = 1, dy = 1, hr = 0, mn = 0;
  sscanf(dateStr.c_str(), "%d-%d-%d", &yr, &mo, &dy);
  sscanf(timeStr.c_str(), "%d:%d",    &hr, &mn);

  struct tm t = {};
  t.tm_year  = yr - 1900;
  t.tm_mon   = mo - 1;
  t.tm_mday  = dy;
  t.tm_hour  = hr;
  t.tm_min   = mn;
  t.tm_isdst = -1;
  time_t utc = mktime(&t);

  timeval tv = {utc, 0};
  settimeofday(&tv, nullptr);
  syncToRTC();

  Serial.println("[time] Set: " + fmtTime(nowTs()));
  g_srv.sendHeader("Location", "/");
  g_srv.send(302);
}

static void handleSave() {
  g_tz = (int8_t)constrain(g_srv.arg("tz").toInt(), -12, 14);

  g_sched.enabled     = g_srv.hasArg("enabled");
  g_sched.mode        = g_srv.arg("mode") == "1" ? 1 : 0;
  g_sched.hour        = (uint8_t)constrain(g_srv.arg("hour").toInt(),   0, 23);
  g_sched.minute      = (uint8_t)constrain(g_srv.arg("minute").toInt(), 0, 59);
  g_sched.durationSec = (uint16_t)constrain(g_srv.arg("dur").toInt(),   5, 7200);

  if (g_sched.mode == 1) {
    g_sched.days = 0;
    for (int i = 0; i < 7; i++)
      if (g_srv.hasArg("d" + String(i))) g_sched.days |= (1 << i);
    if (!g_sched.days) g_sched.days = 0x7F;
  } else {
    g_sched.days = 0x7F;
  }

  cfgSave();
  applyTimezone();
  // DS1307 lưu giờ địa phương → cần ghi lại khi timezone thay đổi
  syncToRTC();

  g_srv.sendHeader("Location", "/");
  g_srv.send(302);
}

static void handleWater() {
  waterStart(g_sched.durationSec > 0 ? g_sched.durationSec : 30);
  g_srv.sendHeader("Location", "/");
  g_srv.send(302);
}

static void handleStop() {
  if (g_watering) {
    g_watering = false;
    valveSet(false);
    Serial.println("[valve] Dừng thủ công");
  }
  g_srv.sendHeader("Location", "/");
  g_srv.send(302);
}

static void handleReset() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(ADDR_MAGIC, 0xFF);
  EEPROM.commit();
  g_srv.send(200, "text/html; charset=utf-8",
    "<meta charset=UTF-8>"
    "<p style='font-family:Arial;padding:20px'>&#10003; Đã reset! Khởi động lại...</p>");
  delay(1000);
  ESP.restart();
}

static void handleNotFound() {
  Serial.println("[http] 404: " + g_srv.uri());
  g_srv.sendHeader("Location", "/");
  g_srv.send(302);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[boot] Solar Watering Controller");

  pinMode(VALVE_PIN, OUTPUT);
  valveSet(false);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // tắt LED lúc boot

  pinMode(BTN_PIN, INPUT_PULLUP);

  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);

  cfgLoad();
  applyTimezone();  // phải set TZ trước khi mktime() trong syncFromRTC

  Wire.begin(I2C_SDA, I2C_SCL);
  g_rtcOk = g_rtc.begin();
  if (g_rtcOk) {
    if (syncFromRTC()) {
      Serial.println("[rtc] Giờ từ DS1307: " + fmtTime(nowTs()));
    } else {
      Serial.println("[rtc] DS1307 chưa set giờ — cần set qua Web UI");
    }
  } else {
    Serial.println("[rtc] Không tìm thấy DS1307 trên I2C!");
  }

  g_srv.on("/",         HTTP_GET,  handleRoot);
  g_srv.on("/synctime", HTTP_GET,  handleSyncTime);
  g_srv.on("/settime",  HTTP_POST, handleSetTime);
  g_srv.on("/save",     HTTP_POST, handleSave);
  g_srv.on("/water",    HTTP_POST, handleWater);
  g_srv.on("/stop",     HTTP_POST, handleStop);
  g_srv.on("/reset",    HTTP_POST, handleReset);
  g_srv.onNotFound(handleNotFound);

  Serial.println("[boot] Ready — giữ nút GPIO14 5s để bật AP");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now_ms = millis();

  if (g_wifiActive) g_srv.handleClient();
  waterCheck();
  checkButton(now_ms);
  checkApTimeout(now_ms);
  updateLed(now_ms);

  // Đồng bộ lại từ DS1307 mỗi 1 giờ (bù trôi oscillator ESP8266)
  if (now_ms - g_lastRtcSync >= RTC_SYNC_MS) {
    g_lastRtcSync = now_ms;
    syncFromRTC();
  }

  // Kiểm tra lịch tưới mỗi 10 giây
  if (now_ms - g_lastCheck >= SCHED_CHECK_MS) {
    g_lastCheck = now_ms;

    if (g_wateredThisCycle && !isWateringTime())
      g_wateredThisCycle = false;

    if (g_sched.enabled && !g_watering && !g_wateredThisCycle) {
      if (isWateringTime()) {
        Serial.printf("[sched] Tưới %ds\n", g_sched.durationSec);
        waterStart(g_sched.durationSec);
      }
    }
  }

  delay(10);
}
