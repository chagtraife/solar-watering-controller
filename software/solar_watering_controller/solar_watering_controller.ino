/*
 * Solar Watering Controller - ESP8266 (ESP-12 Module)
 *
 * Wiring:
 *   D0 (GPIO16) → Relay IN (active-LOW) → Van điện từ
 *
 * Power: WiFi Light Sleep (~1-2mA idle) — phù hợp với solar + pin sạc
 * Time:  User tự set giờ qua Web UI (không cần internet)
 *        Giờ lưu EEPROM khi phát hiện điện sụt + backup mỗi 6 giờ
 *        A0 phải để trống (dùng đo Vcc nội bộ để phát hiện mất điện)
 *
 * Board: Generic ESP8266 Module
 * Core:  ESP8266 Arduino Core (Board Manager)
 *
 * Lần đầu cấu hình:
 *   1. Kết nối WiFi "SolarWater-XXXX"  password: 12345678
 *   2. Mở trình duyệt → 192.168.4.1
 *   3. Nhập WiFi nhà + set giờ + lịch tưới → Lưu
 */

// Đặt trước mọi #include — chuyển ADC sang đo Vcc nội bộ (A0 để trống)
ADC_MODE(ADC_VCC);

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <time.h>

// ── Pin ───────────────────────────────────────────────────────────────────────
#define VALVE_PIN  16   // GPIO16 = D0, active-LOW relay

// ── EEPROM ────────────────────────────────────────────────────────────────────
#define EEPROM_SIZE  256
#define ADDR_MAGIC     0   // 1 byte
#define ADDR_SSID      1   // 64 bytes
#define ADDR_PASS     65   // 64 bytes
#define ADDR_TZ      129   // 1 byte (int8_t)
#define ADDR_SCHED   130   // sizeof(Schedule)
#define ADDR_TIME    160   // 4 bytes — Unix timestamp checkpoint
#define MAGIC_VAL   0xC7

// ── Hằng số ───────────────────────────────────────────────────────────────────
#define AP_PREFIX        "SolarWater"
#define AP_PASSWORD      "12345678"

#define WIFI_TIMEOUT_MS   15000UL
#define SCHED_CHECK_MS   10000UL               // Kiểm tra lịch mỗi 10 giây
#define SCHED_WINDOW_SEC    60                 // Tưới khi sai lệch ≤60s
#define TIME_SAVE_MS     (30UL * 60 * 1000)   // Lưu giờ vào EEPROM mỗi 30 phút
#define RECONNECT_MS     (60UL * 1000)         // Thử reconnect WiFi mỗi 1 phút

// ── Phát hiện mất điện qua Vcc ───────────────────────────────────────────────
#define VCC_CHECK_MS     1000UL    // Đo Vcc mỗi 1 giây
#define VCC_LOW_MV       2700      // Ngưỡng "điện sụt" (mV) — lưu EEPROM ngay
#define TIME_BACKUP_MS   (6UL * 3600 * 1000)  // Backup định kỳ mỗi 6 giờ

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
char     g_ssid[64];
char     g_pass[64];
int8_t   g_tz   = 7;
Schedule g_sched;

ESP8266WebServer g_srv(80);

bool     g_wifiOk     = false;
bool     g_apMode     = false;
bool     g_watering   = false;
uint32_t g_waterEnd   = 0;

uint32_t g_lastCheck     = 0;
uint32_t g_lastTimeSave  = 0;   // Backup 6 giờ/lần
uint32_t g_lastVccCheck  = 0;
uint32_t g_lastReconnect = 0;
bool     g_wateredThisCycle = false;

// ── Thời gian ─────────────────────────────────────────────────────────────────
static time_t nowTs()    { return time(nullptr); }
static bool   timeValid(){ return nowTs() > 1609459200UL; }

// Áp dụng múi giờ (không kết nối NTP server)
static void applyTimezone() {
  configTime((long)g_tz * 3600, 0, "", "");
}

static String fmtTime(time_t t) {
  char buf[24];
  strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", localtime(&t));
  return String(buf);
}

// ── EEPROM ────────────────────────────────────────────────────────────────────
static void cfgLoad() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(ADDR_MAGIC) != MAGIC_VAL) {
    memset(g_ssid, 0, sizeof(g_ssid));
    memset(g_pass, 0, sizeof(g_pass));
    g_tz    = 7;
    g_sched = {false, 6, 0, 300, 0, 0x7F};
    return;
  }
  EEPROM.get(ADDR_SSID,  g_ssid);
  EEPROM.get(ADDR_PASS,  g_pass);
  EEPROM.get(ADDR_TZ,    g_tz);
  EEPROM.get(ADDR_SCHED, g_sched);
}

static void cfgSave() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(ADDR_MAGIC, MAGIC_VAL);
  EEPROM.put(ADDR_SSID,  g_ssid);
  EEPROM.put(ADDR_PASS,  g_pass);
  EEPROM.put(ADDR_TZ,    g_tz);
  EEPROM.put(ADDR_SCHED, g_sched);
  EEPROM.commit();
}

// Lưu giờ hiện tại vào EEPROM (checkpoint để khôi phục sau mất điện)
static void timeSave() {
  if (!timeValid()) return;
  uint32_t ts = (uint32_t)nowTs();
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(ADDR_TIME, ts);
  EEPROM.commit();
}

// Khôi phục giờ từ EEPROM khi boot; mặc định 10/05/2026 00:00 nếu chưa có
static void timeLoad() {
  EEPROM.begin(EEPROM_SIZE);
  uint32_t ts;
  EEPROM.get(ADDR_TIME, ts);
  if (ts > 1609459200UL) {
    timeval tv = {(time_t)ts, 0};
    settimeofday(&tv, nullptr);
  } else {
    struct tm def = {};
    def.tm_year = 2026 - 1900;
    def.tm_mon  = 5 - 1;   // tháng 5
    def.tm_mday = 10;
    def.tm_isdst = -1;
    timeval tv = {mktime(&def), 0};
    settimeofday(&tv, nullptr);
  }
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
    timeSave();   // Lưu giờ ngay sau khi tưới xong
    Serial.println("[valve] Đóng");
  }
}

// ── WiFi ──────────────────────────────────────────────────────────────────────
static bool wifiConnect() {
  if (strlen(g_ssid) == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
  WiFi.begin(g_ssid, g_pass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > WIFI_TIMEOUT_MS) return false;
    delay(250);
  }
  return true;
}

static void startAP() {
  WiFi.mode(WIFI_AP);
  char apName[32];
  snprintf(apName, sizeof(apName), "%s-%04X", AP_PREFIX, (uint16_t)ESP.getChipId());
  WiFi.softAP(apName, AP_PASSWORD);
  Serial.printf("[wifi] AP: %s  IP: 192.168.4.1\n", apName);
}

// ── Web UI ────────────────────────────────────────────────────────────────────
static const char HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="30">
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
input[type=text],input[type=password],input[type=number],
input[type=date],input[type=time],select{
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
@keyframes blink{50%{opacity:0}}
.blink{animation:blink 1s step-end infinite}
</style>
</head>
<body>
<h1>&#127807; Solar Watering Controller</h1>

%WARN%

<div class="stat">
  <b>Thời gian:</b> <span id="clk">%TIME%</span><br>
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
  <h3>&#8987; Cài đặt thời gian</h3>
  <p class="hint" style="margin:0 0 8px">Sau mất điện, giờ có thể lệch — hãy cập nhật lại.</p>
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

<!-- WiFi + Timezone -->
<form method="post" action="/save">
<div class="card">
  <h3>&#128225; Cài đặt WiFi</h3>
  <label>Tên mạng (SSID)</label>
  <input type="text" name="ssid" value="%SSID%" maxlength="63">
  <label>Mật khẩu</label>
  <input type="password" name="pass" maxlength="63" placeholder="Để trống = không đổi">
  <label>Múi giờ UTC+</label>
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
  document.getElementById('daysRow').style.display = v=='1'?'flex':'none';
}
// Đồng hồ real-time — khởi tạo từ giờ server, tăng mỗi giây
%CLOCK_INIT%
function pad(n){return('0'+n).slice(-2)}
function tick(){
  if(!clkEl) return;
  s++; if(s>=60){s=0;m++;} if(m>=60){m=0;h++;} if(h>=24){h=0;}
  clkEl.textContent=pad(d)+'/'+pad(mo)+'/'+y+' '+pad(h)+':'+pad(m)+':'+pad(s);
}
var clkEl=document.getElementById('clk');
setInterval(tick,1000);
</script>
</body>
</html>)HTML";

static String buildPage() {
  String html = FPSTR(HTML);
  time_t n = nowTs();

  // Cảnh báo nếu chưa set giờ
  html.replace("%WARN%", timeValid() ? "" :
    "<div class='warn'>&#9888; <b>Chưa có thời gian!</b> "
    "Vui lòng cập nhật giờ bên dưới để lịch tưới hoạt động.</div>");

  if (timeValid()) {
    struct tm *lt = localtime(&n);
    char jsInit[80];
    snprintf(jsInit, sizeof(jsInit),
      "var y=%d,mo=%d,d=%d,h=%d,m=%d,s=%d;",
      lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
      lt->tm_hour, lt->tm_min, lt->tm_sec);
    html.replace("%TIME%",      fmtTime(n));
    html.replace("%CLOCK_INIT%", jsInit);
  } else {
    html.replace("%TIME%",      "<b style='color:#e65100'>Chưa set</b>");
    html.replace("%CLOCK_INIT%", "var y=0,mo=0,d=0,h=0,m=0,s=0;");
  }
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
    html.replace("%WIFI%", "AP — <b>" + String(apName) + "</b> / 192.168.4.1");
  } else {
    html.replace("%WIFI%", WiFi.status() == WL_CONNECTED
      ? "&#9989; <b>" + WiFi.localIP().toString() + "</b>"
      : "&#10060; Đang kết nối...");
  }

  // Pre-fill date/time inputs với giờ hiện tại
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

  // Form fields
  html.replace("%SSID%",       String(g_ssid));
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
static void handleRoot() {
  g_srv.send(200, "text/html; charset=utf-8", buildPage());
}

static void handleSetTime() {
  String dateStr = g_srv.arg("date");  // "2026-05-10"
  String timeStr = g_srv.arg("t");     // "06:00"

  int yr = 2026, mo = 1, dy = 1, hr = 0, mn = 0;
  sscanf(dateStr.c_str(), "%d-%d-%d", &yr, &mo, &dy);
  sscanf(timeStr.c_str(), "%d:%d",    &hr, &mn);

  // mktime() chuyển giờ địa phương (đã set bởi applyTimezone) → UTC
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
  timeSave();   // Lưu ngay vào EEPROM

  Serial.println("[time] Set: " + fmtTime(nowTs()));
  g_srv.sendHeader("Location", "/");
  g_srv.send(302);
}

static void handleSave() {
  if (g_srv.hasArg("ssid"))
    strncpy(g_ssid, g_srv.arg("ssid").c_str(), sizeof(g_ssid) - 1);
  if (g_srv.arg("pass").length() > 0)
    strncpy(g_pass, g_srv.arg("pass").c_str(), sizeof(g_pass) - 1);

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
  applyTimezone();   // Áp dụng múi giờ mới nếu thay đổi

  g_srv.send(200, "text/html; charset=utf-8",
    "<meta charset=UTF-8>"
    "<p style='font-family:Arial;padding:20px;font-size:1.1em'>"
    "&#10003; Đã lưu! Đang khởi động lại...</p>"
    "<script>setTimeout(()=>location.href='/',3000)</script>");
  delay(1500);
  ESP.restart();
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
  g_srv.sendHeader("Location", "http://192.168.4.1/");
  g_srv.send(302);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[boot] Solar Watering Controller");

  pinMode(VALVE_PIN, OUTPUT);
  valveSet(false);

  cfgLoad();
  applyTimezone();   // Set múi giờ (không cần internet)
  timeLoad();        // Khôi phục giờ từ EEPROM checkpoint

  if (timeValid())
    Serial.println("[time] Khôi phục: " + fmtTime(nowTs()));
  else
    Serial.println("[time] Chưa có giờ — cần set qua Web UI");

  g_wifiOk = wifiConnect();
  if (g_wifiOk) {
    Serial.println("[wifi] " + WiFi.localIP().toString());
  } else {
    g_apMode = true;
    startAP();
  }

  g_srv.on("/",        HTTP_GET,  handleRoot);
  g_srv.on("/settime", HTTP_POST, handleSetTime);
  g_srv.on("/save",    HTTP_POST, handleSave);
  g_srv.on("/water",   HTTP_POST, handleWater);
  g_srv.on("/stop",    HTTP_POST, handleStop);
  g_srv.on("/reset",   HTTP_POST, handleReset);
  g_srv.onNotFound(handleNotFound);
  g_srv.begin();
  Serial.println("[http] Server ready");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  g_srv.handleClient();
  waterCheck();

  uint32_t now_ms = millis();

  // 1a. Phát hiện mất điện qua Vcc — lưu EEPROM ngay khi điện sụt
  if (timeValid() && now_ms - g_lastVccCheck >= VCC_CHECK_MS) {
    g_lastVccCheck = now_ms;
    uint16_t vcc = ESP.getVcc();
    if (vcc < VCC_LOW_MV) {
      timeSave();
      Serial.printf("[power] Vcc thấp (%dmV) — đã lưu giờ!\n", vcc);
    }
  }

  // 1b. Backup định kỳ mỗi 6 giờ (phòng khi điện mất quá nhanh)
  if (timeValid() && now_ms - g_lastTimeSave >= TIME_BACKUP_MS) {
    g_lastTimeSave = now_ms;
    timeSave();
    Serial.println("[time] Backup: " + fmtTime(nowTs()));
  }

  // 2. Kiểm tra lịch tưới mỗi 10 giây
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

  // 3. Tự kết nối lại WiFi nếu mất
  if (g_wifiOk && !g_apMode && WiFi.status() != WL_CONNECTED
      && now_ms - g_lastReconnect >= RECONNECT_MS) {
    g_lastReconnect = now_ms;
    Serial.println("[wifi] Reconnect...");
    WiFi.reconnect();
  }

  delay(10);
}
