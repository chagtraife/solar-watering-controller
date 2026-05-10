/*
 * Solar Watering Controller - ESP8266 (ESP-12 Module)
 *
 * Wiring:
 *   D0 (GPIO16) → Relay IN (active-HIGH) → Van điện từ
 *   A0 để trống (dùng đo Vcc nội bộ để phát hiện mất điện)
 *
 * WiFi:  Boot lần đầu (chưa có SSID) → AP "SolarWater-XXXX" / 192.168.4.1
 *        Đã cấu hình SSID → thử kết nối STA, retry mỗi 5 phút
 * Time:  User tự set giờ qua Web UI (không cần internet)
 *        Lưu EEPROM khi Vcc sụt + backup mỗi 6 giờ
 *
 * Board: Generic ESP8266 Module
 * Core:  ESP8266 Arduino Core (Board Manager)
 */

// Đặt trước mọi #include — chuyển ADC sang đo Vcc nội bộ (A0 để trống)
ADC_MODE(ADC_VCC);

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <time.h>

// ── Pin ───────────────────────────────────────────────────────────────────────
#define VALVE_PIN  16   // GPIO16 = D0, active-HIGH relay

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

#define AP_TX_POWER      0     // 0dBm — tầm phủ ~1-2m khi setup lần đầu
#define AP_BEACON_MS   500     // beacon interval AP (ms)

#define WIFI_TIMEOUT_MS  30000UL
#define WIFI_RETRY_MS    (5UL * 60 * 1000)    // Thử kết nối STA mỗi 5 phút

// IP tĩnh khi kết nối hotspot — đổi theo subnet thực tế (xem Serial lúc DHCP)
static const IPAddress STA_IP     ( 10, 143, 117, 200);
static const IPAddress STA_GW     ( 10, 143, 117,   1);
static const IPAddress STA_SUBNET (255, 255, 255,   0);
#define SCHED_CHECK_MS   10000UL               // Kiểm tra lịch mỗi 10 giây
#define SCHED_WINDOW_SEC   60                  // Tưới khi sai lệch ≤60s

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

bool     g_wifiActive = false;
bool     g_apMode     = false;
bool     g_watering   = false;
uint32_t g_waterEnd   = 0;

uint32_t g_lastCheck      = 0;
uint32_t g_lastTimeSave   = 0;   // Backup 6 giờ/lần
uint32_t g_lastVccCheck   = 0;
uint32_t g_wifiCycleAt    = 0;   // Thời điểm bắt đầu phase hiện tại (on/off)
uint32_t g_reconnectAt    = 0;   // Thời điểm bắt đầu reconnect (0 = không reconnect)
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
  static uint32_t lastSaved = 0;
  time_t now = nowTs();

  // Chỉ lưu khi khác ngày, giờ hoặc phút so với lần trước
  if (lastSaved > 1609459200UL) {
    struct tm tn = *localtime(&now);
    time_t   ls  = (time_t)lastSaved;
    struct tm tp = *localtime(&ls);
    if (tn.tm_mday == tp.tm_mday &&
        tn.tm_hour == tp.tm_hour &&
        tn.tm_min  == tp.tm_min)  return;
  }

  lastSaved = (uint32_t)now;
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(ADDR_TIME, lastSaved);
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

static bool wateringSoon() {
  if (!g_sched.enabled || !timeValid() || g_watering) return g_watering;
  time_t n = nowTs();
  struct tm t = *localtime(&n);
  int nowSec   = t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
  int schedSec = g_sched.hour * 3600 + g_sched.minute * 60;
  int diff     = schedSec - nowSec;
  return diff >= 0 && diff <= (int)(WIFI_TIMEOUT_MS / 1000);
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
static void startAP() {
  char apName[32];
  snprintf(apName, sizeof(apName), "%s-%04X", AP_PREFIX, (uint16_t)ESP.getChipId());
  WiFi.mode(WIFI_AP);
  WiFi.setOutputPower(AP_TX_POWER);
  WiFi.softAP(apName, AP_PASSWORD);
  struct softap_config cfg;
  wifi_softap_get_config(&cfg);
  cfg.beacon_interval = AP_BEACON_MS;
  wifi_softap_set_config(&cfg);
  g_wifiActive = true;
  g_apMode     = true;
  g_srv.begin();
  Serial.printf("[wifi] AP: %s  IP: 192.168.4.1\n", apName);
}

static void staConnect() {
  if (g_wifiActive || strlen(g_ssid) == 0) return;
  Serial.printf("[wifi] Connecting to '%s'...\n", g_ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
  WiFi.config(STA_IP, STA_GW, STA_SUBNET);
  WiFi.begin(g_ssid, g_pass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > WIFI_TIMEOUT_MS) {
      Serial.printf("[wifi] STA failed (status=%d)\n", WiFi.status());
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      return;
    }
    delay(250);
  }
  g_wifiActive = true;
  g_apMode     = false;
  g_srv.begin();
  Serial.println("[wifi] STA: " + WiFi.localIP().toString());
}

static void wifiOff() {
  if (!g_wifiActive) return;
  g_srv.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  g_wifiActive = false;
  g_apMode     = false;
  Serial.println("[wifi] Off");
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
  document.getElementById('daysRow').style.display=v=='1'?'flex':'none';
}
// Đồng hồ khởi tạo từ giờ device (parse chuỗi dd/mm/yyyy HH:MM:SS)
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

  // Cảnh báo nếu chưa set giờ
  html.replace("%WARN%", timeValid() ? "" :
    "<div class='warn'>&#9888; <b>Chưa có thời gian!</b> "
    "Vui lòng cập nhật giờ bên dưới để lịch tưới hoạt động.</div>");

  html.replace("%TIME%", timeValid() ? fmtTime(n) : "Đang đồng bộ...");
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
    html.replace("%WIFI%", "&#9989; <b>" + WiFi.localIP().toString() + "</b>");
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
static void handleSyncTime() {
  if (g_srv.hasArg("ts")) {
    time_t ts = (time_t)g_srv.arg("ts").toInt();
    if (ts > 1609459200UL) {
      timeval tv = {ts, 0};
      settimeofday(&tv, nullptr);
      timeSave();
      Serial.println("[time] Sync từ browser: " + fmtTime(nowTs()));
    }
  }
  g_srv.send(200, "text/plain", "ok");
}

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
  char newSsid[64] = {};
  char newPass[64] = {};
  if (g_srv.hasArg("ssid"))
    strncpy(newSsid, g_srv.arg("ssid").c_str(), sizeof(newSsid) - 1);
  if (g_srv.arg("pass").length() > 0)
    strncpy(newPass, g_srv.arg("pass").c_str(), sizeof(newPass) - 1);

  bool wifiChanged = (strcmp(newSsid, g_ssid) != 0)
                  || (strlen(newPass) > 0 && strcmp(newPass, g_pass) != 0);

  strncpy(g_ssid, newSsid, sizeof(g_ssid));
  if (strlen(newPass) > 0) strncpy(g_pass, newPass, sizeof(g_pass));

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

  if (wifiChanged) {
    g_srv.send(200, "text/html; charset=utf-8",
      "<meta charset=UTF-8>"
      "<p style='font-family:Arial;padding:20px;font-size:1.1em'>"
      "&#10003; Đã lưu! Đang khởi động lại...</p>"
      "<script>setTimeout(()=>location.href='/',3000)</script>");
    delay(1500);
    ESP.restart();
  } else {
    g_srv.sendHeader("Location", "/");
    g_srv.send(302);
  }
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

  cfgLoad();
  applyTimezone();   // Set múi giờ (không cần internet)
  timeLoad();        // Khôi phục giờ từ EEPROM checkpoint

  if (timeValid())
    Serial.println("[time] Khôi phục: " + fmtTime(nowTs()));
  else
    Serial.println("[time] Chưa có giờ — cần set qua Web UI");

  g_srv.on("/",         HTTP_GET,  handleRoot);
  g_srv.on("/synctime", HTTP_GET,  handleSyncTime);
  g_srv.on("/settime",  HTTP_POST, handleSetTime);
  g_srv.on("/save",    HTTP_POST, handleSave);
  g_srv.on("/water",   HTTP_POST, handleWater);
  g_srv.on("/stop",    HTTP_POST, handleStop);
  g_srv.on("/reset",   HTTP_POST, handleReset);
  g_srv.onNotFound(handleNotFound);

  // Lần đầu chưa có SSID → AP để cấu hình; đã có → thử STA ngay
  if (strlen(g_ssid) == 0) {
    startAP();
  } else {
    staConnect();
  }
  Serial.println("[boot] Ready");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  if (g_wifiActive) g_srv.handleClient();
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

  // 3. STA: giữ kết nối, reconnect ngay khi mất, retry mỗi 5 phút nếu fail
  if (!g_apMode) {
    if (g_wifiActive && WiFi.status() != WL_CONNECTED) {
      if (g_reconnectAt == 0) {
        g_reconnectAt = now_ms;
        WiFi.reconnect();
        Serial.println("[wifi] Lost, reconnecting...");
      } else if (now_ms - g_reconnectAt >= WIFI_TIMEOUT_MS) {
        g_reconnectAt = 0;
        wifiOff();
        g_wifiCycleAt = now_ms;
        Serial.println("[wifi] Reconnect failed, retry in 5min");
      }
    } else if (g_wifiActive) {
      g_reconnectAt = 0;
    } else if (!wateringSoon() && now_ms - g_wifiCycleAt >= WIFI_RETRY_MS) {
      staConnect();
      g_wifiCycleAt = now_ms;
    }
  }

  delay(10);
}
