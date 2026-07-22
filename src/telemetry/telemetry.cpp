#include "telemetry/telemetry.h"

#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>

namespace {

constexpr const char* kApSsid = "dragonflyyy";
constexpr const char* kApPassword = "far5678";

WebServer server(80);

const char kIndexHtml[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>dragonflyyy PID</title>
<style>
  body{font-family:system-ui,sans-serif;max-width:420px;margin:24px auto;padding:0 16px;background:#111;color:#eee}
  h1{font-size:1.25rem;margin:0 0 8px}
  h2{font-size:1rem;margin:20px 0 8px;color:#aaa;font-weight:600}
  label{display:flex;justify-content:space-between;align-items:center;margin:8px 0;gap:12px}
  input{width:120px;padding:6px;border:1px solid #444;border-radius:4px;background:#222;color:#eee}
  button{margin:6px 6px 0 0;padding:10px 16px;border:0;border-radius:4px;background:#3a7;color:#fff;font-weight:600;cursor:pointer}
  button.stop{background:#a33}
  button:disabled{opacity:.5}
  .row{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #333}
  .val{font-variant-numeric:tabular-nums;color:#8fd}
  #status{display:inline-block;padding:2px 8px;border-radius:4px;font-size:.85rem;font-weight:600}
  #status.on{background:#163;color:#8f8}
  #status.off{background:#411;color:#f88}
  #msg{margin-top:10px;min-height:1.2em;color:#8f8;font-size:.9rem}
</style>
</head>
<body>
<h1>dragonflyyy</h1>
<p>Drive <span id="status" class="off">STOPPED</span></p>
<button id="start">Start</button>
<button id="stop" class="stop">Stop</button>

<h2>Base speeds</h2>
<label>Left base <input id="lbase" type="number" step="1" min="0" max="255"/></label>
<label>Right base <input id="rbase" type="number" step="1" min="0" max="255"/></label>
<button id="applySpeed">Apply speeds</button>

<h2>PID gains</h2>
<label>kp <input id="kp" type="number" step="0.1"/></label>
<label>ki <input id="ki" type="number" step="0.1"/></label>
<label>kd <input id="kd" type="number" step="0.1"/></label>
<label>integralMax <input id="imax" type="number" step="0.1"/></label>
<button id="apply">Apply gains</button>
<div id="msg"></div>

<h2>Live</h2>
<div class="row"><span>Left speed</span><span class="val" id="ls">—</span></div>
<div class="row"><span>Right speed</span><span class="val" id="rs">—</span></div>
<div class="row"><span>Left PWM duty</span><span class="val" id="ld">—</span></div>
<div class="row"><span>Right PWM duty</span><span class="val" id="rd">—</span></div>
<div class="row"><span>Error</span><span class="val" id="err">—</span></div>
<div class="row"><span>Correction</span><span class="val" id="corr">—</span></div>
<script>
async function post(url,body){
  return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
}
async function refresh(){
  try{
    const r=await fetch('/api/status');
    const j=await r.json();
    const editing=document.activeElement?.tagName==='INPUT';
    if(!editing){
      kp.value=j.kp; ki.value=j.ki; kd.value=j.kd; imax.value=j.integralMax;
      lbase.value=j.leftBase; rbase.value=j.rightBase;
    }
    status.textContent=j.running?'RUNNING':'STOPPED';
    status.className=j.running?'on':'off';
    ls.textContent=j.leftSpeed.toFixed(1);
    rs.textContent=j.rightSpeed.toFixed(1);
    ld.textContent=j.leftDuty;
    rd.textContent=j.rightDuty;
    err.textContent=j.error.toFixed(2);
    corr.textContent=j.correction.toFixed(2);
  }catch(e){}
}
start.onclick=async()=>{
  msg.textContent='Starting…';
  try{const r=await post('/api/drive',{running:true});msg.textContent=r.ok?'Running.':'Failed.';}
  catch(e){msg.textContent='Network error.';}
};
stop.onclick=async()=>{
  msg.textContent='Stopping…';
  try{const r=await post('/api/drive',{running:false});msg.textContent=r.ok?'Stopped.':'Failed.';}
  catch(e){msg.textContent='Network error.';}
};
applySpeed.onclick=async()=>{
  msg.textContent='Sending speeds…';
  try{
    const r=await post('/api/drive',{leftBase:+lbase.value,rightBase:+rbase.value});
    msg.textContent=r.ok?'Speeds applied.':'Failed.';
  }catch(e){msg.textContent='Network error.';}
};
apply.onclick=async()=>{
  msg.textContent='Sending gains…';
  try{
    const r=await post('/api/pid',{kp:+kp.value,ki:+ki.value,kd:+kd.value,integralMax:+imax.value});
    msg.textContent=r.ok?'Gains applied.':'Failed.';
  }catch(e){msg.textContent='Network error.';}
};
setInterval(refresh,200);
refresh();
</script>
</body>
</html>
)HTML";

}  // namespace

float TelemetryServer::parseJsonFloat(const String& body, const char* key,
                                      float fallback) {
  const String needle = String("\"") + key + "\":";
  const int idx = body.indexOf(needle);
  if (idx < 0) {
    return fallback;
  }
  return body.substring(idx + needle.length()).toFloat();
}

bool TelemetryServer::parseJsonBool(const String& body, const char* key,
                                    bool fallback) {
  const String needle = String("\"") + key + "\":";
  const int idx = body.indexOf(needle);
  if (idx < 0) {
    return fallback;
  }
  const String rest = body.substring(idx + needle.length());
  if (rest.startsWith("true")) {
    return true;
  }
  if (rest.startsWith("false")) {
    return false;
  }
  return fallback;
}

void TelemetryServer::begin(TapeFollowPid& pid, MotorDriver& motors) {
  pid_ = &pid;
  motors_ = &motors;

  Serial.println("[WIFI] starting SoftAP…");

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(100);

  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  wifi_country_t country = {};
  strncpy(country.cc, "US", sizeof(country.cc));
  country.schan = 1;
  country.nchan = 11;
  country.max_tx_power = 84;
  country.policy = WIFI_COUNTRY_POLICY_MANUAL;
  esp_wifi_set_country(&country);
  esp_wifi_set_protocol(WIFI_IF_AP,
                        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_wifi_set_ps(WIFI_PS_NONE);

  IPAddress ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(ip, gateway, subnet);

  bool ok = false;
  for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
    ok = WiFi.softAP(kApSsid, kApPassword, /*channel=*/1);
    Serial.printf("[WIFI] softAP WPA attempt %d -> %d (pass len=%u)\n",
                  attempt + 1, ok ? 1 : 0,
                  static_cast<unsigned>(strlen(kApPassword)));
    if (!ok) {
      delay(200);
    }
  }
  if (!ok) {
    Serial.println("[WIFI] WPA SoftAP failed — open network fallback");
    ok = WiFi.softAP(kApSsid, nullptr, 1);
  }

  Serial.printf("[WIFI] softAP()=%d  SSID='%s'  IP=%s  MAC=%s\n", ok ? 1 : 0,
                kApSsid, WiFi.softAPIP().toString().c_str(),
                WiFi.softAPmacAddress().c_str());

  if (!ok) {
    Serial.println("[WIFI] SoftAP did not start");
    return;
  }

  server.on("/", HTTP_GET, [this]() { handleRoot(); });
  server.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
  server.on("/api/pid", HTTP_POST, [this]() { handlePid(); });
  server.on("/api/drive", HTTP_POST, [this]() { handleDrive(); });
  server.begin();
  Serial.println("[WIFI] HTTP server listening on :80 — scan for SSID now");
}

void TelemetryServer::updateSnapshot(const TelemetrySnapshot& snapshot) {
  snapshot_ = snapshot;
}

void TelemetryServer::poll() {
  server.handleClient();

  static uint32_t lastHeartbeatMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastHeartbeatMs >= 3000) {
    lastHeartbeatMs = nowMs;
    Serial.printf("[WIFI] SSID=%s mode=%d apIP=%s stations=%d running=%d\n",
                  kApSsid, WiFi.getMode(), WiFi.softAPIP().toString().c_str(),
                  WiFi.softAPgetStationNum(), drive_.running ? 1 : 0);
  }
}

void TelemetryServer::handleRoot() {
  server.send_P(200, "text/html", kIndexHtml);
}

void TelemetryServer::handleStatus() {
  if (pid_ == nullptr || motors_ == nullptr) {
    server.send(500, "application/json", "{\"error\":\"not ready\"}");
    return;
  }

  const TapeFollowConfig cfg = pid_->getConfig();
  char buf[384];
  snprintf(buf, sizeof(buf),
           "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"integralMax\":%.3f,"
           "\"running\":%s,\"leftBase\":%.2f,\"rightBase\":%.2f,"
           "\"leftSpeed\":%.2f,\"rightSpeed\":%.2f,"
           "\"leftDuty\":%lu,\"rightDuty\":%lu,"
           "\"error\":%.3f,\"correction\":%.3f}",
           cfg.kp, cfg.ki, cfg.kd, cfg.integralMax,
           drive_.running ? "true" : "false", drive_.leftBaseSpeed,
           drive_.rightBaseSpeed, motors_->leftSpeed(), motors_->rightSpeed(),
           static_cast<unsigned long>(motors_->leftDuty()),
           static_cast<unsigned long>(motors_->rightDuty()), snapshot_.error,
           snapshot_.correction);
  server.send(200, "application/json", buf);
}

void TelemetryServer::handlePid() {
  if (pid_ == nullptr) {
    server.send(500, "application/json", "{\"error\":\"not ready\"}");
    return;
  }

  const String body = server.arg("plain");
  const TapeFollowConfig cur = pid_->getConfig();
  const float kp = parseJsonFloat(body, "kp", cur.kp);
  const float ki = parseJsonFloat(body, "ki", cur.ki);
  const float kd = parseJsonFloat(body, "kd", cur.kd);
  const float integralMax = parseJsonFloat(body, "integralMax", cur.integralMax);

  pid_->setGains(kp, ki, kd, integralMax);
  Serial.printf("[PID] gains kp=%.2f ki=%.2f kd=%.2f imax=%.2f\n", kp, ki, kd,
                integralMax);

  server.send(200, "application/json", "{\"ok\":true}");
}

void TelemetryServer::handleDrive() {
  const String body = server.arg("plain");

  if (body.indexOf("\"running\"") >= 0) {
    drive_.running = parseJsonBool(body, "running", drive_.running);
  }
  if (body.indexOf("\"leftBase\"") >= 0) {
    drive_.leftBaseSpeed =
        constrain(parseJsonFloat(body, "leftBase", drive_.leftBaseSpeed), 0.0f,
                  drive_.maxSpeed);
  }
  if (body.indexOf("\"rightBase\"") >= 0) {
    drive_.rightBaseSpeed =
        constrain(parseJsonFloat(body, "rightBase", drive_.rightBaseSpeed),
                  0.0f, drive_.maxSpeed);
  }

  if (!drive_.running && motors_ != nullptr) {
    motors_->stop();
  }

  Serial.printf("[DRIVE] running=%d leftBase=%.1f rightBase=%.1f\n",
                drive_.running ? 1 : 0, drive_.leftBaseSpeed,
                drive_.rightBaseSpeed);

  server.send(200, "application/json", "{\"ok\":true}");
}
