# ESP32 ↔ Raspberry Pi GPIO Handshake (firmware contract)

**Audience:** ESP32 firmware developers  
**Pi side (deployed):** `mars-cv --dual` on **marspi**, model `teletubby-yolov8n-320.onnx`  
**Transport:** 3.3 V digital GPIO only — no UART / I2C / SPI for this path  

Copy this file into the firmware repo. **Pi BCM pin numbers are authoritative.** Map them to whatever ESP32 GPIO pins you wire.

---

## Goal

Competition flow is **two independent search sessions** (one teletubby each):

1. ESP32 tells the Pi to **START** dual-camera YOLO search.
2. When a camera confirms a teletubby, the Pi pulses that camera’s DETECT line (cam0 → GPIO3, cam1 → GPIO4), then **stops inference and returns to idle**.
3. ESP32 stops the robot, flashes the LED arrow (firmware), then drives again and issues a **second START**.
4. Same as step 2 for the second teletubby; Pi returns to idle again.
5. After the second DETECT, ESP32 no longer needs the Pi (leave Pi idle; do not START again).

The Pi process stays up across both sessions (no systemd restart between them). Each START opens the cameras again (default warmup: **5 frames**).

---

## Pin map (Pi BCM)

| Signal | Direction | Pi BCM | Pi header | Idle | Active meaning |
|--------|-----------|--------|-----------|------|----------------|
| **START** | ESP → Pi | **GPIO4** | physical **7** | LOW | Rising edge → start one dual search |
| **DETECT_CAM0** | Pi → ESP | **GPIO3** | physical **5** | LOW | ~100 ms HIGH = teletubby on **cam0** |
| **DETECT_CAM1** | Pi → ESP | **GPIO4** | physical **7** | LOW | ~100 ms HIGH = teletubby on **cam1** |
| **GND** | common | any GND | e.g. physical **9** | — | Must share ground |

### Critical: GPIO4 is multiplexed

**START** and **DETECT_CAM1** share the same wire (Pi GPIO4).

| Phase | Who drives Pi GPIO4 | Role |
|-------|---------------------|------|
| Idle / re-arm | **ESP** (push-pull LOW) | Armed for next START rising edge |
| Session start | **ESP** drives LOW→HIGH | Rising edge starts Pi search |
| After START (~few ms) | **ESP releases** (input / high-Z) | Wire free for Pi |
| Searching | **Pi** (output, idle LOW) | Ready to pulse DETECT_CAM1 |
| cam1 detect | **Pi** pulses HIGH ~100 ms | ESP reads as DETECT_CAM1 |
| After DETECT (any cam) | **Pi** reclaims GPIO4 as START input | Back to idle |
| Before next START | **ESP** drives LOW again | Re-arm, then rising edge when ready |

If the ESP keeps driving GPIO4 HIGH (or LOW push-pull) after START, the Pi cannot reclaim the line as DETECT_CAM1 and cam1 signaling will fail (bus fight / `armDetectOutputs` error on Pi).

**DETECT_CAM0 (GPIO3)** is Pi-driven only. ESP configures it as input for the whole run.

---

## Electrical requirements

- **3.3 V logic only.** Pi GPIO is not 5 V tolerant.
- **Common GND** between ESP32 and Pi.
- Prefer short wires; add series resistors (~220–330 Ω) if you want protection against brief contention during the GPIO4 handoff.
- Optional: weak pulldown on the multiplexed wire so it idles LOW when neither side drives.
- On the Pi, I2C should be disabled if it claims GPIO3 (SCL). That is a Pi config issue, not firmware.

ESP pin choice is up to you. Example placeholders:

```text
#define PIN_TO_PI_GPIO4   /* your ESP GPIO wired to Pi BCM 4 (START + DETECT_CAM1) */
#define PIN_FROM_PI_GPIO3 /* your ESP GPIO wired to Pi BCM 3 (DETECT_CAM0) */
```

---

## Timing (Pi-side constants — treat as contract)

| Parameter | Value | Firmware implication |
|-----------|-------|----------------------|
| START trigger | Rising edge LOW→HIGH | Level after edge does not matter; edge does |
| ESP release window | **≤ ~20 ms** after START edge | Pi sleeps ~20 ms then claims GPIO4 as output |
| DETECT pulse width | **100 ms** (± software jitter) | Accept HIGH ≥ ~50 ms as valid; ignore glitches |
| DETECT polarity | Active HIGH | Idle is LOW |
| Detects per START | **1** | One DETECT pulse, then Pi returns to idle |
| Warmup before counting | **5 frames** on Pi (~0.3 s @ 15 fps) | Do not expect DETECT in the first fraction of a second after START |
| Return to idle | cameras close + GPIO4 → START input | Typically well under **1 s** after DETECT; wait ≥ ~1 s before re-arming START if unsure |
| Between sessions | ESP re-drives GPIO4 **LOW**, then START when ready | Required before the second teletubby search |
| After 2nd DETECT | ESP done with vision | Pi stays idle; no further START needed |
| Search duration | until 1st detect or stop | ESP should timeout if needed for robot safety |

CV thresholds (not required for GPIO code): conf / window / hit-rate as deployed on Pi; imgsz 320. Firmware only needs START / DETECT.

---

## Mission sequence (normative)

```text
ESP32                                         Pi (mars-cv service)
  |                                             |
  |  Pi_GPIO4 = LOW (driven)                    |  idle: wait rising edge on GPIO4
  |  Pi_GPIO3 = input                           |  GPIO3 held LOW as DETECT_CAM0 out
  |                                             |
  |  ==== first teletubby ====                  |
  |  Pi_GPIO4 = HIGH  ---------------------->   |  sees rising edge
  |  delay 1–5 ms                               |
  |  Pi_GPIO4 = INPUT / Hi-Z  -------------->   |  ~20 ms later: claim GPIO4 as DETECT_CAM1
  |                                             |  open cam0 + cam1, warmup 5 frames, YOLO
  |                                             |  … searching …
  |                                             |
  |  first find (example: cam0)                 |
  |  <---------------- Pi_GPIO3 HIGH 100 ms --- |  pulse DETECT_CAM0
  |  <---------------- Pi_GPIO3 LOW ----------- |
  |                                             |  stop cams; reclaim GPIO4 as START input
  |                                             |  idle again
  |                                             |
  |  stop robot, flash LED arrow (firmware)     |
  |  resume driving                             |
  |  Pi_GPIO4 = OUTPUT LOW                      |  (re-arm before 2nd START)
  |                                             |
  |  ==== second teletubby ====                 |
  |  Pi_GPIO4 = HIGH  ---------------------->   |  sees rising edge
  |  delay 1–5 ms                               |
  |  Pi_GPIO4 = INPUT / Hi-Z  -------------->   |  claim DETECT_CAM1; open cams; search
  |                                             |
  |  second find (example: cam1)                |
  |  <---------------- Pi_GPIO4 HIGH 100 ms --- |  pulse DETECT_CAM1
  |  <---------------- Pi_GPIO4 LOW ----------- |
  |                                             |  stop cams; return to idle
  |                                             |
  |  vision done — do not START again           |  stays idle
```

Exactly **one** DETECT pulse is sent per START. Each pulse uses the GPIO for the camera that confirmed (GPIO3 = cam0, GPIO4 = cam1). The second teletubby requires a **new START** after the Pi is idle again.

---

## ESP32 required behavior

### Idle / re-armed

1. Drive the Pi-GPIO4 wire **LOW** (push-pull output).
2. Configure Pi-GPIO3 wire as **input** (prefer pulldown or rely on Pi idle-LOW).
3. Do **not** raise START until the Pi is expected to be idle (after boot, or ≥ ~1 s after the previous DETECT).

### Start search (each teletubby)

1. Drive Pi-GPIO4 **HIGH** → creates rising edge.
2. Within **1–5 ms**, switch that ESP pin to **input / high-Z** (release the bus).
3. Watch for **one** pulse:
   - Pi-GPIO3 HIGH pulse → **cam0** detection
   - Pi-GPIO4 HIGH pulse → **cam1** detection  
     (same physical pin you just released; now read as input)

### On DETECT

1. Treat a sustained HIGH of roughly **50–150 ms** as a valid pulse (debounce short spikes).
2. Record which camera fired; advance robot state for that find (1st or 2nd ordinal on the ESP).
3. After the **first** DETECT: stop robot / flash LED / etc., then re-arm GPIO4 **LOW** and issue START again when ready for the second search.
4. After the **second** DETECT: vision complete — leave Pi idle; do not START again.

### Must not

- Leave Pi-GPIO4 driven after START (blocks cam1 DETECT).
- Issue another START while the Pi is still searching (before the DETECT for that session).
- Expect two DETECT pulses from a single START — each START yields at most one.
- Drive 5 V into the Pi.
- Assume UART ACKs — there are none.

---

## Suggested firmware state machine

```text
states:
  IDLE
  START_PULSE
  WAIT_DETECT   // expect 1 pulse this session
  POST_DETECT   // robot action between finds / after last find

IDLE:
  pin4 = OUTPUT LOW
  pin3 = INPUT
  findsDone = 0   // 0, 1, or 2 teletubbies confirmed this run
  on need_vision && findsDone < 2 → START_PULSE

START_PULSE:
  pin4 = OUTPUT HIGH
  delay 2 ms
  pin4 = INPUT   // release for DETECT_CAM1
  → WAIT_DETECT

WAIT_DETECT:
  if pin3 high for ≥ 50 ms:
    record cam0; findsDone++
    → POST_DETECT
  if pin4 high for ≥ 50 ms:
    record cam1; findsDone++
    → POST_DETECT
  if timeout (robot policy) → handle abort / retry

POST_DETECT:
  // findsDone == 1: stop, flash LED arrow, resume drive, then re-arm
  // findsDone == 2: vision done; stay out of START
  pin4 = OUTPUT LOW   // re-arm only if another START is coming
  if findsDone < 2 → IDLE (then START again when ready)
  else → DONE (no more START)
```

---

## Reference sketch (Arduino / ESP-IDF style pseudocode)

Pin names are placeholders — replace with your board wiring.

```cpp
// Wire these to the Pi header:
//   PIN_PI4  <->  Pi BCM GPIO4  (physical pin 7)  START + DETECT_CAM1
//   PIN_PI3  <->  Pi BCM GPIO3  (physical pin 5)  DETECT_CAM0
//   GND      <->  Pi GND

static constexpr int PIN_PI4 = /* ... */;
static constexpr int PIN_PI3 = /* ... */;
static constexpr int DETECT_MIN_MS = 50;
static constexpr int PI_IDLE_MS = 1000; // wait after DETECT before re-arm/START
static constexpr int REQUIRED_FINDS = 2; // robot-side count across two STARTs

enum class Phase { Idle, WaitDetect, PostDetect, Done };

Phase phase = Phase::Idle;
int findsDone = 0;
int lastCam = -1; // 0 or 1 for most recent pulse

void armIdle() {
  pinMode(PIN_PI4, OUTPUT);
  digitalWrite(PIN_PI4, LOW);
  pinMode(PIN_PI3, INPUT); // or INPUT_PULLDOWN if available
  phase = Phase::Idle;
}

void startVisionSearch() {
  // Rising edge on Pi GPIO4
  pinMode(PIN_PI4, OUTPUT);
  digitalWrite(PIN_PI4, LOW);
  delayMicroseconds(100);
  digitalWrite(PIN_PI4, HIGH);
  delay(2);
  // Release so Pi can drive DETECT_CAM1 on the same wire
  pinMode(PIN_PI4, INPUT);
  phase = Phase::WaitDetect;
}

// Call from loop() or a timer
void pollDetect() {
  if (phase != Phase::WaitDetect) return;

  static unsigned long high3Start = 0, high4Start = 0;
  const unsigned long now = millis();

  auto accept = [&](int cam) {
    lastCam = cam;
    ++findsDone;
    high3Start = high4Start = 0;
    phase = Phase::PostDetect;
    // handle robot event for this cam / ordinal (1st vs 2nd find)
  };

  if (digitalRead(PIN_PI3) == HIGH) {
    if (high3Start == 0) high3Start = now;
    else if (now - high3Start >= DETECT_MIN_MS) accept(0);
  } else {
    high3Start = 0;
  }

  if (phase == Phase::WaitDetect && digitalRead(PIN_PI4) == HIGH) {
    if (high4Start == 0) high4Start = now;
    else if (now - high4Start >= DETECT_MIN_MS) accept(1);
  } else if (digitalRead(PIN_PI4) == LOW) {
    high4Start = 0;
  }
}

void handlePostDetect() {
  if (phase != Phase::PostDetect) return;

  if (findsDone == 1) {
    // stop robot, flash LED arrow, resume driving (firmware)
    delay(PI_IDLE_MS); // give Pi time to return to idle
    armIdle();
    // when ready for second teletubby:
    // startVisionSearch();
  } else {
    // findsDone >= 2 — vision complete
    armIdle(); // optional; leave line LOW
    phase = Phase::Done;
  }
}
```

Interrupt-on-rising + duration check is also fine; the pulse is ~100 ms so polling at 1–5 ms is enough.

---

## Semantics for the robot

| Event | Meaning |
|-------|---------|
| START edge accepted by Pi | Dual YOLO search running (cam0 + cam1) for **one** find |
| DETECT_CAM0 / CAM1 pulse | Teletubby confirmed on that camera; **search stops; Pi idle** |
| 1st DETECT (ESP ordinal) | First teletubby — stop, LED arrow, then START again when driving |
| 2nd DETECT (ESP ordinal) | Second teletubby — vision done; no more START |
| No DETECT before ESP timeout | Robot policy (abort / retry after idle); Pi may still be searching until you stop externally |

There is no “search failed” GPIO from the Pi. Timeout is entirely on the ESP / robot side.

---

## Bring-up checklist (firmware)

- [ ] Common GND connected
- [ ] Both lines measured 3.3 V max
- [ ] Idle: Pi GPIO4 wire sits LOW under ESP drive
- [ ] START: scope/logic analyzer shows clean LOW→HIGH, then ESP releases within a few ms
- [ ] After START, Pi journal shows `START received` / dual cams opening
- [ ] First detect: GPIO3 or GPIO4 pulse; Pi returns to idle (`session 1`)
- [ ] ESP re-drives GPIO4 LOW, then START again
- [ ] Second detect: another pulse; Pi returns to idle (`session 2`); no further START

Pi log watch (on marspi):

```bash
journalctl -u mars-cv -f
```

Expected idle line:

```text
ESP handshake idle: waiting for START on GPIO4 (cam0 DETECT GPIO3, cam1 DETECT GPIO4; session N, stop after 1 detect)
```

Expected on detect:

```text
[cam0] TELETUBBY DETECTED #1/1 (confidence ..., GPIO3 pulsed 100 ms)
[cam0] Session detects reached (1) — stopping search
DETECT done — returned to idle (ESP may START again for the next teletubby)
```

---

## Out of scope

- Serial / UART protocol
- ACK from ESP back to Pi
- Pi exiting after each detect (it **stays running** and re-idles in-process)
- Choosing which ESP GPIO numbers to use (board-specific)
- LED arrow / robot motion (firmware repo)

---

## Revision

| Item | Value |
|------|--------|
| Pi START | BCM **GPIO4** |
| Pi DETECT cam0 | BCM **GPIO3** |
| Pi DETECT cam1 | BCM **GPIO4** (after START release) |
| DETECT pulse | **100 ms** active HIGH |
| Detects per START | **1** (then idle; second find = new START) |
| Warmup | **5 frames** per session |
| Pi between sessions | in-process return to idle (no systemd restart required) |
| Model / mode | dual cam, `teletubby-yolov8n-320.onnx` |
