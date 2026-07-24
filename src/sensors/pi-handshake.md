# Pi ↔ ESP32 GPIO Handshake (firmware contract)

Contract between **Raspberry Pi (`mars-cv`)** and **ESP32 firmware** for teletubby detection.

Copy this into (or link from) the firmware repo. Pi BCM pin numbers below are authoritative for the CV side; map them to whatever ESP GPIO you wire.

---

## Electrical / wiring

| Signal   | Direction   | Pi (BCM)              | Physical header | Idle | Active |
|----------|-------------|-----------------------|-----------------|------|--------|
| **START**  | ESP → Pi  | **GPIO2**             | pin 3           | LOW  | HIGH (rising edge starts search) |
| **DETECT** | Pi → ESP  | **GPIO3**             | pin 5           | LOW  | HIGH (~100 ms pulse = found) |
| **GND**    | common    | any GND               | e.g. pin 6      | —    | — |

**Requirements**

- Common **GND** between ESP32 and Pi.
- **3.3 V logic** on both sides (ESP32 and Pi GPIO are 3.3 V — do not drive 5 V into the Pi).
- On the Pi, **disable I2C** (GPIO2/GPIO3 are also SDA/SCL). If I2C is enabled, handshake lines will fight the bus.

ESP pin choice is up to firmware; only the Pi ends of the wires are fixed as above.

---

## Timing constants (Pi)

| Parameter | Value | Notes |
|-----------|-------|--------|
| DETECT pulse width | **100 ms** (default) | Configurable on Pi via `--detect-pulse-ms` |
| DETECT polarity | Active **HIGH** | Idle LOW before and after pulse |
| START trigger | **Rising edge** (LOW → HIGH) | Level held HIGH after edge is OK |
| Pi re-arm after detect | ~**3 s** | `mars-cv` exits; systemd restarts and waits for START again |

Detection confidence uses a sliding window on the Pi (default: 5 frames, 40% hit rate). That is internal to CV — firmware only cares about DETECT.

---

## Mission sequence

```text
ESP32                         Pi (mars-cv, always-on service)
  |                             |
  |  START = LOW (armed idle)   |  idle: waiting for rising edge on GPIO2
  |                             |
  | ---- START LOW→HIGH ------->|  open cam0, run YOLO
  |  (may leave START HIGH)     |
  |                             |  … inference …
  | <--- DETECT HIGH 100 ms ----|  teletubby yielded
  | <--- DETECT LOW ------------|
  |                             |  process exits
  |  set START = LOW            |  systemd restarts (~3 s) → idle again
  |  (required before next mission)
```

### What the Pi guarantees

1. At boot (and after each restart), `mars-cv` loads the model and **blocks** until a **rising edge** on START (GPIO2).
2. Camera/inference do **not** run until that edge.
3. When a teletubby is confirmed, Pi drives DETECT (GPIO3) **HIGH for ~100 ms**, then LOW, then exits.
4. Systemd brings `mars-cv` back up so the Pi is idle again for the next START edge.

### What firmware must do

1. **Idle:** drive START **LOW**.
2. **Start search:** drive START **HIGH** (produce a clean rising edge). Hold HIGH or pulse — only the edge matters for starting.
3. **Watch DETECT:** treat a HIGH pulse (~100 ms) as “teletubby found.” Prefer edge + duration check or debounce; ignore short glitches.
4. **Re-arm:** after DETECT (or before the next mission), drive START **LOW** and keep it LOW until the next search. Without returning to LOW, the Pi will not see another rising edge.
5. Do **not** assume the Pi is listening for START while it is mid-inference or during the ~3 s restart window after DETECT. Safe pattern: wait for DETECT, then drop START LOW, wait ≥ ~3–5 s (or poll until you’re ready for another mission), then raise START again.

---

## Suggested ESP32 sketch of behavior

```text
boot:
  START_OUT = LOW
  DETECT_IN = input (pulldown or external idle-low)

on_mission_start:
  START_OUT = HIGH          // rising edge → Pi begins inference

on_DETECT_rising (or HIGH for ≥ ~50 ms):
  // teletubby found — stop waiting / advance robot state
  START_OUT = LOW           // re-arm for next mission
  // optional: ignore further DETECT until next mission_start
```

No UART/serial handshake is required for this path. Digital START + DETECT only.

---

## Out of scope (current CV build)

- Dual-camera simultaneous inference
- ACK/hold handshake (pulse only; no wait for ESP ACK)
- Pi staying in-process idle after detect (it **exits** and systemd restarts)

---

## Pi ops (for bringing up the CV side)

Service install (after deploy):

```bash
sudo bash ~/mars-cv/run/install-pi-service.sh
```

```bash
sudo systemctl status mars-cv
journalctl -u mars-cv -f
```

Manual one-shot (no service):

```bash
~/mars-cv/run/mars-cv --camera --loop --model ~/mars-cv/run/models/teletubby-yolov8n.onnx --no-display
```
