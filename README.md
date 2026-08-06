# enph253-firmware

ESP32-S3 firmware for the ENPH 253 competition robot: tape-follow, metal rock pickup, IMU course phases (ramp + shortcut), and dual-cam teletubby handshake with Mars-CV.

## Robot task flow

As implemented in `src/main.cpp`.

### Boot

1. Init motors, OLED, tape-follow PID, metal detectors, sonar, arm, status LEDs.
2. Start Mars-CV search (`vision.enable(true)` — START edge to Pi).
3. Metal-detector baseline (~3 s, robot still, no metal nearby).
4. IMU calib + settle; lock course yaw origin when heading is stable (or after timeout).
5. Enter **LineFollowing** at cruise speed (L/R base = 90).

### Concurrent behaviors while running

Two parallel concerns run every loop:

| Concern | What it does |
|--------|----------------|
| **Modes** | Line follow ↔ metal approach ↔ arm pickup |
| **Course phases** | IMU-driven ramp / shortcut / re-arm for second turn |
| **Vision** | Teletubby DETECT handshake (independent of metal) |

### Modes (`RobotMode`)

```
LineFollowing ──metal hit──► ApproachAfterMetal ──1.8 s──► PickingUp ──done──► LineFollowing
```

1. **LineFollowing**  
   - Tape-follow PID (`kp=55`, `kd=18`) unless an open-loop ramp shortcut is active.  
   - Poll metal detectors; on hit → approach.  
   - Update IMU course phases.  
   - OLED status refresh.

2. **ApproachAfterMetal**  
   - Keep line-following for **1800 ms**, then stop.  
   - Freeze course yaw (IMU on claw — arm rotation must not count as chassis turn).  
   - Start arm pickup toward metal side (L/R).

3. **PickingUp**  
   - Motors stopped; arm sequence: extend → rotate → lower → sonar scan → grip → retract/raise/recenter → open.  
   - On done: unfreeze course yaw, optionally restart Mars-CV if a DETECT arrived during pickup, reset PID, resume line-follow.

### Course phases (`CoursePhase`, IMU)

Measured yaw is absolute accumulated turn while not frozen.

1. **PreFirst180** — cruise @ 90 until `|yaw| ≥ 160°` (start of incline / first big turn).  
2. **AfterFirst180**  
   - Ramp both sides @ **135** for **4.5 s**.  
   - Open-loop shortcut: **L135 / R0** for **400 ms**.  
   - Open-loop blend: **L135 / R90** for **400 ms**.  
   - Reset tape-follow (search straight), resume cruise @ 90.  
   - When `|yaw| ≤ 30°` again → **PreSecond180**.  
3. **PreSecond180** — cruise @ 90, watching for a second ~180° (no further automatic action yet in this firmware).

### Teletubby vision (Mars-CV GPIO handshake)

Runs every loop via `vision.poll()`. Target: **2** finds (one DETECT per START).

| Situation | Behavior |
|-----------|----------|
| DETECT while driving | Stop, blink camera-side arrow LED 3×, then START again (unless 2nd find — leave Pi idle). |
| DETECT during metal approach / pickup | **Do not** stop or blink. If finds incomplete, set flag to **restart Mars-CV after pickup**. If no DETECT during pickup, Pi may still be searching — no forced restart. |

Handshake details: `lib/ESP32-GPIO-HANDSHAKE.md`.

### Not in this build

- SoftAP / Wi-Fi telemetry web UI (removed).  
- IR beacon hunt / post-second-180 arm deploy (present on some `hj` branches, not wired here yet).
