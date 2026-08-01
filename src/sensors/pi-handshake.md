# Pi ↔ ESP32 GPIO Handshake (firmware contract)

**Superseded by dual-cam contract:** see [`lib/ESP32-GPIO-HANDSHAKE.md`](../../lib/ESP32-GPIO-HANDSHAKE.md).

The deployed Pi runs `mars-cv --dual` with multiplexed **START + DETECT_CAM1** on Pi BCM **GPIO4**, and **DETECT_CAM0** on Pi BCM **GPIO3**.

| Signal | Direction | Pi BCM | ESP GPIO (this board) |
|--------|-----------|--------|------------------------|
| START + DETECT_CAM1 | shared wire | GPIO4 | `kPiCam1StartPin` (10) |
| DETECT_CAM0 | Pi → ESP | GPIO3 | `kPiCam0Pin` (11) |

Firmware implementation: `src/sensors/vision.cpp`.
