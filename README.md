# FootPod — milestone 1: raw data capture

Goal: get clean raw IMU data from the pod onto your laptop, so cadence, contact time and stride length can be developed against real runs.

## 1. Flash the pod

1. Arduino IDE → Preferences → Additional Boards Manager URLs:
   `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
2. Boards Manager → install **Seeed nRF52 Boards** (the non-mbed one; this sketch uses the Bluefruit BLE stack).
3. Library Manager → install **Seeed Arduino LSM6DS3**.
4. Tools → Board → **Seeed XIAO nRF52840 Sense**.
5. Open `footpod_capture/footpod_capture.ino`, set `POD_SIDE`, upload.
6. Serial Monitor (115200) should show `WHO_AM_I = 0x6A` and `samples/s: ~416`.

Untested on hardware — expect a compile fix or two. If the IMU fails to init, the board selection is the usual culprit.

## 2. Record

```
pip install bleak pandas numpy scipy matplotlib
python footpod_logger.py --name FootPod-L --out walk1.csv
```

Watch the live line: ~416 samples/s and 0 lost/bad packets. If you see bad packets, the MTU didn't negotiate up — tell me what OS/adapter you're on.

### Or record from your phone

Put `footpod.html` on GitHub Pages next to the sit-up pages. Web Bluetooth needs HTTPS, which Pages provides. Open it in Chrome on Android, tap **Connect**, and pick FootPod-L.

- **Mounting check:** stand still for 3 s, then walk for 8 s. The page learns gyro offset, which way is up, and the foot's pitch axis, and remembers them per pod.
- **Live view:** cadence, steps, sample rate, lost packets, clipping, and 5-second charts with detected strides marked.
- **Record:** label the run, start, stop, then Download or Share the CSV (e.g. to Drive). The file is the same format as the laptop logger.

Keep the screen on and the tab in front while recording.

## 3. Check

```
python footpod_plot.py walk1.csv
```

Prints effective sample rate, peak g / dps with clipping counts, and a median cadence estimate; shows accel magnitude, gyro, and per-stride cadence.

## Test protocol

Mount temporarily (zip ties or tape on the laces), board flat, USB-C pointing toward the toe. Note the orientation — the firmware math will assume it.

1. **Bench:** pod still on the table for 10 s (gyro ≈ 0, |accel| ≈ 1 g), then rotate by hand.
2. **Walk:** 2 min indoors. Cadence should read ~100–120 spm.
3. **Treadmill:** 2 min each at 3 known speeds, e.g. `tm_6mph.csv`. Count steps for 30 s by hand once to check cadence. Known belt speed is the ground truth for stride length later.
4. **Outdoor run:** laptop in a backpack works for now.

Check clipping counts after the treadmill runs. A few clipped accel samples at foot strike are normal; lots means we revisit the range.

## Packet format (BLE notify, 236 bytes)

| Field | Type | Notes |
|---|---|---|
| t0_us | uint32 | micros() at first sample |
| seq | uint16 | gaps = dropped packets |
| n | uint8 | samples in packet (19) |
| odr_div8 | uint8 | 52 = 416 Hz |
| samples | 19 × 6 × int16 | gx, gy, gz, ax, ay, az raw; 0.070 dps/LSB, 0.488 mg/LSB |

Service `a3c40000-5f1d-4b8e-9c3a-7d1e2f3b4c50`, characteristic `a3c40001-…`.

## Next milestones

2. On-pod stride detection + cadence → BLE Running Speed and Cadence profile (works with watches / nRF Toolbox)
3. Contact time, foot-strike angle, ZUPT stride length
4. Battery, sleep/wake, case
5. Android app → Health Connect / Strava
