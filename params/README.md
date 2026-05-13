# Parameter Snapshots

## Purpose

This directory holds **MAVLink parameter dumps from actual fielded ESP32-S3 boards** in known-good states. These are the **only on-disk copy** of runtime calibration values (accel offsets, gyro offsets, AHRS trim, etc.) that ArduPilot learned via on-water procedures. **If the board's NVS gets erased and you don't have these files, you must redo full 6-face accel calibration + compass calibration.**

## Files

| File | Date | State | Notes |
|---|---|---|---|
| `baseline_v2.parm` | 2026-05-13 | Post-skid-config + post-banner-rename | 529 params. Contains: SERVO1=73 (ThrottleLeft) / SERVO3=74 (ThrottleRight), 6-face accel cal, gyro offsets, AHRS trim, FS_THR_ENABLE=0, LOG_BACKEND_TYPE=0, ARMING_CHECK=178, WP_SPEED=2.0, MODE_ACRO_ENABLED runtime (firmware bit). Compass NOT yet calibrated (室内磁干扰太大). |

## Restore procedure (MAVProxy)

```
mavproxy.py --master=COM58 --baudrate=115200
> param load params/baseline_v2.parm
```

Or via pymavlink:

```python
# see tools/verify/_udp_dump_params.py for the reverse direction
# (loading would iterate the .parm file and PARAM_SET each)
```

## After on-water calibration

Once compass calibration is complete (outdoors, away from magnetic interference), regenerate this file with the post-on-water snapshot. Name it `baseline_v3_postsea.parm` and update this README.

## Critical params recap (for sanity check after restore)

```
SERVO1_FUNCTION   = 73    (ThrottleLeft  → H10 pin 5 / GPIO 12)
SERVO3_FUNCTION   = 74    (ThrottleRight → H10 pin 1 / GPIO 45)
FRAME_CLASS       = 2     (Boat)
FRAME_TYPE        = 0     (Default — skid steering via SERVO functions)
ARMING_CHECK      = 178   (Baro | INS | Params | BoardVoltage)
FS_THR_ENABLE     = 0
LOG_BACKEND_TYPE  = 0
WP_SPEED          = 2.0
INS_ACCOFFS_X     = -0.1355  (6-face cal output)
INS_ACCOFFS_Y     = +0.1508
INS_ACCOFFS_Z     = +0.1462
INS_ACCSCAL_*     ≈ 1.0 ± 0.005
INS_GYROFFS_*     ≈ -0.01 to -0.02 rad/s
AHRS_TRIM_X       = -0.0061  (-0.35°)
AHRS_TRIM_Y       = +0.0116  (+0.66°)
AHRS_EKF_TYPE     = 3
```
