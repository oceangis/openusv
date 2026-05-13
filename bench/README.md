# USV Test & Build Tools

These are **keeper scripts** worth committing. One-shot debug scripts stay in project root with `_*` prefix and are gitignored.

## bench/verify/  — bench verification scripts

| Script | Purpose |
|---|---|
| `stability.py` | Phase 1-3 stability: 60s telemetry baseline + mode switch + servo output verification |
| `skid_steering.py` | **50Hz continuous RC override** test — definitively proves skid mixing math (USE THIS, not the legacy version) |
| `skid_arm_legacy.py` | Older single-shot RC override test — kept for archival, **don't use for verification** (the RC override times out so output reverts to baseline ≠ neutral) |
| `param_rw.py` | Comprehensive param read/write/persist test across INT8/INT16/FLOAT types |
| `modes.py` | Probe every ArduRover mode number; documents which are compiled in |
| `apply_skid_config.py` | Idempotent application of `SERVO1_FUNCTION=73`, `SERVO3_FUNCTION=74` with reboot + flash-persist verify |
| `accel_cal_verify.py` | After 6-face accel calibration, read INS_ACCOFFS/SCAL/AHRS_TRIM + 10s level sample |

## bench/analyze/  — offline / passive analysis

| Script | Purpose |
|---|---|
| `tlog_dump.py` | Parse MAVProxy session.tlog → message counts, mode changes, EKF flags transitions, STATUSTEXT, command events |
| `udp_comprehensive.py` | 60s telemetry sample over MAVProxy UDP bridge — HB jitter, CPU load, EKF variance, mode stability |
| `udp_dump_params.py` | Dump all 500+ params via UDP bridge to `baseline.parm` (Mission Planner format) |

## bench/flash/  — build & flash helpers

| Script | Purpose |
|---|---|
| `flash_com58.bat` | Build + flash to COM58 (hardcoded port). Handles ESP-IDF env setup, MSYSTEM unset, Python 3.11 path. |
| `build_only.bat` | Build without flashing |
| `mavproxy_bridge.bat` | Start MAVProxy as passive bridge: COM58 → UDP:14550 + session.tlog |

## Default COM port

Most scripts hardcode `COM58`. Change at top of file (`PORT = "COM58"`) if your USB CDC enumerates differently.

## Python environment

Scripts use:
- `pymavlink` (PARAM_REQUEST_READ/PARAM_SET, mavlink_connection)
- `pyserial` (for raw serial access in some scripts)
- Standard library only otherwise

Install with the IDF Python env:
```
D:\Espressif\tools\python_env\idf5.5_py3.11_env\Scripts\python.exe -m pip install pymavlink
```

## Usage convention

```
$env:PYTHONIOENCODING="utf-8"
D:\Espressif\tools\python_env\idf5.5_py3.11_env\Scripts\python.exe bench\verify\skid_steering.py
```
