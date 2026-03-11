# Copilot instructions for this repository

## Build, test, and lint commands

- No repository-level build or lint tooling is configured (`pyproject.toml`, `setup.cfg`, `tox.ini`, and `requirements*.txt` are not present).
- There is no automated pytest/unittest suite in this repo.
- Available manual hardware validation scripts (run from repo root):
  - `python custom_components\mpu6050\mpuAngle_test.py`
  - `python custom_components\mpu6050\MPU6050_cal.py`
- Single-test equivalent in this codebase is running one script directly, e.g.:
  - `python custom_components\mpu6050\mpuAngle_test.py`

## High-level architecture

- This is a Home Assistant custom integration under `custom_components\mpu6050`.
- `__init__.py` creates one `MPU6050Device` per config entry, stores it on `entry.runtime_data`, forwards platform setup to `sensor` and `switch`, and starts/stops the device loop.
- `config_flow.py` auto-discovers I2C buses (`/dev/i2c-*`) and probes MPU6050 addresses (`0x68`, `0x69`), then stores selected bus/address in config entry data.
- `sensor.py` and `switch.py` are thin wrappers that expose entities already created by `MPU6050Device`.
- `MPU6050Device.py` is the integration runtime core:
  - owns all entities (acceleration, min/max acceleration, pitch, roll, temperature, plus enabled switch)
  - runs an async read loop with retry/backoff
  - applies configurable roll/pitch correction and target interval averaging
  - shares a per-bus `asyncio.Lock` via `hass.data` (key from options) to serialize I2C access across components
- `MPU6050.py` and `MPUConstants.py` are vendored low-level sensor driver code/constants used by the runtime.

## Key conventions in this codebase

- Keep Home Assistant platform files (`sensor.py`, `switch.py`) lightweight; put device behavior in `MPU6050Device`.
- Use `entry.runtime_data` for passing integration runtime state between setup and platform modules.
- Entity IDs/names are bus+address scoped; follow existing unique ID pattern: `<entry_id>_<metric_key>`.
- Non-polled sensor updates are push-based (`should_poll = False` with `schedule_update_ha_state()` after updates).
- New I2C operations should respect the shared lock in `hass.data[option_i2c_locks_key][bus]` to avoid bus contention.
- Config entry data is for selected hardware (`bus`, `address`); tunables belong in options (`roll_offset`, `pitch_offset`, `target_interval`, `i2c_locks`).
