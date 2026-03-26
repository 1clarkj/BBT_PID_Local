# PID_Local

Repository for the Ball Balance Table PID runtime on Raspberry Pi.
This repo communicates with the gesture-ML repo `Manus_MLP` over UDP (target setpoints + position feedback).

## Current Architecture

- `src/pid_local/` contains the canonical runtime code.
- `scripts/` contains current user-facing launch scripts.
- `legacy/` contains previous script entrypoints kept for reference/backward compatibility.
- `debug/` contains debugging utilities (for example touchscreen diagnostics).

## Project Structure

```text
PID_Local/
  README.md
  requirements.txt
  pyproject.toml

  src/
    pid_local/
      __init__.py
      app.py
      planning/
        __init__.py
        occupancy_planner.py
      hardware/
        __init__.py
        table_controller.py
        servo_controller.py
        controller_not_started_error.py

  scripts/
    run_pid.py
    run_ps4.py
    manual_target_input.py
    demo_circle.py

  legacy/
    pid_control.py
    ps4_pid_controlled.py
    pid_input.py
    pid_circle_demo.py
    visibility_planner.py
    ball_balance_table_controller_v2.py
    servo_controller.py
    controller_not_started_error.py

  debug/
    touchscreen_test.py
    ip_handshake.py
    ipsource.py
    toggle_test.py
    UDPPub.py
    UDPSub.py
```

## Planner

Path planning uses an occupancy-grid A* planner with inflated wall occupancy (`src/pid_local/planning/occupancy_planner.py`) for more robust behavior around wall discontinuities.

## Raspberry Pi Setup

Raspberry Pi uses PEP 668, so run inside a virtual environment.

1. `python3 -m venv venv`
2. `source venv/bin/activate`
3. `pip install -r requirements.txt`
4. `sudo pigpiod`

## Run

Preferred commands:

1. Main PID runtime: `python3 scripts/run_pid.py`
2. PS4 control mode: `python3 scripts/run_ps4.py`
3. Manual target input mode: `python3 scripts/manual_target_input.py`
4. Circle demo mode: `python3 scripts/demo_circle.py`

Legacy commands still supported:

- `python3 legacy/pid_control.py`
- `python3 legacy/ps4_pid_controlled.py`
- `python3 legacy/pid_input.py`
- `python3 legacy/pid_circle_demo.py`

## Debug Utilities

- Touchscreen functionality test: `python3 debug/touchscreen_test.py`
- UDP debugging tools (publisher/subscriber + handshake helpers): use scripts in `debug/`

## Shutdown

1. `deactivate`
2. `sudo shutdown now`
