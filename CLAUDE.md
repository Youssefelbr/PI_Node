# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Hardware-in-Loop (HIL)** motor speed regulation system. An STM32F401 microcontroller runs a P-controller, validated against a 1st-order motor simulation on a Raspberry Pi over UART. An ESP32 provides potentiometer-based setpoint input.

**Node roles:**
- **STM32F401** (`SHIL_PERIPH_TESTING/`) — Speed controller firmware (P-controller, 10ms interrupt loop)
- **Raspberry Pi** (`PI_Motor_sim_code/`, `../RASPI_NODE/`) — Motor simulator + test runner
- **ESP32** (`../../ESP32-Node/`) — Potentiometer/setpoint input device

## Repository Layout

```
STM32_PI_communication/
├── PI_Motor_sim_code/
│   ├── pi_motor_sim1erordre.py   # Motor simulator + UART helpers (MOTOR, read_accel_int16, send_vset_vv_u8)
│   ├── hil_runner.py             # HIL test runner — imports from pi_motor_sim1erordre
│   ├── tests/
│   │   └── speed_regulator.yaml  # HIL test suite (TC_001–TC_003)
│   ├── reports/                  # Auto-generated Markdown reports (timestamped)
│   └── logs/                     # Auto-generated CSV logs (timestamped)
└── SHIL_PERIPH_TESTING/          # STM32 firmware project (STM32CubeIDE / Eclipse)
    ├── Core/Src/main.c            # Application logic: speedlimiter(), control ISR
    ├── Core/Inc/
    ├── Debug/makefile             # GNU Make build script
    ├── SHIL_PERIPH_TESTING.ioc   # STM32CubeMX peripheral config
    └── STM32F401RETX_FLASH.ld    # Linker script

../RASPI_NODE/
├── pyrunnner.py                   # Legacy YAML-driven UART/GPIO test automation
├── TEST.YAML / testi.yaml         # Legacy test scenario definitions
├── UART_SPEED.py                  # Speed variation simulator
└── graphplot.py                   # Sigrok/PulseView CSV plotter
```

## Build — STM32 Firmware

The firmware targets **STM32F401RETx** (Cortex-M4, 72 MHz, 512 KB Flash, 96 KB RAM).

```bash
# From a Linux/WSL shell with arm-none-eabi-gcc on PATH
cd SHIL_PERIPH_TESTING/Debug
make clean
make
# Outputs: SHIL_PERIPH_TESTING.elf, .map, .list
```

**Toolchain:** `GNU Tools for STM32 13.3.rel1` (`arm-none-eabi-gcc`)
**Defines:** `USE_HAL_DRIVER`, `STM32F401xE`, `DEBUG`
**FPU:** hard float (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`)

Flashing is done via STM32CubeIDE or ST-Link (not automated in this repo).

## Run — Raspberry Pi Scripts

```bash
# Motor simulation (requires STM32 connected on /dev/serial0, ESP32 on /dev/ttyUSB0)
python3 pi_motor_sim1erordre.py

# HIL test runner (preferred) — runs full test suite, writes CSV + Markdown report
cd STM32_PI_communication/PI_Motor_sim_code
pip install pyserial pyyaml   # one-time
python3 hil_runner.py tests/speed_regulator.yaml

# Legacy YAML-driven test (GPIO stimulus + single-shot verdict)
python3 pyrunnner.py   # reads TEST.YAML in the same directory
```

## UART Communication Protocol

| Direction | Bytes | Meaning |
|-----------|-------|---------|
| Pi → STM32 | `[VSET, VV]` | Speed setpoint (u8), measured velocity (u8) |
| STM32 → Pi | `[LSB, MSB]` | Acceleration command (int16_t, range ±500) |

- **Port:** USART1 on STM32 (`/dev/serial0` on Pi), 115200 8N1
- **Timing:** STM32 control loop fires every ~10 ms (TIM11 interrupt)

## Key Application Code

**`Core/Src/main.c`** — All application logic lives here:
- `speedlimiter(VSET, VV)` — P-controller: `cmd = (VSET - VV) * 2`, clamped to ±500
- `HAL_TIM_PeriodElapsedCallback()` — TIM11 ISR at 10 ms; reads 2-byte UART Rx, calls `speedlimiter()`, transmits 2-byte result
- Peripherals used: USART1 (Pi link), USART2 (debug), TIM11 (control timer), GPIO PA1

**`PI_Motor_sim_code/pi_motor_sim1erordre.py`** — Motor simulator and shared UART helpers:
- Time constant `tau = 0.100 s`, delay `Td = 0.250 s`, timestep `Ts = 0.010 s`
- `read_accel_int16(ser)` — reads 2-byte int16 LE from STM32 (acceleration command)
- `send_vset_vv_u8(ser, vset_u8, vv_u8)` — writes `bytes([vset_u8, vv_u8])` to STM32
- `MOTOR(ser_stm32, poll_vset_u8, ...)` — full real-time motor loop (used standalone with ESP32)
- **`hil_runner.py` imports `read_accel_int16` and `send_vset_vv_u8` from this file** — single source of truth for UART helpers

## HIL Test Framework (`hil_runner.py`) — current

Closed-loop runner that reuses `pi_motor_sim1erordre.py` for all motor and UART logic.
Per-tick loop order (matches `MOTOR()` exactly):
1. `read_accel_int16(ser)` — read CMD int16 from STM32 (or keep last value on timeout)
2. Update motor model: delay line → 1st-order inertia filter → integrate velocity
3. `send_vset_vv_u8(ser, vv_u8, vset_u8)` — send current VV + active setpoint to STM32

A data row is captured every `capture_interval_ms` (default 100 ms). Phase assertions are checked at the end of each phase (or at a specific tick for `assert_cmd_at_ms`).

**YAML schema** (`tests/speed_regulator.yaml`):
```yaml
suite:   {name, version, dut, description}
config:  {port, baud, loop_period_ms, capture_interval_ms, reports_dir, logs_dir,
          motor: {tau_ms, delay_ms, k_drag}}
test_cases:
  - id: TC_001
    initial_vv: 10
    phases:
      - vset: 100
        duration_ms: 3000
        assert_end: {vv_min: 85, vv_max: 105}
      - vset: 160
        duration_ms: 3000
        assert_end: {vv_min: 140, vv_max: 165}
```

Supported phase assertions:
- `assert_end: {vv_min, vv_max}` — check VV at end of phase
- `assert_cmd_at_ms / expected_cmd / cmd_tolerance` — check CMD at a specific tick

**Output files** (timestamped):
- `logs/log_<TC_ID>_<suite>_<timestamp>.csv` — columns: `t_ms, phase_vset, vv_sent, cmd, vv_model`
- `reports/report_<suite>_<timestamp>.md` — Markdown table per phase + pass/fail verdicts

**Import relationship:** `hil_runner.py` does `from pi_motor_sim1erordre import read_accel_int16, send_vset_vv_u8`. Both files must be in the same directory (`PI_Motor_sim_code/`).

**Note on `send_vset_vv_u8` call convention:** in `pi_motor_sim1erordre.py` the `MOTOR()` function calls `send_vset_vv_u8(ser, vv_u8, vset_u8)` (velocity first, setpoint second — reversed from the function signature). `hil_runner.py` replicates this exact call to stay consistent with the existing behaviour.

## Legacy Test Framework (`pyrunnner.py`)

Tests are defined in YAML with timestamped stimulus steps:

```yaml
test_name: Test_UART_5_Steps
steps:
  - time_ms: 0
    gpio: 1
    uart: "VAL,1"
  - time_ms: 100
    gpio: 0
    uart: "VAL,2"
```

Runner phases: emit GPIO/UART stimuli → wait 250 ms → read STM32 response → compare expected vs. measured → pass/fail verdict.

## STM32CubeMX Configuration

Open `SHIL_PERIPH_TESTING.ioc` in STM32CubeMX to view/modify peripheral config. After regenerating code, user logic is preserved in `USER CODE BEGIN/END` blocks.

Enabled peripherals: TIM11, USART1, USART2, GPIO (PA1 input), RCC, NVIC.
