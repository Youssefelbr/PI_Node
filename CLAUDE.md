# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Hardware-in-Loop (HIL)** motor speed regulation system. An STM32F401 microcontroller runs a P-controller, validated against a 1st-order motor simulation on a Raspberry Pi. Communication between the two nodes uses **CAN bus** via an MCP2515 SPI-CAN module on each side.

**Node roles:**
- **STM32F401** (`SHIL_PERIPH_TESTING/`) — Speed controller firmware (P-controller + EMA filter, 10 ms interrupt loop)
- **Raspberry Pi** (`PI_Motor_sim_code/`) — Motor simulator + HIL test runner (CAN bus via MCP2515)

---

## Repository Layout

```
STM32_PI_communication/
├── PI_Motor_sim_code/
│   ├── pi_motor_sim1erordre.py   # CAN helpers: read_accel_int16, send_vset_vv_u16
│   ├── hil_runner.py             # HIL test runner — imports from pi_motor_sim1erordre
│   ├── plot_results.py           # Result visualiser — plots CSV logs (speed + CMD)
│   ├── tests/
│   │   └── speed_regulator.yaml  # HIL test suite (TC_001–TC_004)
│   ├── reports/                  # Auto-generated Markdown reports (timestamped)
│   └── logs/                     # Auto-generated CSV logs (timestamped)
└── SHIL_PERIPH_TESTING/          # STM32 firmware project (STM32CubeIDE / Eclipse)
    ├── Core/Src/main.c            # Application logic: speedlimiter(), TIM11 ISR
    ├── Core/Inc/
    ├── Debug/makefile             # GNU Make build script
    ├── SHIL_PERIPH_TESTING.ioc   # STM32CubeMX peripheral config
    └── STM32F401RETX_FLASH.ld    # Linker script

```

---

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

---

## Run — Raspberry Pi Scripts

### One-time CAN interface setup
```bash
# Load MCP2515 SPI-CAN driver (add to /boot/config.txt for persistence)
sudo ip link set can0 up type can bitrate 500000
ip link show can0   # confirm interface is UP
```

### Install dependencies
```bash
sudo apt update
sudo apt install python3-pip python3-yaml
pip install python-can matplotlib pandas
```

### Run HIL test suite (preferred)
```bash
cd PI_Motor_sim_code          # hil_runner.py and pi_motor_sim1erordre.py must be in same dir
python3 hil_runner.py tests/speed_regulator.yaml
# Outputs: logs/log_<TC>_<suite>_<timestamp>.csv
#          reports/report_<suite>_<timestamp>.md
```

### Plot results
```bash
python3 plot_results.py                         # plots latest CSV in logs/
python3 plot_results.py logs/log_TC_001_*.csv   # specific file
python3 plot_results.py logs/tc1.csv logs/tc2.csv  # overlay multiple
# Saves a PNG alongside the first input CSV
```

---

## CAN Communication Protocol

| Direction | CAN ID | Data | Meaning |
|-----------|--------|------|---------|
| Pi → STM32 | `0x100` | 4 bytes | Bytes 0–1 = VSET (uint16 LE), Bytes 2–3 = VV (uint16 LE) |
| STM32 → Pi | `0x200` | 2 bytes | Acceleration command (int16 LE, range ±500) |

- **Interface:** SocketCAN `can0`, 500 kbps (MCP2515 SPI-CAN module)
- **Pi receive timeout:** 25 ms per `read_accel_int16` call
- **STM32 control loop:** TIM11 fires every 10 ms (`Prescaler=7199, Period=99`, APB2=72 MHz)

---

## Key Application Code

### `Core/Src/main.c` — STM32 firmware

**TIM11 ISR (`HAL_TIM_PeriodElapsedCallback`):**
```c
if (htim->Instance == TIM11) {
    // Receive [VSET_u16_LE, VV_u16_LE] via CAN 0x100
    // speedlimiter(VSET, VV)
    // Transmit int16 LE CMD via CAN 0x200
}
```

**`speedlimiter(VSET, VV)` — current implementation:**
1. Deadband ±1 on error (suppresses jitter)
2. P-controller: `cmd = (VSET - VV) * 2`
3. Saturation: clamped to `±500`
4. **EMA smoothing:** `cmd_f = 0.8 × cmd_f + 0.2 × cmd` (alpha=0.2)
5. Output packed as int16 LE into CAN frame

**EMA impact:** The filter introduces lag on the first ~200 ms after a setpoint change. Starting from `cmd_f=0`, it takes ~15–20 ticks to reach steady-state command value. Widen `assert_end` tolerances if tests fail near phase boundaries.

**Static variable warning:** `cmd_f` is `static float` — it persists across test cases within the same STM32 power cycle. TC_002/TC_003/TC_004 will not start from `cmd_f=0`.

**Timer configuration (confirmed):**
```c
htim11.Init.Prescaler = 7200-1;   // 7200
htim11.Init.Period    = 100-1;    // 100  →  7200×100/72MHz = 10 ms ✅
```

**Peripherals:** CAN (MCP2515 via SPI), USART2 (debug), TIM11 (control timer), GPIO PA1 (input)

---

### `PI_Motor_sim_code/pi_motor_sim1erordre.py` — CAN helpers

Pure CAN communication module. Contains no motor model or standalone simulator.

```python
CAN_ID_CMD  = 0x200   # STM32 → Pi : int16 LE  (acceleration command, ±500)
CAN_ID_CTRL = 0x100   # Pi → STM32 : uint16 LE VSET | uint16 LE VV
```

- `read_accel_int16(bus)` — waits up to 25 ms for CAN `0x200`, returns `int16` or `None`
- `send_vset_vv_u16(bus, vset_u16, vv_u16)` — packs `[VSET, VV]` as 4 bytes LE and sends on CAN `0x100`
- **`hil_runner.py` imports both functions from this file** — both must be in the same directory

---

## HIL Test Framework (`hil_runner.py`)

Closed-loop runner using `python-can` + SocketCAN. Per-tick loop (10 ms):
1. `read_accel_int16(bus)` — read CMD from STM32 via CAN `0x200` (fallback to last value on timeout)
2. Motor model update: delay line (25 steps) → 1st-order inertia filter → integrate velocity
3. `send_vset_vv_u16(bus, vset_u16, vv_u16)` — send `[VSET, VV]` to STM32 via CAN `0x100`

Captures one data row every `capture_interval_ms` (default 100 ms). Runs each phase for `duration_ms`, then checks assertions. Drains stale CAN messages before each test case starts.

### YAML schema (`tests/speed_regulator.yaml`)
```yaml
suite:   {name, version, dut, description}
config:  {can_interface, bitrate, loop_period_ms, capture_interval_ms, reports_dir, logs_dir,
          motor: {tau_ms, delay_ms, k_drag}}
test_cases:
  - id: TC_001
    name: "Step_Response_Acceleration"
    initial_vv: 0
    phases:
      - vset: 100
        duration_ms: 5000
        assert_end: {vv_min: 80, vv_max: 110}
  - id: TC_004
    initial_vv: 0
    phases:
      - vset: 100
        duration_ms: 5000
        assert_end: {vv_min: 80, vv_max: 110}
        assert_cmd_at_ms: 10        # check CMD at exact tick
        expected_cmd: 500
        cmd_tolerance: 0
```

### Test cases defined
| ID | Name | initial_vv | Phases | Purpose |
|----|------|-----------|--------|---------|
| TC_001 | Step_Response_Acceleration | 0 | 0→100 km/h | Convergence from rest |
| TC_002 | Multi_Step_Ramp | 10 | 80→150 km/h | Convergence at two operating points |
| TC_003 | Deceleration_Check | 150 | 150→40 km/h | Negative CMD (braking), convergence |
| TC_004 | Setpoint_Hold_Stability | 0 | 0→100→hold 100 | Steady-state stability after convergence |

### Output files (timestamped)
- `logs/log_<TC_ID>_<suite>_<timestamp>.csv` — columns: `t_ms, phase_vset, vv_sent, cmd, vv_model`
- `reports/report_<suite>_<timestamp>.md` — Markdown table per phase + assertion verdicts

---

### `PI_Motor_sim_code/plot_results.py` — Result visualiser

Reads one or more CSV log files and renders a 2-row subplot per file:
- **Top:** VSET (step), VV model, VV sent — with phase boundary markers
- **Bottom:** CMD with ±500 saturation lines

```bash
python3 plot_results.py                         # auto-picks latest CSV
python3 plot_results.py logs/log_TC_001_*.csv   # specific file(s)
```

Saves output as `logs/plot_<original_csv_name>.png`.

**Dependencies:** `matplotlib`, `pandas`

---

## Known Limitations / Next Steps

- **EMA on STM32 persists across test cases** — `cmd_f` is static. A cold-start reset or power cycle between TC runs gives cleaner results for TC_002/TC_003/TC_004.
- **CAN interface must be up** — run `sudo ip link set can0 up type can bitrate 500000` before executing the HIL runner; otherwise `python-can` will fail to open the bus.
- **Pi loop not hardware-timed** — `time.sleep()` is used for 10 ms pacing. On a loaded Pi, actual loop time can vary ±2–5 ms.
- **`assert_cmd_at_ms`** — due to EMA (alpha=0.2), the first CMD sent by STM32 will be `~0.2×500 = 100`, not 500. Any assertion expecting 500 at tick 0 will FAIL unless `cmd_tolerance` accounts for the EMA.
- **No standalone motor simulator** — `pi_motor_sim1erordre.py` no longer contains a `MOTOR()` function; it is a pure CAN helper. The motor model now lives entirely inside `hil_runner.py`.

---

## STM32CubeMX Configuration

Open `SHIL_PERIPH_TESTING.ioc` to view/modify peripheral config. User logic is preserved in `USER CODE BEGIN/END` blocks after regeneration.

Enabled peripherals: TIM11, SPI (MCP2515 CAN), USART2, GPIO (PA1 input), RCC, NVIC.
