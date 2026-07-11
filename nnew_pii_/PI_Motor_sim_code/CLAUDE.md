# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Hardware-in-Loop (HIL)** motor speed regulation system with **Adaptive Cruise Control (ACC)**.
An STM32F401 microcontroller runs two independent controllers validated against a 1st-order motor simulation on a Raspberry Pi. Communication uses **CAN bus** via MCP2515 SPI-CAN modules.

**Node roles:**
- **STM32F401** (`SHIL_PERIPH_TESTING/`) — Speed controller + ACC firmware (10 ms interrupt loop via TIM11)
- **Raspberry Pi** (`PI_Motor_sim_code/`) — Motor simulator + HIL test runner (CAN bus via MCP2515)

---

## Repository Layout

```
STM32_PI_communication/
├── PI_Motor_sim_code/
│   ├── can_interface.py               # CAN helpers: read_accel_int16, send_vset_vv_u16, send_radar_frame
│   ├── hil_test_runner.py             # HIL test runner — imports from can_interface
│   ├── plot_results.py                # Result visualiser — plots CSV logs
│   ├── tests/
│   │   ├── test_suite_speed_acc.yaml  # Speedlimiter baseline + ACC mixed scenarios
│   │   ├── test_suite_acc_only.yaml   # ACC-only suite (gap assertions)
│   │   └── test_suite_acc_calibrated.yaml
│   ├── reports/                       # Auto-generated Markdown reports (timestamped)
│   └── logs/                          # Auto-generated CSV logs (timestamped)
└── SHIL_PERIPH_TESTING/               # STM32 firmware project (STM32CubeIDE / Eclipse)
    ├── Core/Src/main.c                 # Application logic: acc_controller(), speedlimiter(), TIM11 ISR
    ├── Core/Inc/
    ├── Debug/makefile                  # GNU Make build script
    ├── SHIL_PERIPH_TESTING.ioc        # STM32CubeMX peripheral config
    └── STM32F401RETX_FLASH.ld         # Linker script
```

---

## Build — STM32 Firmware

The firmware targets **STM32F401RETx** (Cortex-M4, 84 MHz SYSCLK, 512 KB Flash, 96 KB RAM).

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
sudo ip link set can0 up type can bitrate 500000
ip link show can0   # confirm interface is UP
```

### Install dependencies
```bash
sudo apt update && sudo apt install python3-pip python3-yaml
pip install python-can matplotlib pandas
```

### Run HIL test suite
```bash
cd PI_Motor_sim_code
python3 hil_test_runner.py tests/test_suite_acc_only.yaml
# Outputs: logs/log_<TC>_<suite>_<timestamp>.csv
#          reports/report_<suite>_<timestamp>.md
```

### Plot results
```bash
python3 plot_results.py                          # plots latest CSV in logs/
python3 plot_results.py logs/log_TC_001_*.csv    # specific file
```

---

## CAN Communication Protocol

| Direction | CAN ID | Data | Meaning |
|-----------|--------|------|---------|
| Pi → STM32 | `0x100` | 4 bytes | Bytes 0–1 = VSET (uint16 LE), Bytes 2–3 = VV (uint16 LE) |
| Pi → STM32 | `0x300` | 5 bytes | Bytes 0–1 = distance_cm (uint16 LE), Bytes 2–3 = lead_speed_kmh (uint16 LE), Byte 4 = flags (bit0 = lead_present) |
| STM32 → Pi | `0x200` | 2 bytes | Acceleration command (int16 LE, range ±500) |

- **Interface:** SocketCAN `can0`, 500 kbps (MCP2515 SPI-CAN module)
- **Pi receive timeout:** 8 ms per `read_accel_int16` call
- **STM32 control loop:** TIM11 fires every 10 ms (`Prescaler=8399, Period=99`, APB2=84 MHz)

---

## Key Application Code — STM32 (`Core/Src/main.c`)

### Clock configuration
PLL source: HSI 16 MHz → PLLM=8 → PLLN=84 → PLLP=2 → **SYSCLK = 84 MHz**, APB2 = 84 MHz.
TIM11 timer: `(8399+1) × (99+1) / 84 MHz = 10 ms` ✅

### TIM11 ISR
```c
if (htim->Instance == TIM11) {
    acc_controller(vset, vv);   // always called; routes to speedlimiter when no lead
    tx_ready = 1;
}
```

### Main loop (CAN receive)
```c
// 0x100 → vset, vv
// 0x300 → acc.distance_cm, acc.lead_speed_kmh, acc.lead_present
```

---

### `speedlimiter(vset, vv)` — pure speed P-controller

Called by `acc_controller` when **no lead vehicle is present**.

1. Deadband ±1 on error
2. P-controller: `cmd = (vset - vv) * 2`
3. Saturation: clamped to `±500`
4. EMA: `cmd_f = 0.8 × cmd_f + 0.2 × cmd` (alpha=0.2)
5. Packs `cmd_f` as int16 LE into `txMessage` (CAN `0x200`)

**Static state:** `cmd_f` is `static float` — persists across test cases within a power cycle.

---

### `acc_controller(vset_in, vv_in)` — controller dispatcher

**Architecture**: two fully isolated controllers. Only one is active per tick.

```
lead_present = false  →  speedlimiter(vset_in, vv_in)   [speed regulation]
lead_present = true   →  ACC distance P-controller       [gap regulation, ignores vset]
```

**Transition management** (via `static uint8_t prev_lead_present`):
- ACC → speedlimiter: `cmd_f = 0.0f` reset to avoid EMA transient
- speedlimiter → ACC: `cmdd_f = 0.0f` reset to avoid EMA transient

**ACC distance P-controller (lead present):**
```c
d_target_cm = (vv_in * 550) / 90          // safety distance [cm], scales with ego speed
e_d         = distance_cm - d_target_cm   // distance error [cm]
cmdd        = (e_d * 2) / 100             // P gain = 0.02 cmd/cm
cmdd        = clamp(cmdd, -500, 500)
cmdd_f      = 0.8*cmdd_f + 0.2*cmdd      // EMA alpha=0.2
```
Output packed as int16 LE into `txMessage` (CAN `0x200`). **`vset` is not used when ACC is active.**

**d_target reference values:**

| vv (km/h) | d_target (cm) |
|-----------|---------------|
| 60        | 366           |
| 70        | 427           |
| 80        | 488           |
| 90        | 549           |
| 100       | 611           |

**Effective dead zone:** ±50 cm (integer division: `e_d * 2 / 100` rounds to 0 for `|e_d| < 50 cm`).

---

## HIL Test Framework (`hil_test_runner.py`)

Closed-loop runner using `python-can` + SocketCAN. Per-tick loop (10 ms):
1. Update lead vehicle gap dynamics: `gap += (v_lead - v_ego) × 0.2778` cm/tick
2. `send_radar_frame(bus, gap, v_lead, lead_present)` — CAN `0x300`
3. `send_vset_vv_u16(bus, vset, vv)` — CAN `0x100`
4. `read_accel_int16(bus)` — read CMD from STM32 via CAN `0x200`
5. Motor model update: delay line → 1st-order inertia → integrate velocity

### YAML schema

```yaml
suite:   {name, version, dut, description}
config:  {can_interface, bitrate, loop_period_ms, capture_interval_ms, reports_dir, logs_dir,
          motor: {tau_ms, delay_ms, k_drag}}
test_cases:
  - id: TC_ACC_001
    initial_vv: 80
    phases:
      - vset: 100
        duration_ms: 10000
        radar:
          lead_present: true
          lead_speed_kmh: 80
          initial_distance_cm: 488
        assert_end: {gap_min: 300, gap_max: 650}   # ACC phase → check gap

      - vset: 100
        duration_ms: 8000
        radar:
          lead_present: false
        assert_end: {vv_min: 88, vv_max: 108}       # speedlimiter phase → check speed
```

### Assertion logic (end of phase)

| Keys present in `assert_end` | What is checked |
|------------------------------|-----------------|
| `gap_min / gap_max` | `gap_cm` at end of phase — used for **ACC phases** (`lead_present: true`) |
| `vv_min / vv_max` | `vv_model` at end of phase — used for **speedlimiter phases** (`lead_present: false`) |

### CSV log columns
`t_ms, phase_vset, vv_sent, cmd, vv_model, gap_cm, lead_present`

### Test suites

| File | Purpose |
|------|---------|
| `test_suite_acc_only.yaml` | ACC-only validation (gap assertions) |
| `test_suite_speed_acc.yaml` | Speedlimiter baseline (TC_001–TC_004) + mixed ACC scenarios |

---

## Known Limitations

- **EMA persists across test cases** — `cmd_f` and `cmdd_f` are `static`. Power-cycle STM32 between runs for clean initial state.
- **ACC dead zone ±50 cm** — `(e_d * 2) / 100` integer division: errors below 50 cm produce zero command.
- **ACC gain is low (0.02)** — for typical distance errors of 100–300 cm, cmd is 2–6. Response is slow; use long phase durations (≥8 s) in test scenarios.
- **No speed cap in ACC mode** — when `lead_present=true`, `vset` is ignored. If `gap >> d_target`, the ACC commands positive acceleration with no upper speed limit.
- **CAN interface must be up** — run `sudo ip link set can0 up type can bitrate 500000` before the runner.
- **Pi loop not hardware-timed** — `time.sleep()` pacing; actual loop time can vary ±2–5 ms under load.

---

## STM32CubeMX Configuration

Open `SHIL_PERIPH_TESTING.ioc` to view/modify peripheral config. User logic is preserved in `USER CODE BEGIN/END` blocks after regeneration.

Enabled peripherals: TIM11, SPI1 (MCP2515 CAN), USART2 (debug 115200 baud), GPIO (SPI_CS, LED), RCC, NVIC.
