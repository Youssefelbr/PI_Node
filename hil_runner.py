#!/usr/bin/env python3
"""
HIL Runner — STM32 Speed Regulator Validation

Uses the motor model (MOTOR function) from pi_motor_sim1erordre.py directly.
Runs a YAML-defined test suite, captures data every capture_interval_ms,
and writes a CSV log + Markdown report per test case.

Usage:
    python3 hil_runner.py tests/speed_regulator.yaml

Dependencies:
    pip install pyserial pyyaml
"""

import sys
import os
import time
import csv
import yaml
from collections import deque
from datetime import datetime

# Reuse the UART helpers from the motor simulator — single source of truth
from pi_motor_sim1erordre import read_accel_int16, send_vset_vv_u8


# ---------------------------------------------------------------------------
# Core test-case runner
# ---------------------------------------------------------------------------

def run_test_case(ser, tc: dict, cfg: dict, motor_params: dict):
    """
    Run one test case using the same motor physics as pi_motor_sim1erordre.MOTOR().

    Returns (log_rows, assertions).
      log_rows   : list of dicts — t_ms, phase_vset, vv_sent, cmd, vv_model
      assertions : list of (description_str, passed_bool)
    """
    Ts = cfg["loop_period_ms"] / 1000.0
    capture_every = cfg["capture_interval_ms"] // cfg["loop_period_ms"]

    # Motor state — identical initialisation to MOTOR()
    tau      = motor_params["tau_ms"] / 1000.0
    k_drag   = motor_params["k_drag"]
    delay_steps = max(1, int(round(motor_params["delay_ms"] / cfg["loop_period_ms"])))
    delay_line  = deque([0.0] * delay_steps, maxlen=delay_steps)
    a_eff = 0.0
    v     = float(tc["initial_vv"])

    log_rows   = []
    assertions = []
    tick  = 0
    t_ms  = 0

    for phase_idx, phase in enumerate(tc["phases"]):
        vset     = phase["vset"]
        vset_u8  = vset & 0xFF
        phase_ticks = phase["duration_ms"] // cfg["loop_period_ms"]

        for phase_tick in range(phase_ticks):
            t_next = time.monotonic() + Ts

            # ---- Step 1 : read accel from STM32 (same order as MOTOR()) ----
            a_cmd = read_accel_int16(ser)
            if a_cmd is None:
                a_cmd = float(delay_line[-1])   # keep last value on timeout
            a_cmd = float(a_cmd)

            # ---- Step 2 : motor model update (identical to MOTOR()) --------
            delay_line.append(a_cmd)
            a_delayed = delay_line[0]

            alpha = Ts / tau
            if alpha > 1.0:
                alpha = 1.0
            a_eff = a_eff + alpha * (a_delayed - a_eff)

            v = v + (a_eff - k_drag * v) * Ts
            if v < 0.0:
                v = 0.0

            vv_u8 = int(v)
            if vv_u8 > 255:
                vv_u8 = 255

            # ---- Step 3 : send [VSET, VV] (same call as MOTOR()) -----------
            send_vset_vv_u8(ser, vv_u8, vset_u8)

            # ---- assert_cmd_at_ms ------------------------------------------
            if "assert_cmd_at_ms" in phase:
                t_in_phase_ms = phase_tick * cfg["loop_period_ms"]
                if t_in_phase_ms == phase["assert_cmd_at_ms"]:
                    expected  = phase["expected_cmd"]
                    tolerance = phase.get("cmd_tolerance", 0)
                    cmd_int   = int(a_cmd)
                    passed    = abs(cmd_int - expected) <= tolerance
                    assertions.append((
                        f"Phase {phase_idx + 1} CMD@{t_in_phase_ms}ms: "
                        f"expected {expected}±{tolerance}, got {cmd_int}",
                        passed,
                    ))

            # ---- Capture snapshot ------------------------------------------
            if tick % capture_every == 0:
                log_rows.append({
                    "t_ms":       t_ms,
                    "phase_vset": vset,
                    "vv_sent":    vv_u8,
                    "cmd":        int(a_cmd),
                    "vv_model":   round(v, 2),
                })

            tick  += 1
            t_ms  += cfg["loop_period_ms"]

            sleep_s = t_next - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

        # ---- End-of-phase assertion -----------------------------------------
        if "assert_end" in phase:
            ae     = phase["assert_end"]
            vv_end = int(v)
            passed = ae["vv_min"] <= vv_end <= ae["vv_max"]
            assertions.append((
                f"Phase {phase_idx + 1} end VV={vv_end} ∈ [{ae['vv_min']}, {ae['vv_max']}]",
                passed,
            ))

    return log_rows, assertions


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

def write_csv(log_rows: list, path: str) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f, fieldnames=["t_ms", "phase_vset", "vv_sent", "cmd", "vv_model"]
        )
        writer.writeheader()
        writer.writerows(log_rows)


def write_report(suite_meta, motor_params, results, timestamp, path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    lines = [
        f"# HIL Test Report — {suite_meta['name']}",
        "",
        f"Date: {timestamp}",
        f"DUT: {suite_meta.get('dut', 'STM32F401RETx')}",
        f"Motor model: tau={motor_params['tau_ms']}ms, "
        f"delay={motor_params['delay_ms']}ms, k_drag={motor_params['k_drag']}",
        "",
        "---",
        "",
    ]

    passed_cases = 0

    for r in results:
        tc         = r["tc"]
        log_rows   = r["log_rows"]
        assertions = r["assertions"]
        tc_passed  = all(ok for _, ok in assertions)
        if tc_passed:
            passed_cases += 1

        lines += [
            f"## {tc['id']} — {tc['name']}",
            "",
            f"Initial speed: **{tc['initial_vv']} km/h**",
            "",
        ]

        phase_start_t = 0
        for pi, phase in enumerate(tc["phases"]):
            phase_end_t  = phase_start_t + phase["duration_ms"]
            phase_rows   = [r for r in log_rows if phase_start_t <= r["t_ms"] < phase_end_t]

            lines += [
                f"### Phase {pi + 1} — VSET = {phase['vset']} km/h "
                f"({phase_start_t} ms → {phase_end_t} ms)",
                "",
                "| t (ms) | VSET | VV (km/h) | CMD (accel) |",
                "|--------|------|-----------|-------------|",
            ]
            for row in phase_rows:
                lines.append(
                    f"| {row['t_ms']} | {row['phase_vset']} | {row['vv_sent']} | {row['cmd']} |"
                )
            lines.append("")
            phase_start_t = phase_end_t

        for desc, ok in assertions:
            lines.append(f"Assertion: {desc} → {'✅ PASS' if ok else '❌ FAIL'}")
        lines.append("")

        lines += [f"### {tc['id']} Result: {'✅ PASS' if tc_passed else '❌ FAIL'}", "", "---", ""]

    total = len(results)
    symbol = "✅ PASS" if passed_cases == total else "❌ FAIL"
    lines.append(f"## Overall Result: {symbol} ({passed_cases}/{total} test cases)")

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 hil_runner.py <test_suite.yaml>")
        sys.exit(1)

    try:
        import serial as _serial
    except ImportError:
        print("ERROR: pyserial not installed. Run: pip install pyserial pyyaml")
        sys.exit(1)

    with open(sys.argv[1], encoding="utf-8") as f:
        doc = yaml.safe_load(f)

    suite_meta   = doc["suite"]
    cfg          = doc["config"]
    motor_params = cfg["motor"]
    test_cases   = doc["test_cases"]
    timestamp    = datetime.now().strftime("%Y%m%d_%H%M%S")
    suite_name   = suite_meta["name"]

    print(f"=== HIL Runner — {suite_name} ===")
    print(f"Port : {cfg['port']} @ {cfg['baud']} baud")
    print(f"Loop : {cfg['loop_period_ms']} ms | Capture : {cfg['capture_interval_ms']} ms")
    print(f"Motor: tau={motor_params['tau_ms']}ms  delay={motor_params['delay_ms']}ms  k_drag={motor_params['k_drag']}")
    print()

    results = []

    with _serial.Serial(cfg["port"], cfg["baud"], timeout=0.025) as ser:
        ser.reset_input_buffer()

        for tc in test_cases:
            print(f"--- {tc['id']}: {tc['name']} ---")
            print(f"    initial_vv={tc['initial_vv']}  phases={len(tc['phases'])}")

            log_rows, assertions = run_test_case(ser, tc, cfg, motor_params)
            tc_passed = all(ok for _, ok in assertions)

            print(f"    Result: {'PASS' if tc_passed else 'FAIL'}")
            for desc, ok in assertions:
                print(f"      {'✅' if ok else '❌'} {desc}")

            csv_path = os.path.join(
                cfg["logs_dir"], f"log_{tc['id']}_{suite_name}_{timestamp}.csv"
            )
            write_csv(log_rows, csv_path)
            print(f"    CSV  : {csv_path}")
            print()

            results.append({"tc": tc, "log_rows": log_rows, "assertions": assertions})

    report_path = os.path.join(
        cfg["reports_dir"], f"report_{suite_name}_{timestamp}.md"
    )
    write_report(suite_meta, motor_params, results, timestamp, report_path)
    print(f"Report: {report_path}")

    passed = sum(1 for r in results if all(ok for _, ok in r["assertions"]))
    total  = len(results)
    print(f"\n=== Overall: {'✅ PASS' if passed == total else '❌ FAIL'} ({passed}/{total}) ===")
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
