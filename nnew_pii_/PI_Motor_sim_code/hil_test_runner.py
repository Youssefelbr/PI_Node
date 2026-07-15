#!/usr/bin/env python3
"""
HIL Runner — STM32 Speed Regulator Validation (CAN / SocketCAN)

Uses the motor model from pi_motor_sim1erordre.py.
Runs a YAML-defined test suite, captures data every capture_interval_ms,
and writes a CSV log + Markdown report per test case.

Usage:
    python3 hil_runner.py tests/speed_regulator.yaml

Dependencies:
    pip install python-can pyyaml
    (SocketCAN kernel driver must be up: sudo ip link set can0 up type can bitrate 500000)
"""

import sys
import os
import time
import csv
import yaml
from collections import deque
from datetime import datetime
import socket
import json
UDP_IP = "192.168.11.107"   # IP du PC sur le réseau local
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_speed(speed,lead_present,vset_):
    msg = json.dumps({                                                                                                                                                 
          "speed": speed,
          "lead_present": lead_present,
          "setpoint_speed":vset_,
          "speed_reg":      True,          # ← manquant
                                            }).encode("utf-8")
    sock.sendto(msg, (UDP_IP, UDP_PORT))


# Reuse the CAN helpers from the motor simulator — single source of truth
from can_interface import read_accel_int16, send_vset_vv_u16, send_radar_frame

# Physical clamp for lead_accel_mps2 (tire/road adhesion limit for a passenger car)
MAX_LEAD_ACCEL_MPS2 = 9.0


# ---------------------------------------------------------------------------
# Motor model
# ---------------------------------------------------------------------------

def motor_step(a_cmd, a_eff, v, delay_line, tau, k_drag, Ts):
    """
    Advance the motor model by one tick (Ts seconds).

    Inputs:
        a_cmd      : acceleration command from STM32 this tick (float, ±500)
        a_eff      : current effective acceleration (filter state)
        v          : current speed (km/h)
        delay_line : deque representing the transport delay
        tau        : inertia time constant (seconds)
        k_drag     : drag coefficient
        Ts         : loop period (seconds)

    Returns:
        (a_eff, v) : updated filter state and speed
    """
    delay_line.append(a_cmd)
    a_delayed = delay_line[0]

    alpha = min(Ts / tau, 1.0)
    a_eff = a_eff + alpha * (a_delayed - a_eff)

    v = v + (a_eff - k_drag * v) * Ts
    if v < 0.0:
        v = 0.0

    return a_eff, v


# ---------------------------------------------------------------------------
# Core test-case runner
# ---------------------------------------------------------------------------

def run_test_case(bus, tc: dict, cfg: dict, motor_params: dict):
    """
    Run one test case using the same motor physics as pi_motor_sim1erordre.MOTOR().

    Returns (log_rows, assertions).
      log_rows   : list of dicts — t_ms, phase_vset, vv_sent, cmd, vv_model
      assertions : list of (description_str, passed_bool)
    """
    Ts = cfg["loop_period_ms"] / 1000.0
    capture_every = cfg["capture_interval_ms"] // cfg["loop_period_ms"]

    # Motor state
    tau         = motor_params["tau_ms"] / 1000.0
    k_drag      = motor_params["k_drag"]
    delay_steps = max(1, int(round(motor_params["delay_ms"] / cfg["loop_period_ms"])))
    delay_line  = deque([0.0] * delay_steps, maxlen=delay_steps)
    a_eff = 0.0
    v     = float(tc["initial_vv"])

    log_rows   = []
    assertions = []
    tick  = 0
    t_ms  = 0
    gap_cm = None   # carried across phases when a phase omits initial_distance_cm
    v_lead_matched_prev = False   # true if v == v_lead on the previous tick

    # Drain stale CAN messages before starting this test case
    import can as _can
    while bus.recv(timeout=0) is not None:
        pass

    for phase_idx, phase in enumerate(tc["phases"]):
        vset      = phase.get("vset", 0)
        vset_u16  = int(vset)
        phase_ticks = phase["duration_ms"] // cfg["loop_period_ms"]
        radar      = phase.get("radar", {})
        lead_pres  = radar.get("lead_present", False)
        v_lead_matched_prev = False   # reset the confirmation window on each new phase
        if "initial_distance_cm" in radar:
            gap_cm = float(radar["initial_distance_cm"])

        # ---- Lead speed: constant, or ramped over the phase ----
        # Ramp can be driven either by:
        #   - lead_speed_kmh_start + lead_speed_kmh_end (legacy: linear over the whole phase duration)
        #   - lead_speed_kmh_start + lead_accel_mps2 (slope in m/s², decoupled from duration_ms;
        #     if lead_speed_kmh_end is also given, the ramp plateaus there instead of overshooting)
        ramp_start = radar.get("lead_speed_kmh_start")
        ramp_end   = radar.get("lead_speed_kmh_end")
        accel_mps2 = radar.get("lead_accel_mps2")
        is_ramp    = ramp_start is not None and (ramp_end is not None or accel_mps2 is not None)
        if is_ramp:
            ramp_start = float(ramp_start)
            ramp_end   = float(ramp_end) if ramp_end is not None else None
            v_lead     = ramp_start
        else:
            v_lead = float(radar.get("lead_speed_kmh", 0))

        if accel_mps2 is not None:
            accel_mps2 = max(-MAX_LEAD_ACCEL_MPS2, min(MAX_LEAD_ACCEL_MPS2, float(accel_mps2)))
            rate_kmh_s = accel_mps2 * 3.6

        for phase_tick in range(phase_ticks):
            t_next = time.monotonic() + Ts

            if is_ramp:
                if accel_mps2 is not None:
                    elapsed_s = (phase_tick * cfg["loop_period_ms"]) / 1000.0
                    v_lead    = ramp_start + rate_kmh_s * elapsed_s
                    if ramp_end is not None:
                        v_lead = min(v_lead, ramp_end) if rate_kmh_s >= 0 else max(v_lead, ramp_end)
                    v_lead = max(v_lead, 0.0)
                else:
                    frac   = phase_tick / max(phase_ticks - 1, 1)
                    v_lead = ramp_start + (ramp_end - ramp_start) * frac

            # ---- Lead vehicle gap dynamics (1 km/h = 0.2778 cm per 10 ms tick) ----
            if lead_pres and gap_cm is not None:
                gap_cm += (int(v_lead) - int(v)) * (Ts/3.6) * 100.0*0.8
                gap_cm  = max(gap_cm, 0.0)

            d_send = int(gap_cm) if (lead_pres and gap_cm is not None) else 0
            send_radar_frame(bus, d_send, int(v_lead), lead_pres)

            vv_u16 = int(v)#int() previously
            if vv_u16 > 65535:
                vv_u16 = 65535

            # ---- Step 3 : send [VSET, VV] to STM32 via CAN 0x100 ----
            send_vset_vv_u16(bus, vset_u16, vv_u16)
            #UDP send datagramme """"""""""""""""""""""""""""""""""""""""
            send_speed(vv_u16,lead_pres,vset_u16)




            #UDP end sending
            #-------------------------paliative : 
            # ---- Step 1 : read CMD from STM32 (response to previous 0x100) ----
            a_cmd = read_accel_int16(bus)
            if a_cmd is None:
                a_cmd = float(delay_line[-1])   # keep last value on timeout
            a_cmd = float(a_cmd)

            # ---- Step 2 : motor model update --------
            a_eff, v = motor_step(a_cmd, a_eff, v, delay_line, tau, k_drag, Ts)

            #-------------------------paliative end
            #changement de read accel avec la position en au ------->1
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
                    "t_ms":         t_ms,
                    "phase_vset":   vset,
                    "vv_sent":      vv_u16,
                    "cmd":          int(a_cmd),
                    "vv_model":     int(v),
                    "gap_cm":       round(gap_cm, 1) if gap_cm is not None else "",
                    "lead_present": int(lead_pres),
                    "lead_speed_kmh": v_lead if lead_pres else "",
                })

            tick  += 1
            t_ms  += cfg["loop_period_ms"]

            sleep_s = t_next - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

        # ---- End-of-phase assertion -----------------------------------------
        if "assert_end" in phase:
            ae = phase["assert_end"]
            if "vv_min" in ae:
                vv_end = int(v)
                passed = ae["vv_min"] <= vv_end <= ae["vv_max"]
                assertions.append((
                    f"Phase {phase_idx + 1} end VV={vv_end} ∈ [{ae['vv_min']}, {ae['vv_max']}]",
                    passed,
                ))
            if "gap_min" in ae:
                gap_end = gap_cm if gap_cm is not None else 0.0
                passed = ae["gap_min"] <= gap_end <= ae["gap_max"]
                assertions.append((
                    f"Phase {phase_idx + 1} end GAP={round(gap_end, 1)} ∈ [{ae['gap_min']}, {ae['gap_max']}]",
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
            f, fieldnames=["t_ms", "phase_vset", "vv_sent", "cmd", "vv_model", "gap_cm", "lead_present", "lead_speed_kmh"]
        )
        writer.writeheader()
        writer.writerows(log_rows)


def write_global_csv(results: list, path: str) -> None:
    """Write a single CSV combining every test case's rows, tagged with tc_id/tc_name."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fieldnames = [
        "tc_id", "tc_name", "t_ms", "phase_vset", "vv_sent", "cmd",
        "vv_model", "gap_cm", "lead_present", "lead_speed_kmh",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            tc = r["tc"]
            for row in r["log_rows"]:
                writer.writerow({"tc_id": tc["id"], "tc_name": tc["name"], **row})


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
                f"### Phase {pi + 1} — VSET = {phase.get('vset', 'N/A')} km/h "
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
        import can as _can
    except ImportError:
        print("ERROR: python-can not installed. Run: pip install python-can")
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
    print(f"Interface : {cfg['can_interface']} @ {cfg['bitrate']} bps")
    print(f"Loop : {cfg['loop_period_ms']} ms | Capture : {cfg['capture_interval_ms']} ms")
    print(f"Motor: tau={motor_params['tau_ms']}ms  delay={motor_params['delay_ms']}ms  k_drag={motor_params['k_drag']}")
    print()

    results = []

    with _can.interface.Bus(cfg["can_interface"], bustype="socketcan") as bus:
        for tc in test_cases:
            print(f"--- {tc['id']}: {tc['name']} ---")
            print(f"    initial_vv={tc['initial_vv']}  phases={len(tc['phases'])}")

            log_rows, assertions = run_test_case(bus, tc, cfg, motor_params)
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

    global_csv_path = os.path.join(
        cfg["logs_dir"], f"log_ALL_{suite_name}_{timestamp}.csv"
    )
    write_global_csv(results, global_csv_path)
    print(f"Global CSV: {global_csv_path}")

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
