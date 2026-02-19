import time, struct
from collections import deque
import serial

PORT_STM32 = "/dev/serial0"
PORT_ESP32 = "/dev/ttyUSB0"
BAUD = 115200

# ========== STM32 helpers ==========
def read_accel_int16(ser):
    b = ser.read(2)
    if len(b) != 2:
        return None
    return struct.unpack("<h", b)[0]  # int16 little-endian [web:117]

def send_vset_vv_u8(ser, vset_u8, vv_u8):
    ser.write(bytes([vset_u8 & 0xFF, vv_u8 & 0xFF]))

# ========== ESP32 non-blocking reader ==========
def make_pot_reader_u8(serusb, scale=19.0, v_min=0.0, v_max=255.0):
    """
    Lit des lignes ASCII du type '1234\\n' sur serusb (non bloquant),
    convertit en vset (0..255) via raw/scale, et garde la dernière valeur.
    """
    buf = bytearray()
    last_vset = 0  # uint8

    def poll_vset_u8():
        nonlocal buf, last_vset
        n = serusb.in_waiting  # bytes dispo [web:92]
        if n:
            buf += serusb.read(n)  # ne bloque pas si timeout=0 [web:92]
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                s = line.decode(errors="ignore").strip()
                if not s:
                    continue
                try:
                    raw = int(s)
                    v = raw / scale
                    if v < v_min: v = v_min
                    if v > v_max: v = v_max
                    last_vset = int(v) & 0xFF
                except ValueError:
                    pass
        return last_vset

    return poll_vset_u8

# ========== Main motor loop ==========
def MOTOR(ser_stm32, poll_vset_u8,
          a_default=0.0, a_limit=None,
          Td=0.250, tau=0.100, Ts=0.010, steps=2000, k_drag=0.0, v_init=0.0):

    delay_steps = max(1, int(round(Td / Ts)))
    delay_line = deque([float(a_default)] * delay_steps, maxlen=delay_steps)

    v = float(v_init)
    a_eff = 0.0
    t_next = time.monotonic()

    vset_u8 = poll_vset_u8()  # valeur initiale

    for _ in range(steps):
        # Tick ~fixe Ts
        now = time.monotonic()
        if now < t_next:
            time.sleep(t_next - now)
        t_next += Ts

        # Update consigne depuis ESP32 (non bloquant)
        vset_u8 = poll_vset_u8()

        # 1) Lire accel (STM32 -> Pi) : output du speedlimiter
        a_cmd = read_accel_int16(ser_stm32)
        if a_cmd is None:
            a_cmd = delay_line[-1]
        a_cmd = float(a_cmd)

        if a_limit is not None:
            if a_cmd > a_limit: a_cmd = a_limit
            if a_cmd < -a_limit: a_cmd = -a_limit

        # 2) Retard
        delay_line.append(a_cmd)
        a_delayed = delay_line[0]

        # 3) Inertie 1er ordre
        alpha = Ts / tau
        if alpha > 1.0:
            alpha = 1.0
        a_eff = a_eff + alpha * (a_delayed - a_eff)

        # 4) Intégration vitesse
        v = v + (a_eff - k_drag * v) * Ts
        if v < 0.0:
            v = 0.0

        # VV = vitesse mesurée envoyée au STM32 en uint8
        vv_u8 = int(v)
        if vv_u8 > 255:
            vv_u8 = 255

        # 5) Envoyer (Pi -> STM32): [VSET, VV]
        send_vset_vv_u8(ser_stm32, vv_u8, vset_u8)
        print(vset_u8)

    return v

def main():
    with serial.Serial(PORT_STM32, BAUD, timeout=0.02) as ser_stm32, \
         serial.Serial(PORT_ESP32, BAUD, timeout=0) as serusb:  # timeout=0 => non-bloquant [web:92]

        poll_vset_u8 = make_pot_reader_u8(serusb, scale=19.0, v_min=0.0, v_max=255.0)
        MOTOR(ser_stm32, poll_vset_u8, a_default=0.0, a_limit=2000.0)

if __name__ == "__main__":
    main()
