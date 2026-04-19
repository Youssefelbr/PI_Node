import struct, time
import can

CAN_ID_CMD  = 0x200   # STM32 -> Pi : int16 LE  (acceleration command, ±500)
CAN_ID_CTRL = 0x100   # Pi -> STM32 : uint16 LE VSET | uint16 LE VV


def read_accel_int16(bus):
    """
    Wait up to 25 ms for CAN ID 0x200 from STM32.
    Returns the CMD as int16, or None on timeout.
    Ignores frames with other IDs.
    """
    deadline = time.monotonic() + 0.025
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=remaining)
        if msg is None:
            return None
        if msg.arbitration_id == CAN_ID_CMD and len(msg.data) >= 2:
            return struct.unpack("<h", msg.data[:2])[0]


def send_vset_vv_u16(bus, vset_u16, vv_u16):
    """
    Send VSET and VV to STM32 via CAN ID 0x100.
    Byte 0-1 = VSET little-endian, Byte 2-3 = VV little-endian.
    """
    data = struct.pack("<HH", int(vset_u16) & 0xFFFF, int(vv_u16) & 0xFFFF)
    msg = can.Message(arbitration_id=CAN_ID_CTRL, data=data, is_extended_id=False)
    bus.send(msg)
