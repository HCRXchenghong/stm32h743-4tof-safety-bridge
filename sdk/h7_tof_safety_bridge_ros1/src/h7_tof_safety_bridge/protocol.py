from dataclasses import dataclass
from typing import List


INVALID_DISTANCE_MM = 65535
MOTION_LABELS = {
    0: "idle",
    1: "forward",
    2: "reverse",
    3: "turning",
    4: "planar",
    5: "failsafe",
}


@dataclass
class ControlCommand:
    seq: int
    vx_mmps: int
    vy_mmps: int
    wz_mradps: int
    release_req: bool
    takeover_enable: bool


@dataclass
class TofStatusFrame:
    seq: int
    tof_mm: List[int]
    estop: bool
    self_estop: bool
    external_estop: bool
    active_mask: int
    trip_mask: int
    valid_mask: int
    fault_mask: int
    motion_mode: int
    takeover_enabled: bool


def xor_checksum(payload: str) -> int:
    checksum = 0
    for ch in payload:
        checksum ^= ord(ch)
    return checksum


def parse_int(raw: str) -> int:
    text = raw.strip()
    if not text:
        raise ValueError("empty integer field")
    if text.lower().startswith("0x") or any(ch in "abcdefABCDEF" for ch in text):
        return int(text, 16)
    return int(text, 10)


def build_control_frame(command: ControlCommand) -> str:
    payload = (
        f"H7CTL,{command.seq},{command.vx_mmps},{command.vy_mmps},"
        f"{command.wz_mradps},{1 if command.release_req else 0},"
        f"{1 if command.takeover_enable else 0}"
    )
    checksum = xor_checksum(payload)
    return f"${payload}*{checksum:02X}\r\n"


def parse_status_line(line: str) -> TofStatusFrame:
    text = line.strip()
    if not text:
        raise ValueError("empty line")

    clean = text[1:] if text.startswith("$") else text
    if "*" in clean:
        payload, checksum_text = clean.split("*", 1)
        expected = int(checksum_text.strip(), 16)
        actual = xor_checksum(payload)
        if expected != actual:
            raise ValueError(f"checksum mismatch: {actual:02X} != {checksum_text.strip().upper()}")
    else:
        payload = clean

    parts = [item.strip() for item in payload.split(",")]
    if len(parts) < 9:
        raise ValueError("not enough fields")
    if parts[0] not in ("H7TOF", "TOF"):
        raise ValueError("unexpected frame head")

    seq = parse_int(parts[1])
    tof_mm = [
        parse_int(parts[2]),
        parse_int(parts[3]),
        parse_int(parts[4]),
        parse_int(parts[5]),
    ]
    estop = bool(parse_int(parts[6]))

    if len(parts) >= 15:
        self_estop = bool(parse_int(parts[7]))
        external_estop = bool(parse_int(parts[8]))
        active_mask = parse_int(parts[9])
        trip_mask = parse_int(parts[10])
        valid_mask = parse_int(parts[11])
        fault_mask = parse_int(parts[12])
        motion_mode = parse_int(parts[13])
        takeover_enabled = bool(parse_int(parts[14]))
    else:
        self_estop = estop
        external_estop = False
        active_mask = 0x0F
        trip_mask = 0x00
        valid_mask = parse_int(parts[7])
        fault_mask = parse_int(parts[8])
        motion_mode = 5
        takeover_enabled = False

    return TofStatusFrame(
        seq=seq,
        tof_mm=tof_mm,
        estop=estop,
        self_estop=self_estop,
        external_estop=external_estop,
        active_mask=active_mask,
        trip_mask=trip_mask,
        valid_mask=valid_mask,
        fault_mask=fault_mask,
        motion_mode=motion_mode,
        takeover_enabled=takeover_enabled,
    )


def motion_label(motion_mode: int) -> str:
    return MOTION_LABELS.get(motion_mode, f"unknown({motion_mode})")
