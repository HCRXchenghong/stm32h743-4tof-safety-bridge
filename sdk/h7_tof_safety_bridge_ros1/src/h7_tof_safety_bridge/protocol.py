from dataclasses import dataclass
from typing import Dict, List


INVALID_DISTANCE_MM = 65535
DEFAULT_BASELINE_MM = 391
DEFAULT_TOLERANCE_MM = 28
DEFAULT_THRESHOLD_MM = min(INVALID_DISTANCE_MM - 1, DEFAULT_BASELINE_MM + DEFAULT_TOLERANCE_MM)
TOF_CHANNEL_LABELS = (
    "right_front",
    "right_rear",
    "left_rear",
    "left_front",
)


@dataclass
class ControlCommand:
    seq: int
    release_req: bool
    baseline_mm: int
    tolerance_mm: int
    release_hold_ms: int


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
    baseline_mm: int
    tolerance_mm: int
    threshold_mm: int
    release_remaining_ms: int
    has_board_params: bool


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
    baseline_mm = max(0, int(command.baseline_mm))
    tolerance_mm = max(0, int(command.tolerance_mm))
    release_hold_ms = max(0, int(command.release_hold_ms))
    payload = (
        f"H7CTL,{int(command.seq)},{1 if command.release_req else 0},"
        f"{baseline_mm},{tolerance_mm},{release_hold_ms}"
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
    valid_mask = parse_int(parts[7])
    fault_mask = parse_int(parts[8])
    fields = _parse_key_value_fields(parts[9:])
    self_estop = bool(_parse_field_int(fields, "SE", int(estop)))
    external_estop = bool(_parse_field_int(fields, "EE", 0))
    active_mask = _parse_field_int(fields, "A", 0x0F)
    trip_mask = _parse_field_int(fields, "TM", fault_mask)
    has_board_params = "B" in fields and "T" in fields
    baseline_mm = _parse_field_int(fields, "B", DEFAULT_BASELINE_MM)
    tolerance_mm = _parse_field_int(fields, "T", DEFAULT_TOLERANCE_MM)
    threshold_mm = _parse_field_int(
        fields,
        "TH",
        min(INVALID_DISTANCE_MM - 1, baseline_mm + tolerance_mm),
    )
    release_remaining_ms = _parse_field_int(fields, "RT", 0)

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
        baseline_mm=baseline_mm,
        tolerance_mm=tolerance_mm,
        threshold_mm=threshold_mm,
        release_remaining_ms=release_remaining_ms,
        has_board_params=has_board_params,
    )


def active_missing_mask(active_mask: int, valid_mask: int) -> int:
    return int(active_mask) & (~int(valid_mask) & 0x0F)


def mask_to_tof_labels(mask: int) -> List[str]:
    normalized_mask = int(mask) & 0x0F
    return [TOF_CHANNEL_LABELS[index] for index in range(len(TOF_CHANNEL_LABELS)) if (normalized_mask & (1 << index)) != 0]


def _parse_key_value_fields(parts: List[str]) -> Dict[str, str]:
    fields: Dict[str, str] = {}
    for item in parts:
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        fields[key.strip().upper()] = value.strip()
    return fields


def _parse_field_int(fields: Dict[str, str], key: str, default: int) -> int:
    raw_value = fields.get(key)
    if raw_value is None:
        return default
    return parse_int(raw_value)
