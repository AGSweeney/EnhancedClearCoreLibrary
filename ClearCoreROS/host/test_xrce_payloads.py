"""XRCE JointState body checks without a board or UDP socket."""

from __future__ import annotations

import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "host"))

from xrce_check import joint_from_write, writer_id  # noqa: E402


class CdrOut:
    def __init__(self) -> None:
        self.data = bytearray()

    def align(self, n: int) -> None:
        while len(self.data) % n:
            self.data.append(0)

    def u32(self, v: int) -> None:
        self.align(4)
        self.data.extend(struct.pack("<I", v))

    def f64(self, v: float) -> None:
        self.align(8)
        self.data.extend(struct.pack("<d", v))

    def string(self, s: str) -> None:
        raw = s.encode() + b"\x00"
        self.u32(len(raw))
        self.data.extend(raw)


def _write_payload(writer: int, body: bytes) -> bytes:
    obj0 = (writer >> 4) & 0xFF
    obj1 = ((writer << 4) | 0x05) & 0xFF
    return bytes([0, 1, obj0, obj1]) + body


def _names(cdr: CdrOut, names: tuple[str, ...]) -> None:
    cdr.u32(len(names))
    for name in names:
        cdr.string(name)


def joint_states_body(*, effort: bool) -> bytes:
    cdr = CdrOut()
    cdr.u32(1700000000)
    cdr.u32(1)
    cdr.string("")
    names = ("joint_x", "joint_y", "joint_z", "joint_a")
    _names(cdr, names)
    cdr.u32(4)
    for _ in names:
        cdr.f64(0.0)
    cdr.u32(4)
    for _ in names:
        cdr.f64(0.0)
    if effort:
        cdr.u32(4)
        cdr.f64(0.1)
        cdr.f64(0.0)
        cdr.f64(0.0)
        cdr.f64(0.0)
    else:
        cdr.u32(0)
    return bytes(cdr.data)


def hlfb_body() -> bytes:
    cdr = CdrOut()
    cdr.u32(1700000000)
    cdr.u32(1)
    cdr.string("")
    names = ("joint_x", "joint_y", "joint_z", "joint_a")
    _names(cdr, names)
    cdr.u32(0)
    cdr.u32(0)
    cdr.u32(4)
    cdr.f64(0.1)
    cdr.f64(0.0)
    cdr.f64(0.0)
    cdr.f64(0.0)
    return bytes(cdr.data)


def test_empty_effort_joint_states_accepted():
    payload = _write_payload(1, joint_states_body(effort=False))
    assert writer_id(payload) == 1
    text = joint_from_write(payload)
    assert text is not None and text.startswith("JOINT")
    assert "effort_len=0" in text


def test_filled_effort_joint_states_rejected():
    """0.1.2 published four effort values on rt/joint_states."""
    payload = _write_payload(1, joint_states_body(effort=True))
    assert joint_from_write(payload) is None


def test_hlfb_duty_writer_accepted():
    payload = _write_payload(2, hlfb_body())
    assert writer_id(payload) == 2
    text = joint_from_write(payload)
    assert text is not None and text.startswith("HLFB")
    assert "joint_x=0.1000" in text


if __name__ == "__main__":
    test_empty_effort_joint_states_accepted()
    test_filled_effort_joint_states_rejected()
    test_hlfb_duty_writer_accepted()
    print("xrce_payloads ok")
