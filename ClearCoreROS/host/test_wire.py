"""Compare the Python frame codec with firmware/RosProtocol.h."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.wire import (  # noqa: E402
    HDR_SIZE,
    POSITION_PAYLOAD,
    STATE_PAYLOAD,
    feed,
    pack_heartbeat,
    pack_position,
)


def test_sizes_and_roundtrip() -> None:
    frame = pack_position(7, 0x03, (0.01, -0.02, 0.0, 1.0))
    assert len(frame) == HDR_SIZE + POSITION_PAYLOAD
    buf = bytearray(b"\x00" + frame)
    parsed = feed(buf)
    assert buf == bytearray()
    assert len(parsed) == 1
    assert parsed[0]["type"] == "position"
    assert parsed[0]["seq"] == 7
    assert parsed[0]["mask"] == 0x03
    assert abs(parsed[0]["values"][0] - 0.01) < 1e-7
    assert abs(parsed[0]["values"][1] + 0.02) < 1e-7
    beat = pack_heartbeat(9)
    buf = bytearray(beat)
    got = feed(buf)
    assert got[0]["type"] == "heartbeat" and got[0]["seq"] == 9


def _host_cxx() -> str | None:
    from shutil import which

    found = which("g++") or which("clang++")
    if found:
        return found
    atmel = Path(r"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-g++.exe")
    return str(atmel) if atmel.is_file() else None


def test_matches_firmware_header() -> None:
    src = ROOT / "host" / "test_protocol.cpp"
    cxx = _host_cxx()
    if cxx is None:
        raise RuntimeError("no C++ compiler for firmware/RosProtocol.h")
    cross = "arm-none-eabi" in Path(cxx).name
    if cross:
        obj = ROOT / "host" / "test_protocol.o"
        subprocess.check_call([cxx, "-std=c++17", "-Wall", "-c", "-o", str(obj), str(src)], cwd=ROOT)
        obj.unlink(missing_ok=True)
        print("compiled RosProtocol.h with the firmware toolchain (no host g++ to execute it)")
        return
    exe = ROOT / "host" / "test_protocol.exe"
    subprocess.check_call([cxx, "-std=c++17", "-Wall", "-o", str(exe), str(src)], cwd=ROOT)
    out = subprocess.check_output([str(exe)], text=True).strip().splitlines()
    assert len(out) == 3 and out[2] == "ok"
    py_pos = pack_position(7, 0x03, (0.01, -0.02, 0.0, 1.0)).hex()
    assert out[0] == py_pos
    raw = bytes.fromhex(out[1])
    assert len(raw) == HDR_SIZE + STATE_PAYLOAD
    parsed = feed(bytearray(raw))
    assert parsed[0]["type"] == "state"
    assert parsed[0]["time_ms"] == 1000
    assert parsed[0]["seq"] == 3
    assert parsed[0]["flags"] == 0x03
    assert parsed[0]["enabled"] and parsed[0]["moving"]
    assert abs(parsed[0]["position"][0] - 0.01) < 1e-7
    assert abs(parsed[0]["effort"][0] - 0.5) < 1e-7


if __name__ == "__main__":
    test_sizes_and_roundtrip()
    test_matches_firmware_header()
    print("wire ok")
