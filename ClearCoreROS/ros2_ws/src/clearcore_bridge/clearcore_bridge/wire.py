"""Little-endian ClearCoreROS stream frames. No ROS dependency.

Byte layout is firmware/RosProtocol.h and PROTOCOL.md.
"""

from __future__ import annotations

import json
import socket
import struct
import threading
from typing import Iterable

MAGIC = 0xC5
VERSION = 1
HDR_SIZE = 6

TYPE_STATE = 1
TYPE_POSITION = 2
TYPE_VELOCITY = 3
TYPE_HEARTBEAT = 4
TYPE_TRACK = 5

FLAG_ENABLED = 0x01
FLAG_MOVING = 0x02
FLAG_ESTOP = 0x04
FLAG_FAULT = 0x08
FLAG_WATCHDOG = 0x10

STATE_PAYLOAD = 128
POSITION_PAYLOAD = 20
HEARTBEAT_PAYLOAD = 2
TRACK_PAYLOAD = 36

JOINTS = ["joint_x", "joint_y", "joint_z", "joint_a"]
AXIS = {"x": 0, "y": 1, "z": 2, "a": 3, "joint_x": 0, "joint_y": 1, "joint_z": 2, "joint_a": 3}
ROTARY = [False, False, False, True]


def apply_joint_map(names, rotary=None) -> None:
    """Replace the live joint names in place. Callers keep the same list and dict."""
    if not isinstance(names, list) or len(names) != 4:
        return
    if any(not isinstance(name, str) or not name for name in names):
        return
    if len(set(names)) != 4:
        return
    for old in list(JOINTS):
        AXIS.pop(old, None)
    for index, name in enumerate(names):
        JOINTS[index] = name
        AXIS[name] = index
        if isinstance(rotary, list) and index < len(rotary):
            ROTARY[index] = bool(rotary[index])


def _hdr(msg_type: int, payload_len: int) -> bytes:
    return struct.pack("<BBBBH", MAGIC, VERSION, msg_type, 0, payload_len)


def pack_position(seq: int, mask: int, q: Iterable[float]) -> bytes:
    vals = tuple(q)
    if len(vals) != 4:
        raise ValueError("position needs 4 joints")
    payload = struct.pack("<HBB4f", seq & 0xFFFF, mask & 0xFF, 0, *vals)
    if len(payload) != POSITION_PAYLOAD:
        raise RuntimeError("position payload size")
    return _hdr(TYPE_POSITION, len(payload)) + payload


def pack_velocity(seq: int, mask: int, v: Iterable[float]) -> bytes:
    vals = tuple(v)
    if len(vals) != 4:
        raise ValueError("velocity needs 4 joints")
    payload = struct.pack("<HBB4f", seq & 0xFFFF, mask & 0xFF, 0, *vals)
    return _hdr(TYPE_VELOCITY, len(payload)) + payload


def pack_track(seq: int, mask: int, position: Iterable[float], velocity: Iterable[float]) -> bytes:
    pos = tuple(position)
    vel = tuple(velocity)
    if len(pos) != 4 or len(vel) != 4:
        raise ValueError("track needs 4 positions and 4 velocities")
    payload = struct.pack("<HBB4f4f", seq & 0xFFFF, mask & 0xFF, 0, *pos, *vel)
    if len(payload) != TRACK_PAYLOAD:
        raise RuntimeError("track payload size")
    return _hdr(TYPE_TRACK, len(payload)) + payload


def pack_heartbeat(seq: int) -> bytes:
    payload = struct.pack("<H", seq & 0xFFFF)
    return _hdr(TYPE_HEARTBEAT, len(payload)) + payload


def pack_state(
    time_ms: int = 0,
    seq: int = 1,
    flags: int = 0,
    mask: int = 3,
    alert: int = 0,
    position: Iterable[float] = (0.0, 0.0, 0.0, 0.0),
    velocity: Iterable[float] = (0.0, 0.0, 0.0, 0.0),
    effort: Iterable[float] = (0.0, 0.0, 0.0, 0.0),
    track_mask: int = 0,
    target_position: Iterable[float] = (0.0, 0.0, 0.0, 0.0),
    target_velocity: Iterable[float] = (0.0, 0.0, 0.0, 0.0),
    command_velocity: Iterable[float] = (0.0, 0.0, 0.0, 0.0),
    target_latch_ms: Iterable[int] = (0, 0, 0, 0),
) -> bytes:
    """Build a state frame. Layout matches CcrosEncodeState."""
    payload = bytearray(STATE_PAYLOAD)
    struct.pack_into("<IHBBI", payload, 0, time_ms & 0xFFFFFFFF, seq & 0xFFFF, flags & 0xFF, mask & 0xFF, alert & 0xFFFFFFFF)
    struct.pack_into("<4f", payload, 12, *tuple(position))
    struct.pack_into("<4f", payload, 28, *tuple(velocity))
    struct.pack_into("<4f", payload, 44, *tuple(effort))
    payload[60] = track_mask & 0xFF
    struct.pack_into("<4f", payload, 64, *tuple(target_position))
    struct.pack_into("<4f", payload, 80, *tuple(target_velocity))
    struct.pack_into("<4f", payload, 96, *tuple(command_velocity))
    struct.pack_into("<4I", payload, 112, *tuple(int(v) & 0xFFFFFFFF for v in target_latch_ms))
    return _hdr(TYPE_STATE, STATE_PAYLOAD) + bytes(payload)


def _parse_payload(msg_type: int, payload: bytes) -> dict:
    if msg_type == TYPE_STATE:
        if len(payload) != STATE_PAYLOAD:
            raise ValueError("bad state payload")
        time_ms, seq, flags, mask, alert = struct.unpack_from("<IHBBI", payload, 0)
        pos = struct.unpack_from("<4f", payload, 12)
        vel = struct.unpack_from("<4f", payload, 28)
        eff = struct.unpack_from("<4f", payload, 44)
        track_mask = payload[60]
        target_pos = struct.unpack_from("<4f", payload, 64)
        target_vel = struct.unpack_from("<4f", payload, 80)
        command_vel = struct.unpack_from("<4f", payload, 96)
        target_latch_ms = struct.unpack_from("<4I", payload, 112)
        return {
            "type": "state",
            "time_ms": time_ms,
            "seq": seq,
            "flags": flags,
            "axis_mask": mask,
            "alert_reg": alert,
            "position": pos,
            "velocity": vel,
            "effort": eff,
            "track_mask": track_mask,
            "target_position": target_pos,
            "target_velocity": target_vel,
            "command_velocity": command_vel,
            "target_latch_ms": target_latch_ms,
            "enabled": bool(flags & FLAG_ENABLED),
            "moving": bool(flags & FLAG_MOVING),
            "estop": bool(flags & FLAG_ESTOP),
            "fault": bool(flags & FLAG_FAULT),
            "watchdog": bool(flags & FLAG_WATCHDOG),
        }
    if msg_type in (TYPE_POSITION, TYPE_VELOCITY):
        if len(payload) != POSITION_PAYLOAD:
            raise ValueError("bad command payload")
        seq, mask, _reserved = struct.unpack_from("<HBB", payload, 0)
        vals = struct.unpack_from("<4f", payload, 4)
        kind = "position" if msg_type == TYPE_POSITION else "velocity"
        return {"type": kind, "seq": seq, "mask": mask, "values": vals}
    if msg_type == TYPE_TRACK:
        if len(payload) != TRACK_PAYLOAD:
            raise ValueError("bad track payload")
        seq, mask, _reserved = struct.unpack_from("<HBB", payload, 0)
        pos = struct.unpack_from("<4f", payload, 4)
        vel = struct.unpack_from("<4f", payload, 20)
        return {"type": "track", "seq": seq, "mask": mask, "position": pos, "velocity": vel}
    if msg_type == TYPE_HEARTBEAT:
        if len(payload) != HEARTBEAT_PAYLOAD:
            raise ValueError("bad heartbeat payload")
        (seq,) = struct.unpack("<H", payload)
        return {"type": "heartbeat", "seq": seq}
    return {"type": "unknown", "code": msg_type, "payload": payload}


def feed(buf: bytearray) -> list[dict]:
    """Pull every complete frame out of buf. Incomplete bytes stay in buf."""
    frames = []
    while buf:
        if buf[0] != MAGIC:
            del buf[0]
            continue
        if len(buf) < HDR_SIZE:
            break
        _magic, version, msg_type, _reserved, plen = struct.unpack_from("<BBBBH", buf, 0)
        if version != VERSION or plen > STATE_PAYLOAD:
            del buf[0]
            continue
        if msg_type == TYPE_STATE and plen != STATE_PAYLOAD:
            del buf[0]
            continue
        if msg_type in (TYPE_POSITION, TYPE_VELOCITY) and plen != POSITION_PAYLOAD:
            del buf[0]
            continue
        if msg_type == TYPE_HEARTBEAT and plen != HEARTBEAT_PAYLOAD:
            del buf[0]
            continue
        if msg_type == TYPE_TRACK and plen != TRACK_PAYLOAD:
            del buf[0]
            continue
        total = HDR_SIZE + plen
        if len(buf) < total:
            break
        payload = bytes(buf[HDR_SIZE:total])
        del buf[:total]
        frames.append(_parse_payload(msg_type, payload))
    return frames


class SessionClient:
    def __init__(self, host: str, port: int = 9200, timeout: float = 2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = socket.create_connection((host, port), timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._buf = b""
        self._next_id = 1

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def call(self, method: str, params: dict | None = None, timeout: float | None = None) -> dict:
        msg = {"jsonrpc": "2.0", "id": self._next_id, "method": method}
        self._next_id += 1
        if params is not None:
            msg["params"] = params
        self.sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))
        line = self._readline(self.timeout if timeout is None else timeout)
        reply = json.loads(line)
        if "error" in reply:
            err = reply["error"]
            message = err.get("message", err) if isinstance(err, dict) else err
            raise RuntimeError(str(message))
        return reply.get("result", reply)

    def _readline(self, timeout: float) -> str:
        self.sock.settimeout(timeout)
        while b"\n" not in self._buf:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise ConnectionError("session closed")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("utf-8").strip()


class StreamClient:
    def __init__(self, host: str, port: int = 9201, timeout: float = 2.0):
        self.sock = socket.create_connection((host, port), timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._buf = bytearray()
        self._seq = 1
        self._send_lock = threading.Lock()

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def send_position(self, mask: int, q: Iterable[float]) -> None:
        self._send(pack_position(self._seq, mask, q))

    def send_velocity(self, mask: int, v: Iterable[float]) -> None:
        self._send(pack_velocity(self._seq, mask, v))

    def send_track(self, mask: int, position: Iterable[float], velocity: Iterable[float]) -> None:
        self._send(pack_track(self._seq, mask, position, velocity))

    def send_heartbeat(self) -> None:
        self._send(pack_heartbeat(self._seq))

    def _send(self, frame: bytes) -> None:
        with self._send_lock:
            self.sock.sendall(frame)
            self._seq = (self._seq + 1) & 0xFFFF

    def read(self, timeout: float = 0.0) -> list[dict]:
        self.sock.settimeout(timeout if timeout > 0 else 0.0)
        try:
            chunk = self.sock.recv(512)
        except (BlockingIOError, socket.timeout):
            return feed(self._buf)
        except OSError:
            raise
        if chunk == b"":
            raise ConnectionError("stream closed")
        self._buf.extend(chunk)
        return feed(self._buf)


def discover(host: str, port: int = 9202, timeout: float = 1.0) -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(timeout)
        sock.sendto(b"CLEARCORE_ROS_DISCOVER?", (host, port))
        data, _addr = sock.recvfrom(512)
        return data.decode("utf-8", errors="replace").strip()
    finally:
        sock.close()
