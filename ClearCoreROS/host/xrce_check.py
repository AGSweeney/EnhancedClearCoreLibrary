"""Minimal XRCE-DDS agent checker.

Accepts the ClearCore client, answers CREATE_CLIENT and CREATE, and prints
sensor_msgs/JointState samples from rt/joint_states. This is not a DDS bridge.
A micro-ROS agent is the process that would publish those samples onto a ROS graph.
"""

from __future__ import annotations

import socket
import struct
import sys
import time


def pad4(n: int) -> int:
    return (n + 3) & ~3


def reply_status_agent(session: int) -> bytes:
    # session >= 0x80, so the header has no client key.
    header = bytes([session, 0, 0, 0])
    # STATUS_AGENT: status 0, implementation 0, cookie XRCE, version 1.0, vendor, no properties.
    payload = bytes([0, 0, 0x58, 0x52, 0x43, 0x45, 1, 0, 0x01, 0x0F, 0])
    sub = bytes([4, 1]) + struct.pack("<H", len(payload)) + payload
    return header + sub + bytes(pad4(len(header) + len(sub)) - (len(header) + len(sub)))


def reply_ack(session: int, seq: int) -> bytes:
    header = bytes([session, 0x80, seq & 0xFF, (seq >> 8) & 0xFF])
    # ACKNACK: first unacked = seq+1, bitmap 0, stream 0x80.
    ack_payload = struct.pack("<H", (seq + 1) & 0xFFFF) + bytes([0, 0, 0x80])
    ack = bytes([10, 1]) + struct.pack("<H", len(ack_payload)) + ack_payload
    return header + ack


def reply_status(session: int, seq: int, request: bytes, obj: bytes) -> bytes:
    header = bytes([session, 0x80, seq & 0xFF, (seq >> 8) & 0xFF])
    payload = request + obj + bytes([0, 0])
    sub = bytes([5, 1]) + struct.pack("<H", len(payload)) + payload
    ack = reply_ack(session, seq)[4:]
    body = sub + bytes((4 - (len(sub) % 4)) % 4) + ack
    return header + body


class Cdr:
    def __init__(self, data: bytes, i: int, origin: int | None = None) -> None:
        self.data = data
        self.i = i
        self.origin = i if origin is None else origin

    def align(self, n: int) -> None:
        while (self.i - self.origin) % n:
            self.i += 1

    def u32(self) -> int:
        self.align(4)
        v = struct.unpack_from("<I", self.data, self.i)[0]
        self.i += 4
        return v

    def f64(self) -> float:
        self.align(8)
        v = struct.unpack_from("<d", self.data, self.i)[0]
        self.i += 8
        return v

    def string(self) -> str:
        n = self.u32()
        raw = self.data[self.i : self.i + n]
        self.i += n
        return raw.split(b"\x00", 1)[0].decode()


def reply_timestamp(session: int, seq: int, t1_sec: int, t1_nsec: int) -> bytes:
    """TIMESTAMP_REPLY: echoed client Time_t, then agent receive and transmit Time_t."""
    header = bytes([session, 0x80, seq & 0xFF, (seq >> 8) & 0xFF])
    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1_000_000_000)
    payload = struct.pack("<iIiIiI", t1_sec, t1_nsec, sec, nsec, sec, nsec)
    sub = bytes([15, 1]) + struct.pack("<H", len(payload)) + payload
    ack = reply_ack(session, seq)[4:]
    body = sub + bytes((4 - (len(sub) % 4)) % 4) + ack
    return header + body


def joint_from_write(payload: bytes) -> str | None:
    """Decode WRITE_DATA. The first four bytes are the XRCE request and object id.

    The JointState body starts at byte 4. Fast DDS adds the CDR encapsulation,
    so this body has none. Alignment is from that first body byte.
    """
    if len(payload) < 8:
        return None
    cdr = Cdr(payload, 4)
    try:
        sec = cdr.u32()
        nsec = cdr.u32()
        frame = cdr.string()
        count = cdr.u32()
        names = [cdr.string() for _ in range(count)]
        npos = cdr.u32()
        pos = [cdr.f64() for _ in range(npos)]
    except (struct.error, IndexError, UnicodeDecodeError):
        return None
    if frame == "unsync" or sec == 0:
        return None
    joints = " ".join(f"{name}={value:.5f}" for name, value in zip(names, pos))
    return f"sec={sec} nsec={nsec} frame={frame!r} {joints}"


def main() -> None:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 9204
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)
    print(f"listening {port}", flush=True)
    deadline = time.time() + 20
    samples = 0
    addr = None
    sync_reply: bytes | None = None
    sync_addr = None
    sync_due: float | None = None
    sync_sent = False
    while time.time() < deadline and samples < 3:
        if sync_reply is not None and sync_due is not None and time.time() >= sync_due:
            sock.sendto(sync_reply, sync_addr)
            print("TIMESTAMP_REPLY", flush=True)
            sync_reply = None
            sync_sent = True
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        if len(data) < 8:
            continue
        session = data[0]
        stream = data[1]
        seq = data[2] | (data[3] << 8)
        i = 4 if session >= 0x80 else 8
        # One reply datagram per received message. CREATE and its heartbeat share
        # a packet, and this reply already carries STATUS and the ACKNACK.
        reply = None
        while i + 4 <= len(data):
            while i % 4 and i < len(data):
                i += 1
            if i + 4 > len(data):
                break
            mid = data[i]
            length = data[i + 2] | (data[i + 3] << 8)
            payload = data[i + 4 : i + 4 + length]
            i += 4 + length
            if mid == 0:
                print("CREATE_CLIENT", payload[:4], flush=True)
                reply = reply_status_agent(0x81)
            elif mid == 1 and len(payload) >= 4:
                kind = payload[4] if len(payload) > 4 else 0
                print(f"CREATE kind={kind} seq={seq}", flush=True)
                reply = reply_status(0x81, seq, payload[0:2], payload[2:4])
            elif mid == 14 and len(payload) >= 8:
                t1_sec, t1_nsec = struct.unpack_from("<iI", payload, 0)
                print("TIMESTAMP", t1_sec, t1_nsec, flush=True)
                if not sync_sent and sync_reply is None:
                    # Hold the reply so a premature WRITE_DATA is visible.
                    sync_due = time.time() + 0.4
                    sync_reply = reply_timestamp(0x81, seq, t1_sec, t1_nsec)
                    sync_addr = addr
                    reply = reply_ack(0x81, seq)
                elif reply is None:
                    reply = reply_ack(0x81, seq)
            elif mid == 11 and reply is None:
                reply = reply_ack(0x81, seq)
            elif mid == 7:
                if not sync_sent:
                    raise SystemExit("JointState before TIMESTAMP_REPLY")
                text = joint_from_write(payload)
                if text is None:
                    raise SystemExit("unsynced or invalid JointState")
                samples += 1
                print("JOINT", text, flush=True)
        if reply is not None:
            sock.sendto(reply, addr)
    if samples < 1:
        raise SystemExit("no JointState sample")
    print(f"XRCE_OK samples={samples}", flush=True)


if __name__ == "__main__":
    main()
