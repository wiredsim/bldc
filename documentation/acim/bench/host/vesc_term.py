#!/usr/bin/env python3
"""Send a VESC terminal command over VESC Tool's TCP bridge and print the reply.

Usage: vesc_term.py "acim_status" [--host 127.0.0.1] [--port 65102] [--wait 1.5]
"""
import argparse
import socket
import time

COMM_TERMINAL_CMD = 20
COMM_PRINT = 21


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def pack(payload: bytes) -> bytes:
    n = len(payload)
    head = bytes([2, n]) if n < 256 else bytes([3, n >> 8, n & 0xFF])
    c = crc16(payload)
    return head + payload + bytes([c >> 8, c & 0xFF, 3])


def unpack_all(buf: bytearray):
    """Yield complete payloads from buf, consuming them."""
    while buf:
        if buf[0] == 2 and len(buf) >= 2:
            n, hl = buf[1], 2
        elif buf[0] == 3 and len(buf) >= 3:
            n, hl = (buf[1] << 8) | buf[2], 3
        elif buf[0] in (2, 3):
            return
        else:
            del buf[0]
            continue
        total = hl + n + 3
        if len(buf) < total:
            return
        payload = bytes(buf[hl:hl + n])
        crc = (buf[hl + n] << 8) | buf[hl + n + 1]
        ok = crc == crc16(payload) and buf[total - 1] == 3
        if ok:
            del buf[:total]
            yield payload
        else:
            del buf[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=65102)
    ap.add_argument("--wait", type=float, default=1.5)
    a = ap.parse_args()

    s = socket.create_connection((a.host, a.port), timeout=3)
    s.sendall(pack(bytes([COMM_TERMINAL_CMD]) + a.cmd.encode()))
    buf = bytearray()
    end = time.time() + a.wait
    got = False
    while time.time() < end:
        s.settimeout(max(0.05, end - time.time()))
        try:
            chunk = s.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        for p in unpack_all(buf):
            if p and p[0] == COMM_PRINT:
                print(p[1:].decode(errors="replace"))
                got = True
    s.close()
    if not got:
        print("(no terminal output received)")


if __name__ == "__main__":
    main()
