#!/usr/bin/env python3
"""Run the VESC 'Measure R and L' (COMM_DETECT_MOTOR_R_L) via VESC Tool's TCP bridge."""
import socket, struct, time, importlib.util, os
spec = importlib.util.spec_from_file_location("v", os.path.join(os.path.dirname(os.path.abspath(__file__)), "vesc_term.py"))
v = importlib.util.module_from_spec(spec); spec.loader.exec_module(v)
COMM_DETECT_MOTOR_R_L = 25
s = socket.create_connection(("127.0.0.1", 65102), timeout=3)
s.sendall(v.pack(bytes([COMM_DETECT_MOTOR_R_L])))
buf, end = bytearray(), time.time() + 30
while time.time() < end:
    s.settimeout(0.5)
    try: c = s.recv(4096)
    except socket.timeout: continue
    if not c: break
    buf += c
    for p in v.unpack_all(buf):
        if p[0] == COMM_DETECT_MOTOR_R_L:
            print("raw:", p.hex())
            vals = [struct.unpack(">i", p[1 + 4*i:5 + 4*i])[0] for i in range((len(p) - 1) // 4)]
            print("ints:", vals)
            print(f"R = {vals[0]/1e6*1000:.3f} mOhm, L = {vals[1]/1e3:.3f} uH" + (f", Ld-Lq = {vals[2]/1e3:.3f} uH" if len(vals) > 2 else ""))
            raise SystemExit
        elif p[0] == v.COMM_PRINT:
            print("print:", p[1:].decode(errors="replace").strip())
print("(no R/L reply within 30 s)")
