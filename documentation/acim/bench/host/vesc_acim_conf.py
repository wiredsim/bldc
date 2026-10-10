#!/usr/bin/env python3
"""Read or write the ACIM custom config page (COMM_GET/SET_CUSTOM_CONFIG, index 0).

Usage: vesc_acim_conf.py                     # print current values
       vesc_acim_conf.py id_mag=20 enable=1  # change fields, write, read back
Field order matches motor/acim_confgen.c (signature, 2 bytes, 13 float32_auto).
"""
import socket, struct, sys, time, importlib.util, os
spec = importlib.util.spec_from_file_location("v", os.path.join(os.path.dirname(os.path.abspath(__file__)), "vesc_term.py"))
v = importlib.util.module_from_spec(spec); spec.loader.exec_module(v)
GET, SET = 93, 95
BYTES = ["enable", "speed_src"]
FLOATS = ["id_mag", "tau_r", "lm", "lr_lm", "current_max", "slip_max", "flux_build_time",
          "flux_hold_time", "sl_min_hz", "sl_if_ramp", "obs_bw", "fault_flux_err", "fault_slip_fac"]

def request(s, payload, want, timeout=3.0):
    for attempt in range(4):
        s.sendall(v.pack(payload)); buf = bytearray(); end = time.time() + timeout
        while time.time() < end:
            s.settimeout(0.2)
            try: c = s.recv(4096)
            except socket.timeout: continue
            buf += c
            for p in v.unpack_all(buf):
                if p[0] == want: return p
                if p[0] == v.COMM_PRINT: print("print:", p[1:].decode(errors="replace").strip())
    raise SystemExit(f"no reply to packet {payload[0]}")

def decode(body):
    sig = struct.unpack(">I", body[0:4])[0]
    c = {"_sig": sig, "enable": body[4], "speed_src": body[5]}
    for i, n in enumerate(FLOATS):
        c[n] = struct.unpack(">f", body[6 + 4*i:10 + 4*i])[0]   # float32_auto == IEEE754 for normal values
    return c

def encode(c):
    out = struct.pack(">IBB", c["_sig"], int(c["enable"]), int(c["speed_src"]))
    return out + b"".join(struct.pack(">f", c[n]) for n in FLOATS)

s = socket.create_connection(("127.0.0.1", 65102), timeout=3)
cur = decode(request(s, bytes([GET, 0]), GET)[2:])
if len(sys.argv) > 1:
    for a in sys.argv[1:]:
        k, val = a.split("=")
        cur[k] = int(val) if k in BYTES else float(val)
    request(s, bytes([SET, 0]) + encode(cur), SET, timeout=6.0)
    time.sleep(0.5)
    back = decode(request(s, bytes([GET, 0]), GET)[2:])
    bad = [k for k in BYTES + FLOATS if abs(float(back[k]) - float(cur[k])) > 1e-6 * max(1, abs(float(cur[k])))]
    cur = back
    print("VERIFY FAILED: " + ", ".join(bad) if bad else "written and verified")
for k in BYTES + FLOATS:
    print(f"  {k:16} {cur[k]}")
s.close()
