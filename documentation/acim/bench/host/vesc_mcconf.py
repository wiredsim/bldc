#!/usr/bin/env python3
"""Read/modify the full VESC motor config (COMM_GET_MCCONF 14 / COMM_SET_MCCONF 13).

The byte layout is parsed from confgenerator_serialize_mcconf() in ~/bldc/confgenerator.c,
so it matches the flashed firmware. Only the named fields are changed; every other byte is
sent back exactly as read.

Usage: vesc_mcconf.py field [field ...]            # print fields
       vesc_mcconf.py --set foc_dt_us=0.47 ...     # write, then read back and verify
"""
import re, socket, struct, sys, time, importlib.util, os
spec = importlib.util.spec_from_file_location("v", os.path.join(os.path.dirname(os.path.abspath(__file__)), "vesc_term.py"))
v = importlib.util.module_from_spec(spec); spec.loader.exec_module(v)
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "confgenerator.c")

def layout():
    src = open(SRC).read()
    body = src[src.index("int32_t confgenerator_serialize_mcconf"):]
    body = body[:body.index("\n}\n")]
    fields, off = {}, 0
    for line in body.splitlines():
        line = line.strip()
        m = re.match(r"buffer_append_(\w+)\(buffer, (?:\(\w+\))?conf->([\w\[\].]+)(?:, ([\d.e+-]+))?, &ind\);", line)
        if m:
            t, name, scale = m.group(1), m.group(2), m.group(3)
            size = {"float32_auto": 4, "float32": 4, "uint32": 4, "int32": 4, "float16": 2, "uint16": 2, "int16": 2}[t]
            fields[name] = (off, t, float(scale) if scale else None); off += size; continue
        m = re.match(r"buffer\[ind\+\+\] = (?:\(\w+\))?conf->([\w\[\].]+);", line)
        if m:
            fields[m.group(1)] = (off, "u8", None); off += 1; continue
        if line.startswith("buffer_append_uint32(buffer, MCCONF_SIGNATURE"):
            off += 4; continue
        if line.startswith(("buffer", "conf->")):
            raise SystemExit(f"unparsed serializer line: {line}")
    return fields, off

def get(fields, raw, name):
    off, t, sc = fields[name]
    b = raw[off:off + {"u8": 1, "float16": 2, "uint16": 2, "int16": 2}.get(t, 4)]
    if t in ("float32_auto", "float32"): return struct.unpack(">f", b)[0]
    if t == "float16": return struct.unpack(">h", b)[0] / sc
    if t == "u8": return b[0]
    return struct.unpack({"uint16": ">H", "int16": ">h", "uint32": ">I", "int32": ">i"}[t], b)[0]

def put(fields, raw, name, val):
    off, t, sc = fields[name]
    if t in ("float32_auto", "float32"): raw[off:off + 4] = struct.pack(">f", float(val))
    elif t == "float16": raw[off:off + 2] = struct.pack(">h", round(float(val) * sc))
    elif t == "u8": raw[off] = int(float(val))
    elif t in ("uint32", "int32", "uint16", "int16"):
        fmt = {"uint16": ">H", "int16": ">h", "uint32": ">I", "int32": ">i"}[t]
        raw[off:off + struct.calcsize(fmt)] = struct.pack(fmt, int(float(val)))
    else: raise SystemExit(f"type {t} not supported for writing")

def request(s, payload, want, timeout=4.0):
    for _ in range(4):
        s.sendall(v.pack(payload)); buf = bytearray(); end = time.time() + timeout
        while time.time() < end:
            s.settimeout(0.2)
            try: c = s.recv(65536)
            except socket.timeout: continue
            buf += c
            for p in v.unpack_all(buf):
                if p[0] == want: return p
    raise SystemExit(f"no reply to packet {payload[0]}")

fields, total = layout()
s = socket.create_connection(("127.0.0.1", 65102), timeout=3)
raw = bytearray(request(s, bytes([14]), 14)[1:])
if len(raw) != total:
    raise SystemExit(f"length mismatch: board sent {len(raw)} bytes, layout says {total}. Not touching anything.")
args = sys.argv[1:]
if args and args[0] == "--set":
    changes = dict(a.split("=") for a in args[1:])
    for k, val in changes.items():
        print(f"{k}: {get(fields, raw, k)} -> {val}")
        put(fields, raw, k, val)
    request(s, bytes([13]) + bytes(raw), 13, timeout=8.0)
    time.sleep(1.0)
    back = bytearray(request(s, bytes([14]), 14)[1:])
    bad = [k for k, val in changes.items() if abs(get(fields, back, k) - float(val)) > 1e-3 * max(1, abs(float(val)))]
    other = [k for k in fields if k not in changes and get(fields, back, k) != get(fields, raw, k)]
    print("VERIFY FAILED: " + ", ".join(bad) if bad else "written and verified")
    if other: print("note, other fields differ after write:", other)
    args = list(changes)
for k in args:
    print(f"  {k} = {get(fields, raw if not sys.argv[1:2] == ['--set'] else back, k)}")
s.close()
