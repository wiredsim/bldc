#!/usr/bin/env python3
"""Stream a LispBM file to the VESC (COMM_LISP_STREAM_CODE, runs from RAM) and log its prints.

Usage: vesc_stream.py file.lisp [max_seconds] [done_regex]
Polls motor current / input current / vin / fault / encoder every 2 s while it runs.
"""
import re, socket, struct, sys, time, importlib.util, os
spec = importlib.util.spec_from_file_location("v", os.path.join(os.path.dirname(os.path.abspath(__file__)), "vesc_term.py"))
v = importlib.util.module_from_spec(spec); spec.loader.exec_module(v)
STREAM, PRINT, REPL = 139, 135, 138
code = open(sys.argv[1], "rb").read()
max_s = float(sys.argv[2]) if len(sys.argv) > 2 else 60
done_re = re.compile(sys.argv[3] if len(sys.argv) > 3 else r"tau_r|Decay not captured|Error")
s = socket.create_connection(("127.0.0.1", 65102), timeout=3)
buf = bytearray(); t0 = time.time()

def log(msg): print(f"{time.time()-t0:6.1f}s  {msg}", flush=True)

def pump(timeout, want=None):
    end = time.time() + timeout
    while time.time() < end:
        s.settimeout(0.1)
        try: c = s.recv(4096)
        except socket.timeout: continue
        if not c: raise SystemExit("connection closed")
        buf.extend(c)
        for p in v.unpack_all(buf):
            if p[0] == STREAM and want is not None:
                off, res = struct.unpack(">ih", p[1:7])
                if off == want: return res
            elif p[0] == 21 and os.environ.get("POLL_TERM"):
                for ln in p[1:].decode(errors="replace").splitlines():
                    if any(k in ln for k in ("State:", "Slip:", "Voltage model", "i_mr:", "I/f", "Id:")): log("  | " + ln.strip())
            elif p[0] == PRINT:
                m = p[1:].decode(errors="replace").strip()
                if '"angle' in m: continue
                log(m)
                if done_re.search(m) and want is None: return "done"
    return None

off, CH = 0, 400
while off < len(code):
    chunk = code[off:off + CH]
    s.sendall(v.pack(bytes([STREAM]) + struct.pack(">iib", off, len(code), 0) + chunk))
    res = pump(5, want=off)
    if res != 0: raise SystemExit(f"stream failed at offset {off}: result {res}")
    off += len(chunk)
log(f"streamed {len(code)} bytes")
end = time.time() + max_s; last = 0
while time.time() < end:
    if time.time() - last > float(os.environ.get('POLL_S', '2')):
        if os.environ.get("POLL_TERM"):
            s.sendall(v.pack(bytes([20]) + os.environ["POLL_TERM"].encode()))
        else:
            s.sendall(v.pack(bytes([REPL]) + b"(list 'tel (get-current) (get-current-in) (get-vin) (get-fault) (get-temp-fet) (get-temp-mot))"))
        last = time.time()
    if pump(0.3) == "done":
        pump(1.5)
        break
s.sendall(v.pack(bytes([REPL]) + b"(set-current 0)"))
log("host: done")
