#!/usr/bin/env python3
"""Evaluate LispBM expressions on the VESC via VESC Tool's TCP bridge (COMM_LISP_REPL_CMD)."""
import socket, sys, time, importlib.util, os
spec = importlib.util.spec_from_file_location("v", os.path.join(os.path.dirname(os.path.abspath(__file__)), "vesc_term.py"))
v = importlib.util.module_from_spec(spec); spec.loader.exec_module(v)
COMM_LISP_REPL_CMD, COMM_LISP_PRINT = 138, 135

def repl(expr, wait=1.5, s=None):
    own = s is None
    if own:
        s = socket.create_connection(("127.0.0.1", 65102), timeout=3)
    s.sendall(v.pack(bytes([COMM_LISP_REPL_CMD]) + expr.encode()))
    buf, out, end = bytearray(), [], time.time() + wait
    while time.time() < end:
        s.settimeout(0.2)
        try: c = s.recv(4096)
        except socket.timeout:
            if out: break
            continue
        if not c: break
        buf += c
        for p in v.unpack_all(buf):
            if p and p[0] == COMM_LISP_PRINT and p[1:3] == b"> ":
                out.append(p[3:].decode(errors="replace"))
    if own: s.close()
    return "".join(out).strip() or "(no reply)"

if __name__ == "__main__":
    wait = float(os.environ.get("WAIT", "1.5"))
    s = socket.create_connection(("127.0.0.1", 65102), timeout=3)
    for e in sys.argv[1:]:
        print(f"{e}  =>  {repl(e, wait, s)}")
    s.close()
