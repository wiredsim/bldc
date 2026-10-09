#!/usr/bin/env python3
"""Set VESC config values via the REPL and verify each by reading it back (retries lost packets).

Usage: vesc_set.py name=value [name=value ...]   e.g. l-current-max=40
Exits non-zero if any value can't be verified.
"""
import socket, sys, importlib.util, os
spec = importlib.util.spec_from_file_location("r", os.path.join(os.path.dirname(os.path.abspath(__file__)), "vesc_repl.py"))
r = importlib.util.module_from_spec(spec); spec.loader.exec_module(r)
ok_all = True
s = socket.create_connection(("127.0.0.1", 65102), timeout=3)
for arg in sys.argv[1:]:
    name, val = arg.split("=")
    want = float(val)
    for attempt in range(5):
        r.repl(f"(conf-set '{name} {want})", 2.0, s)
        got = r.repl(f"(conf-get '{name})", 2.0, s)
        try:
            if abs(float(got.replace("f32", "")) - want) < 1e-3 * max(1, abs(want)):
                print(f"{name} = {got}"); break
        except ValueError:
            pass
    else:
        print(f"{name}: NOT VERIFIED (last reply {got})"); ok_all = False
s.close()
sys.exit(0 if ok_all else 1)
