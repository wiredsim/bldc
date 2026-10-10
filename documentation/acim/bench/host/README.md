# Bench tools

Python scripts that talk to the board through VESC Tool's TCP server (Connection > TCP Server,
port 65102), so VESC Tool stays connected and shows everything live. Only one TCP client at a
time. Standard library only.

| Script | Does |
|---|---|
Build target for this bench: **`make 100_250_hi200_bsg`** (`hwconf/acim_bench/`). It is the
100_250 config with the bench values as motor config defaults, because the VESC bootloader
erases the emulated EEPROM (flash sectors 1-2) on every update, so a reflash lands on these
defaults instead of 400 A abs current, -200 A regen and no ERPM limit. The ACIM page still
resets to disabled; run the restore script after flashing.

| `restore_bench_config.sh` | Writes the full bench motor config and ACIM page. **Run after every reflash**: flashing resets both to firmware defaults (60 A, 400 A abs, -200 A regen, 96 V, no ERPM limit, phase filters on) |
| `vesc_mcconf.py` | Read or set any motor config field by its C name (`foc_dt_us`, `l_current_max`, ...). The byte layout is parsed from `confgenerator.c`, every other byte is sent back unchanged, and the write is read back and checked |
| `vesc_acim_conf.py` | Read or set the ACIM page (custom config 0) |
| `vesc_term.py "cmd"` | Run a terminal command (`acim_status`, `faults`, `measure_res 20`) |
| `vesc_repl.py "(expr)"` | Evaluate LispBM. After a `conf-set` the board spends about 3 s applying the config and silently drops REPL commands that arrive meanwhile; commands over about 512 bytes are also dropped silently |
| `vesc_set.py name=value` | Set a LispBM `conf-set` value and read it back, retrying lost replies |
| `vesc_stream.py file.lisp [secs] [done_regex]` | Stream a LispBM script (runs from RAM, flash untouched) and log its prints. `POLL_TERM=acim_status POLL_S=0.25` polls a terminal command while it runs |
| `vesc_rl.py` | Measure R and L (the VESC Tool button) |
| `rsweep.py` | `measure_res` at 5 to 30 A and a fit of V = R*I + V0, where V0 is the uncompensated dead-time voltage |
