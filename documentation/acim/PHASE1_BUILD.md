# ACIM support: Phase 1 build baseline

Baseline for the AC induction motor (ACIM) work on a MakerX HI200-C V2.2b.
Nothing in this phase changes firmware source.

## Base

- Upstream: vedderb/bldc, branch `release_7_00` (FW 7.00, `FW_TEST_VERSION_NUMBER 0`),
  commit `20cbb362687291242ab90b99f25fbfe8835540fc`.
  Upstream stopped tagging releases after 6.00; releases now live on `release_X_YY` branches.
- Work branch: `acim` (this file is the only change on top of the base).

## Hardware config

- Target: `go_foc_hi200` -> `hwconf/makerx/hw_go_foc_hi200.h` (+ `hw_go_foc_hi200.c`)
- `HW_NAME` = `"Go-FOC HI200"` (what VESC Tool should show as the hardware name)
- 3 phase shunts, 0.5 mOhm / 3, gain 20; Vin divider 56k/2.2k; 11-75 V limits;
  HW current limit +/-300 A, abs 400 A; dead time 1000 ns.
- A `go_foc_hi200_no_limits` variant exists (same HW_NAME, `DISABLE_HW_LIMITS`). Not used.

## Toolchain

Upstream pins ARM GCC `7-2018-q2-update`. developer.arm.com was unreachable from the build
container, so the build used GNU MCU Eclipse `7.3.1-1.1-20180724` (GCC 7.3.1 20180622,
ARM/embedded-7-branch rev 261907), which is built from the same 7-2018-q2 sources:
https://github.com/gnu-mcu-eclipse/arm-none-eabi-gcc/releases/tag/v7.3.1-1.1

    export PATH=<toolchain>/bin:$PATH
    make -j4 go_foc_hi200

## Result

Clean build, no compiler warnings.

    text    data   bss     dec     filename
    462544  2772   171992  637308  go_foc_hi200.elf
    flash2: 449660 B / 475120 B (94.64 %)  -> ~25 KB free for new code
    ram4:   63208 B / 62 KB (99.56 %)

`go_foc_hi200.bin` sha256 `afb8f84fdcf7c23b98419a037069fb25536af6ef909b97d96e9682bcc15e482c`

Flash headroom is tight. ACIM code must stay small (target < 8 KB) or use space
freed elsewhere.

Nothing has been flashed.
