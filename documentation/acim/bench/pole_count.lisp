; ACIM bench: pole pairs without the encoder.
;
; Turns a rotating current vector at a fixed electrical speed for a fixed time. Count the
; shaft turns with a tape mark (or film it) between "COUNT START" and "COUNT STOP".
;   pole pairs = field turns / shaft turns, rounded DOWN (slip makes the rotor slower)
;
; ACIM mode off. Motor clamped, no belt. Paste into VESC Tool > LispBM Scripting and Run.
; Stop at any time with the stop button in the LispBM tab.

(define cur 20.0)     ; A. Raise in steps of 10 A if the shaft stalls or jerks
(define erpm 300.0)   ; field speed in electrical RPM (300 ERPM = 5 Hz electrical)
(define ramp 3.0)     ; s to ramp the field from 0 to erpm
(define secs 60.0)    ; counting time

(defun run-for (t-len f)
  (let ((t0 (systime)))
    (loopwhile (< (secs-since t0) t-len) {
        (foc-openloop cur (f (secs-since t0)))
        (sleep 0.01)
    })))

(print "Ramping the field")
(run-for ramp (fn (s) (* erpm (/ s ramp))))
(run-for 2.0 (fn (s) erpm))
(print "COUNT START")
(run-for secs (fn (s) erpm))
(print "COUNT STOP")
(set-current 0)

(print (str-merge "Field turns: " (str-from-n (* erpm (/ secs 60.0)) "%.1f")
                  "  -> pole pairs = field turns / shaft turns, rounded down"))
