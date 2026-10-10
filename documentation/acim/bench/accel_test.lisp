; ACIM bench: acceleration time at a fixed current, for the rotor time constant sweep.
;
; ACIM mode on, Speed Source Encoder (after acim_enc_check passes). Motor clamped, no belt.
; Run once per Rotor Time Constant setting. With nothing on the shaft but its own inertia,
; the shortest time from erpm-a to erpm-b means the most torque per amp.
; Paste into VESC Tool > LispBM Scripting and Run. Stop at any time with the stop button.

(define iq 15.0)        ; A torque current request
(define erpm-a 600.0)   ; timing starts here (rotor ERPM)
(define erpm-b 2400.0)  ; timing stops here. Keep below Max ERPM
(define runs 3)
(define give-up 20.0)   ; s per run before giving up

(defun fmt (x f) (str-from-n x f))
(defun maxf (a b) (if (> a b) a b))

(defun wait-slow ()
  (let ((t0 (systime)))
    {
      (set-current 0)
      (if (> (abs (get-rpm)) (* 0.5 erpm-a)) (print "Waiting for the rotor to coast down"))
      (loopwhile (and (> (abs (get-rpm)) (* 0.5 erpm-a)) (< (secs-since t0) 120.0)) (sleep 0.1))
      (sleep 1.0)
    }))

(defun one-run (n)
  (let ((t0 (systime)) (ta -1.0) (tb -1.0) (iin 0.0) (flux 0.0) (slip 0.0))
    {
      (wait-slow)
      (setq t0 (systime))
      (loopwhile (and (< tb 0.0) (< (secs-since t0) give-up)) {
          (set-current iq)
          (var r (abs (get-rpm)))
          (setq iin (maxf iin (get-current-in)))
          (if (and (< ta 0.0) (>= r erpm-a)) (setq ta (secs-since t0)))
          (if (and (>= ta 0.0) (>= r erpm-b)) {
              (setq tb (secs-since t0))
              (setq flux (acim-flux))
              (setq slip (acim-slip))
          })
          (sleep 0.002)
      })
      (set-current 0)
      (if (< tb 0.0)
          (print (str-merge "Run " (str-from-n n) ": did not reach erpm-b in " (fmt give-up "%.0f")
                            " s, state " (str-from-n (acim-state))))
          (print (str-merge "Run " (str-from-n n) ": " (fmt (- tb ta) "%.3f") " s   i_mr "
                            (fmt flux "%.1f") " A   slip " (fmt slip "%.2f") " Hz   max input "
                            (fmt iin "%.1f") " A")))
    }))

(print (str-merge "Iq " (fmt iq "%.1f") " A, " (fmt erpm-a "%.0f") " -> " (fmt erpm-b "%.0f") " ERPM"))
(looprange n 1 (+ runs 1) (one-run n))
(set-current 0)
(print "Done. If max input is close to Battery Current Max, lower erpm-b or iq")
