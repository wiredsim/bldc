; ACIM bench: no-load test. Magnetizing curve (Lm, Id_mag) and rotor time constant (tau_r).
;
; ACIM mode off. Motor clamped, no belt. Paste into VESC Tool > LispBM Scripting and Run.
; Stop at any time with the stop button in the LispBM tab.
;
; Part 1 spins the motor open-loop at a fixed electrical speed and steps the current down.
;   At no load the rotor turns at almost the field speed, so almost no rotor current flows
;   and the stator looks like Rs + j*w*Ls, with Ls = Lm + stator leakage, close to Lm.
;   Where L starts to fall with current, the iron is saturating: that is the knee.
; Part 2 lets go of the motor and records how the stator voltage dies away. With the stator
;   open, the rotor flux decays with exactly the rotor time constant Lr/Rr, so the time
;   between the 60% and 20% points is tau_r * ln(3). The voltage just after the release
;   also gives Lm without the dead-time error that affects part 1.
;
; Results print in the REPL. Part 2 also draws the decay in the Experiment plot
; (Realtime Data > Experiment).

(define erpm 6000.0)        ; field speed, ERPM. 6000 = 100 Hz electrical, about 1000 rpm at 6 pole pairs
(define currents (list 60.0 50.0 40.0 30.0 20.0 10.0))  ; A, high to low, all <= Motor Current Max
(define decay-cur 30.0)     ; A before the release in part 2. Re-run with your chosen Id_mag
(define ramp 5.0)           ; s to ramp the field from 0 to erpm
(define settle 2.0)         ; s at each current before measuring
(define meas 1.0)           ; s averaging time per point
(define rec 1.5)            ; s of decay to record

(define rs (conf-get 'foc-motor-r))
(define w (* 2.0 3.14159265 (/ erpm 60.0)))

(defun mag (a b) (sqrt (+ (* a a) (* b b))))
(defun maxf (a b) (if (> a b) a b))
(defun fmt (x f) (str-from-n x f))

(defun hold (cur t-len)
  (let ((t0 (systime)))
    (loopwhile (< (secs-since t0) t-len) {
        (foc-openloop cur erpm)
        (sleep 0.01)
    })))

(defun ramp-up (cur)
  (let ((t0 (systime)))
    (loopwhile (< (secs-since t0) ramp) {
        (foc-openloop cur (* erpm (/ (secs-since t0) ramp)))
        (sleep 0.01)
    })))

; One point: hold the current, average vd, vq, id, iq, print V, L and psi
(defun point (cur)
  {
    (hold cur settle)
    (get-vd 1) (get-vq 1) (get-id 1) (get-iq 1)
    (hold cur meas)
    (var v (mag (get-vd 1) (get-vq 1)))
    (var i (mag (get-id 1) (get-iq 1)))
    (var x (sqrt (maxf 0.0 (- (* v v) (* rs rs i i)))))
    (print (str-merge "I " (fmt i "%5.1f") " A   V " (fmt v "%6.3f")
                      " V   L " (fmt (* 1000000.0 (/ x (* w i))) "%7.1f")
                      " uH   psi " (fmt (* 1000.0 (/ x w)) "%6.3f") " mWb"))
  })

; Magnitude of the measured stator voltage, lightly smoothed
(define vs 0.0)
(defun vsample () (setq vs (+ (* 0.7 vs) (* 0.3 (mag (get-vd) (get-vq))))))

(print (str-merge "Rs " (fmt (* rs 1000.0) "%.2f") " mOhm, field " (fmt erpm "%.0f")
                  " ERPM (" (fmt (/ erpm 60.0) "%.1f") " Hz electrical)"))

; Offset of the voltage measurement with the motor stopped and released
(set-current 0)
(sleep 0.5)
(define floor-sum 0.0)
(looprange k 0 200 { (setq floor-sum (+ floor-sum (mag (get-vd) (get-vq)))) (sleep 0.002) })
(define vfloor (/ floor-sum 200.0))
(print (str-merge "Voltage reading at rest: " (fmt vfloor "%.3f") " V"))

(print "Part 1: magnetizing curve")
(ramp-up decay-cur)
(loopforeach c currents (point c))

(print "Part 2: flux decay")
(hold decay-cur settle)
(plot-init "Time (s)" "Voltage (V)")
(plot-add-graph "|V| stator")
(plot-set-graph 0)
(set-current 0)
(define t0 (systime))
(define v5 -1.0)
(define t60 -1.0)
(define t20 -1.0)
(setq vs (mag (get-vd) (get-vq)))
(loopwhile (< (secs-since t0) rec) {
    (vsample)
    (var ts (secs-since t0))
    (plot-send-points ts vs)
    (if (and (< v5 0.0) (> ts 0.005)) (setq v5 (- vs vfloor)))
    (if (and (> v5 0.0) (< t60 0.0) (< (- vs vfloor) (* 0.6 v5))) (setq t60 ts))
    (if (and (> v5 0.0) (< t20 0.0) (< (- vs vfloor) (* 0.2 v5))) (setq t20 ts))
    (sleep 0.001)
})

(if (or (< t60 0.0) (< t20 0.0))
    (print "Decay not captured: raise decay-cur or erpm, or make rec longer")
    {
      (var tau (/ (- t20 t60) (log 3.0)))
      (var v0 (* v5 (exp (/ 0.005 tau))))
      (print (str-merge "tau_r " (fmt (* tau 1000.0) "%.1f") " ms   (60% at "
                        (fmt (* t60 1000.0) "%.1f") " ms, 20% at " (fmt (* t20 1000.0) "%.1f") " ms)"))
      (print (str-merge "V at release " (fmt v0 "%.3f") " V -> Lm*Lm/Lr "
                        (fmt (* 1000000.0 (/ v0 (* w decay-cur))) "%.1f") " uH (multiply by Lr/Lm for Lm)"))
    })
