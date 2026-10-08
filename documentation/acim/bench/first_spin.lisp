; ACIM bench: short ACIM runs. Requests a torque current for a fixed time, prints the ACIM
; state, flux, slip and speeds every 0.2 s, then lets go and keeps printing while the motor
; goes through HOLD to OFF.
;
; ACIM mode on. Motor clamped, no belt. Paste into VESC Tool > LispBM Scripting and Run.
; Stop at any time with the stop button in the LispBM tab. Use iq < 0 to run backwards.

(define iq 5.0)      ; A torque current request (I/f uses sqrt(Id_mag^2 + iq^2) at start)
(define secs 3.0)    ; s with the request on
(define after 1.5)   ; s to keep printing after the release

(define names (list "OFF" "FLUXING" "RUN" "HOLD" "FAULT"))
(defun fmt (x f) (str-from-n x f))

(defun report (ts)
  (print (str-merge (fmt ts "%5.2f") " s  " (ix names (acim-state))
                    "  i_mr " (fmt (acim-flux) "%5.1f")
                    " A  slip " (fmt (acim-slip) "%6.2f")
                    " Hz  rotor " (fmt (acim-wr) "%7.2f")
                    " Hz el  " (fmt (get-rpm) "%6.0f")
                    " ERPM  id " (fmt (get-id) "%5.1f")
                    " iq " (fmt (get-iq) "%5.1f"))))

(define t0 (systime))
(define tp -1.0)
(loopwhile (< (secs-since t0) (+ secs after)) {
    (if (< (secs-since t0) secs) (set-current iq) (set-current 0))
    (if (>= (secs-since t0) (+ tp 0.2)) {
        (setq tp (secs-since t0))
        (report tp)
    })
    (sleep 0.01)
})
(set-current 0)
