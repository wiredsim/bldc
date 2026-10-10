; No-load release test: at each current, hold the field at erpm, release, and dump |V|^2 samples.
; Host side fits the decay. Prints:  REST <mean V^2>,  PT <I> <duty> <id> <iq> <rotor_rpm> <vin>,
; then lines "S k v2 v2 ..." with the samples, then "PTEND".
(define erpm 6000.0)
(define currents (list 40.0 30.0 20.0 15.0 10.0))
(define n 320)
(define buf (bufcreate (* 4 n)))
(define ea (get-encoder))
(define en 0.0)
(defun etk () (let ((b (get-encoder)) (d (- b ea)))
  { (setq en (+ en (if (> d 180) (- d 360) (if (< d -180) (+ d 360) d)))) (setq ea b) }))
(defun v2 () (+ (* (get-vd) (get-vd)) (* (get-vq) (get-vq))))
(defun drive (cur f t-len)
  (let ((t0 (systime)))
    (loopwhile (< (secs-since t0) t-len) { (foc-openloop cur (f (secs-since t0))) (etk) (sleep 0.01) })))
(defun record () {
  (var t0 (systime))
  (looprange k 0 n { (bufset-f32 buf (* 4 k) (v2)) (sleep 0.001) })
  (var dt (/ (secs-since t0) n))
  (print (str-merge "DT " (str-from-n (* dt 1000.0) "%.4f")))
  (looprange k 0 (/ n 16) {
    (var s "S")
    (looprange j 0 16 { (setq s (str-merge s " " (str-from-n (bufget-f32 buf (* 4 (+ (* k 16) j))) "%.4f"))) })
    (print s)
    (sleep 0.004)
  })
})

; offset at rest
(set-current 0)
(sleep 0.5)
(define acc 0.0)
(looprange k 0 400 { (setq acc (+ acc (v2))) (sleep 0.001) })
(print (str-merge "REST " (str-from-n (/ acc 400.0) "%.5f")))

; ramp at 30 A over 20 s
(drive 30.0 (fn (s) (* erpm (/ s 20.0))) 20.0)
(loopforeach c currents {
  (drive c (fn (s) erpm) 2.5)
  (get-duty) (get-id 1) (get-iq 1)
  (var n0 en)
  (var tm (systime))
  (var dsum 0.0) (var dn 0)
  (loopwhile (< (secs-since tm) 1.0) {
    (foc-openloop c erpm) (etk)
    (setq dsum (+ dsum (get-duty-abs))) (setq dn (+ dn 1))
    (sleep 0.01) })
  (var rpm (* 60.0 (/ (- en n0) (* 360.0 (secs-since tm)))))
  (var id (get-id 1)) (var iq (get-iq 1))
  (set-current 0)
  (record)
  (print (str-merge "PT " (str-from-n c "%.1f") " " (str-from-n (/ dsum dn) "%.5f") " " (str-from-n id "%.2f") " "
                    (str-from-n iq "%.2f") " " (str-from-n rpm "%.1f") " " (str-from-n (get-vin) "%.2f")))
  (print "PTEND")
})
(set-current 0)
(print "CURVE DONE")
