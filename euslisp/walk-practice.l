(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(samplerobot-init)
(setq *robot* *sr*)

(defun make-footstep-list (step-len step-num)
  (let* ((footstep-list (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg) (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg)))
        (rleg-y (elt (send *robot* :rleg :end-coords :worldpos) 1))
        (lleg-y (elt (send *robot* :lleg :end-coords :worldpos) 1))
        (support-leg-param (list :rleg rleg-y)))

    (dotimes (i step-num)
      (setq footstep-list (append footstep-list (list (make-coords :name (elt support-leg-param 0)
                                                             :coords (send (send (car (last footstep-list)) :copy-worldcoords) :translate (float-vector step-len (* 2 (elt support-leg-param 1)) 0))))))
      (if (eq :rleg (elt support-leg-param 0))
        (progn
          (setf (elt support-leg-param 0) :lleg)
          (setf (elt support-leg-param 1) lleg-y))
        (progn
          (setf (elt support-leg-param 0) :rleg)
          (setf (elt support-leg-param 1) rleg-y))
        )
      )
    footstep-list)
  )

(defmethod gait-generator
  (:proc-one-tick
    (&key (type :shuffling) (solve-angle-vector :solve-av-by-move-centroid-on-foot)
          (solve-angle-vector-args) (debug nil))
    (let* ((preview-data ;; data to push to preview-controller's que
            (if support-leg-coords-list
                (send self :calc-one-tick-gait-parameter type)))
           (dd (send apreview-controller :update-xk ;; return value from preview-controller
                     (if preview-data (elt preview-data 3)) preview-data))
           (gp (send apreview-controller :current-additional-data))) ;; gait parameter
      (if dd (setf (elt (cadr dd) 2) (elt gp 4))) ;; overwrite 
      (let ((ret (if (and dd solve-angle-vector)
                     (append (send self :solve-angle-vector
                                   (elt gp 0) (elt gp 1) (elt gp 2) (cadr dd)
                                   :solve-angle-vector solve-angle-vector
                                   :solve-angle-vector-args solve-angle-vector-args)
                             (if debug (append (butlast gp 2) (list (cadr dd)
                                                                  (send apreview-controller :cart-zmp))
                                               (last gp)
                                               ))))))
        (when (>= 0 (decf index-count))
          (send self :update-current-gait-parameter))
        ret)))

  (:solve-angle-vector
    (support-leg support-leg-coords swing-leg-coords cog
     &key (solve-angle-vector :solve-av-by-move-centroid-on-foot)
          (solve-angle-vector-args))
    (let ((ik-ret
           (cond
            ((functionp solve-angle-vector)
             (apply solve-angle-vector
                    support-leg support-leg-coords
                    swing-leg-coords cog robot solve-angle-vector-args))
            ((and (symbolp solve-angle-vector) (find-method self solve-angle-vector))
             (send* self solve-angle-vector
                    support-leg support-leg-coords
                    swing-leg-coords cog robot solve-angle-vector-args))
            (t (error ";; in :solve-angle-vector, invalid function or method~%!")))))
      (list ik-ret (send (car (send robot :links)) :copy-worldcoords))))

  ;; solve-angle-vector methods
  (:solve-av-by-move-centroid-on-foot
   (support-leg support-leg-coords swing-leg-coords cog robot
   &rest args &key (cog-gain 3.5) (stop 100) (additional-nspace-list) &allow-other-keys)
   (let* ((legs (append (send self :get-counter-footstep-limbs support-leg)
                        support-leg))
          (leg-order (mapcar #'(lambda (l) (position l legs)) all-limbs))
          (fix-coords (append swing-leg-coords support-leg-coords)))
     (unless (memq :thre args)
       (append args (list :thre (mapcar #'(lambda (x) (if (memq x '(:rleg :lleg)) ik-thre (* 5 ik-thre))) all-limbs))))
     (unless (memq :rthre args)
       (append args (list :rthre (mapcar #'(lambda (x) (deg2rad (if (memq x '(:rleg :lleg)) ik-rthre (* 5 ik-rthre)))) all-limbs))))
     (send* robot :move-centroid-on-foot
            :both all-limbs :target-centroid-pos cog
            :fix-limbs-target-coords (mapcar #'(lambda (idx) (elt fix-coords idx)) leg-order)
            :cog-gain cog-gain :stop stop
            :additional-nspace-list
            (append
             additional-nspace-list
             (list (list (car (send robot :links))
                         #'(lambda ()
                             (let* ((fcoords (apply #'midcoords 0.5
                                                    (mapcar #'(lambda (x) (elt fix-coords (position x legs))) '(:rleg :lleg))))
                                    (xvr (send robot :rotate-vector (case (length legs) (2 #f(1 0 0)) (4 #f(0 0 1)))))
                                    (xvf (send fcoords :rotate-vector #f(1 0 0))))
                               (if (> (elt (send fcoords :inverse-rotate-vector xvr) 0) 0) (setq xvr (scale -1 xvr)))
                               (dolist (xv (list xvr xvf)) (setf (elt xv 2) 0.0))
                               (let ((dth (if (and (eps= (norm xvf) 0.0) (eps= (norm xvr) 0.0))
                                              0.0 (asin (elt (v* (normalize-vector xvf) (normalize-vector xvr)) 2)))))
                                 (float-vector 0 0 0 0 0 dth)
                                 ))))))
            args)
     (while (send robot (elt support-leg 0) :move-end-pos #f(0 0 -10) :world :warnp nil)
        (send robot (if (eq (elt support-leg 0) :rleg) :lleg :rleg) :move-end-pos #f(0 0 -10))
        (send robot :translate #f(0 0 10))
        )
     (send robot :angle-vector)))
  )
