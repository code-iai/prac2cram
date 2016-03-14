(in-package :prac2cram)

;;; TODO: call the plan with a designator containing the action roles
;;; (subseq plan 0 (search " " plan))
(def-service-callback Prac2Cram (action_cores plan)
  (ros-info (basics-system) "Printing parameters: Action Cores: ~a  Plan: ~a" action_cores  (subseq plan 0 (search " " plan)))
  (make-response :status 0))

;;; TODO params erklären

(defun prac2cram-server ()
  (with-ros-node ("prac2cram_server" :spin t)
     (register-service "prac2cram" 'Prac2Cram)
     (ros-info (basics-system) "This is the prac2cram server. I'm ready to start a CRAM query. Please send me a list of action_cores and a plan.")))

;;; we need these plans:
; unscrew
; pull-object
; open-door
; pour-from-container
; operate-tap
; pour-from-spice-jar
; use-spoon
; turn-on-device
; use-measuring-cup `
; use-pipette

(cram-language-implementation:declare-goal
     pour-from-container (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (pour-from-container action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )


(cram-language-implementation:declare-goal
    use-pipette (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (use-pipette action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )




(cram-language-implementation:declare-goal
    unscrew (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (unscrew action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )


(cram-language-implementation:declare-goal
    pull-object (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (pull-object action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )


(cram-language-implementation:declare-goal
    open-door (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (open-door action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )


(cram-language-implementation:declare-goal
    operate-tap (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (operate-tap action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )


(cram-language-implementation:declare-goal
    pour-from-spice-jar (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (pour-from-spice-jar action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )


(cram-language-implementation:declare-goal
    use-spoon (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (use-spoon action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )


(cram-language-implementation:declare-goal
    turn-on-device (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (turn-on-device action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )


(cram-language-implementation:declare-goal
    use-measuring-cup (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (use-measuring-cup action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )






