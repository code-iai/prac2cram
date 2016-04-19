(in-package :prac2cram)

;;; TODO: call the plan with a designator containing the action roles
;;; (subseq plan 0 (search " " plan))
(def-service-callback Prac2Cram (action_cores)
  (ros-info (basics-system) "Printing parameters: Action Cores: ~a  Plan: ~a" action_cores  (subseq plan 0 (search " " plan)))
  (make-response :status 0 :plan "Test String"))

;;; TODO Fehler abfangen, data structure erklären 

(defun prac2cram-server () 
  (with-ros-node ("prac2cram_server" :spin t)
     (register-service "prac2cram" 'Prac2Cram)
     (ros-info (basics-system) "This is the prac2cram server. I'm ready to start a CRAM query.")))

; use-measuring-cup `
     pour-from-container (action-cores-designator)
    (pour-from-container action-cores-designator)
    use-pipette (action-cores-designator)
    (use-pipette action-cores-designator)


    unscrew (action-cores-designator)
    (unscrew action-cores-designator)
    pull-object (action-cores-designator)
    (pull-object action-cores-designator)
    (roslisp:ros-info 'prac2cram "Plan called, will be executed now.!")
    )


(cram-language-implementation:declare-goal
    open-door (action-cores-designator)
    )

(cram-language-implementation:def-goal
    (open-door action-cores-designator)