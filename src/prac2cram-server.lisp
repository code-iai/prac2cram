(in-package :prac2cram)

;;; TODO: call the plan with a designator containing the action roles
;;; (subseq plan 0 (search " " plan))
(def-service-callback Prac2Cram (action_cores)
  (ros-info (basics-system) "Received service call with these parameters: Action Cores: ~a" action_cores)
  (make-response :status 0 :message "Test Message"))

; This is an example of what the server receives:
;  #([ACTIONCORE
;   ACTION_CORE_NAME:
;     Starting
;   ACTION_ROLES:
;     #([ACTIONROLE
;          ROLE_NAME:
;            obj_to_be_started
;          ROLE_VALUE:
;            centrifuge.n.01])]
;[ACTIONCORE
;   ACTION_CORE_NAME:
;     TurningOnElectricalDevice
;   ACTION_ROLES:
;     #([ACTIONROLE
;          ROLE_NAME:
;            device
;          ROLE_VALUE:
;            centrifuge.n.01])])


;;; we need these plans:
; unscrew
; pull-object
; open-door
; pour-from-container
; operate-tap
; pour-from-spice-jar
; use-spoon
; turn-on-device
; use-measuring-cup
; use-pipette


(defun prac2cram-server ()
  (with-ros-node ("prac2cram_server" :spin t)
     (register-service "prac2cram" 'Prac2Cram)
     (ros-info (basics-system) "This is the prac2cram server. I'm ready to start a CRAM query.")))



