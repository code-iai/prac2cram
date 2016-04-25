(in-package :prac2cram)

;;; Make a test plan for debugging purposes.
;;; Note that for each plan we want prac2cram to be able to call we need:
;;;   the plan itself;
;;;   a function to generate arguments for the plan, and get an info string about the plan about to be run.
;;; We can also employ the plan info string to be an error report, if needed.

(cpl:def-top-level-cram-function test-plan (obj-desig)
  ;; TODO: also add calls to logging here. Because if we test functionality, let's be thorough :P
  (format t "Hello world! I just got this object designator through the prac2cram connection:~%~a~%isn't it awesome?~%" obj-desig))

(defun get-test-plan-args (&rest action-roles)
  ;; for convenience convert action-roles from list-of-messages to list-of-lists(-of-string)
  (let* ((action-roles (mapcar (lambda (action-role)
                                 (format t "action-role ~a~%" action-role)
                                 (roslisp:with-fields ((role-name role_name) (role-value role_value)) action-role
                                   (list role-name role-value)))
                               action-roles))
         (action-role-count (length action-roles)))
    ;; For this test plan, let's say we expect EXACTLY 1 action role, of name 'test-flag'
    (if (equal action-role-count 1)
      (if (equal "test-flag" (car (car action-roles)))
        (values T
                (desig:make-designator 'object (list (list 'test-flag (second (car action-roles)))))
                (format nil "test-plan with parameter obj-desig:((test-flag ~a))" (second (car action-roles))))
        (values nil nil (format nil "expected test-flag as action role, instead got ~a" (car (car action-roles)))))
      (values nil nil (format nil "expected exactly one role, got ~a" action-role-count)))))

;;; TODO: call the plan with a designator containing the action roles

(defun start-plan (plan-fn args)
  (sb-thread:make-thread (lambda ()
                           (apply plan-fn args))))

(defun prepare-plan (plan-fn prep-args-fn action-roles)
  (let* ((action-roles (coerce action-roles 'list)))
    ;; We give the prep-args function the opportunity to decide whether to run the plan or not (maybe arguments
    ;; missing, can't be resolved, whatever).
    (multiple-value-bind (should-start args plan-info) (apply prep-args-fn action-roles)
      (if should-start
        (start-plan plan-fn args))
      (values T plan-info))))


;;action_core: Pouring
;;action_roles: ['stuff','goal','action_verb']
;;cram_plan: "(pour-from-container (from (an object (type container.n.01)
;;                                                    (contains (some stuff (type {stuff}))))
;;                                   (to (an object (type {goal})))))"


(defun find-and-start-plan (action-cores)
  ;; At the moment, assumes the last action core is the most specific, and that only the most specific is needed.
  (let* ((action-core (car (last (coerce action-cores 'list)))))
    (roslisp:with-fields ((action-core-name action_core_name) (action-roles action_roles)) action-core
      (cond
        ((equalp action-core-name "dbg-prac2cram")
          (prepare-plan #'test-plan #'get-test-plan-args action-roles))
        (T
          (values nil (format nil "no plan found to match the action core ~a" action-core-name)))))))

;;; (subseq plan 0 (search " " plan))
(roslisp:def-service-callback Prac2Cram (action_cores)
  (roslisp:ros-info (basics-system) "Received service call with these parameters: Action Cores: ~a" action_cores)

  (if cpl:*task-tree*  ;; test if another cram plan is running; *task-tree* will be non-NIL if so
    (roslisp:make-response :status 0 :message "Another cram plan is running. Wait for it to finish and send the request again.")
    (multiple-value-bind (plan-started plan-info) (find-and-start-plan action_cores)  ;; identify plan based on action_core_name
      (if plan-started
        (make-response :status 1 :message (format nil "Started plan: ~a." plan-info))
        (make-response :status 0 :message (format nil "No plan started: ~a." plan-info)))))
  )

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
  (roslisp:with-ros-node ("prac2cram_server" :spin t)
     (roslisp:register-service "prac2cram" 'Prac2Cram)
     (roslisp:ros-info (basics-system) "This is the prac2cram server. I'm ready to start a CRAM query.")))



