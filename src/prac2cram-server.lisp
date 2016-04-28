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
                (list (desig:make-designator 'object (list (list 'test-flag (second (car action-roles))))))
                (format nil "test-plan with parameter obj-desig:((test-flag ~a))" (second (car action-roles)))
                (format nil "(testplan (test-flag ~a))" (second (car action-roles))))
        (values nil 
                nil 
                (format nil "expected test-flag as action role, instead got ~a" (car (car action-roles)))
                (format nil "")))
      (values nil 
              nil
              (format nil "expected exactly one role, got ~a" action-role-count)
              (format nil "")))))


(defparameter *plan-matchings* (acons "dbg-prac2cram" (list #'test-plan #'get-test-plan-args) nil))

(defun start-plan (plan-fn args)
  (sb-thread:make-thread (lambda ()
                           (apply plan-fn args))))

(defun prepare-plan (plan-fn prep-args-fn &rest action-roles)
  ;; We give the prep-args function the opportunity to decide whether to run the plan or not (maybe arguments
  ;; missing, can't be resolved, whatever).
  (multiple-value-bind (should-start args plan-info plan-string) (apply prep-args-fn action-roles)
    (if should-start
      (start-plan plan-fn args))
    (values T plan-info plan-string)))

(defun find-and-start-plan (action-cores plan-matchings)
  ;; At the moment, assumes the last action core is the most specific, and that only the most specific is needed.
  (let* ((action-core (car (last (coerce action-cores 'list)))))
    (roslisp:with-fields ((action-core-name action_core_name) (action-roles action_roles)) action-core
      (let* ((action-roles (coerce action-roles 'list))
             (matching-plan (cdr (assoc (format nil "~a" action-core-name) plan-matchings :test #'equal)))
             (fn-args (append matching-plan action-roles)))
        (if matching-plan
          (apply #'prepare-plan fn-args)
          (values nil 
                  (format nil "no plan found to match the action core ~a (known are ~a)" action-core-name plan-matchings)
                  ""))))))

(defun interpret-task (task plan-matchings)
  (roslisp:with-fields ((action-cores action_cores)) task
    (let* ((action-core (car (last (coerce action-cores 'list)))))
      (roslisp:with-fields ((action-core-name action_core_name) (action-roles action_roles)) action-core
        (let* ((action-roles (coerce action-roles 'list))
               (matching-plan-data (cdr (assoc (format nil "~a" action-core-name) plan-matchings :test #'equal))))
          (if matching-plan-data
            (let* ((matching-plan-fn (first matching-plan-data))
                   (matching-arg-fn (second matching-plan-data)))
              (multiple-value-bind (should-start args message plan-string) (apply matching-arg-fn action-roles)
                (list should-start message plan-string matching-plan-fn matching-arg-fn args)))
            (list -1 (format nil "no plan found for action core ~a." action-core-name) "" nil nil nil)))))))

(defun find-and-start-plans (tasks plan-matchings)
  (let* ((plan-data (mapcar (lambda (task) 
                              (interpret-task task plan-matchings))
                            tasks))
         (statuses (mapcar #'first plan-data))
         (messages (mapcar #'second plan-data))
         (plan-strings (mapcar #'third plan-data))
         (plan-fns (mapcar #'fourth plan-data))
         (plan-arg-fns (mapcar #'fifth plan-data))
         (args-list (mapcar #'sixth plan-data))
         (started (if (member -1 statuses) -1 0)))
    (if (equal started 0)
      (sb-thread:make-thread (lambda ()
                               (mapcar (lambda (plan-fn plan-arg-fn args)
                                         ;; in the future, compute arguments for a plan just before running it.
                                         (declare (ignore plan-arg-fn))
                                         (apply plan-fn args))
                                       plan-fns plan-arg-fns args-list))))
    (values started statuses messages plan-strings)))

;;(lambda (req) 
;;  (roslisp:with-fields (tasks) req
;;    (Prac2Cram tasks plan-matchings)))

(roslisp:def-service-callback Prac2Cram (tasks)
  (let* ((tasks (coerce tasks 'list)))
    (roslisp:ros-info (basics-system) "Received service call with these parameters: Tasks: ~a" tasks)
    (if cpl:*task-tree*   ;; test if another cram plan is running; *task-tree* will be non-NIL if so
      (roslisp:make-response :status -1
                             :individual_status (vector)
                             :messages (vector "Another cram plan is running. Wait for it to finish and send the request again.")
                             :plan_strings (vector ))
      (multiple-value-bind (plan-started statuses messages plan-strings)
                           (find-and-start-plans tasks *plan-matchings*)  ;; identify plan based on action_core_name
        (if plan-started
          (roslisp:make-response :status 0
                                 :individual_status (coerce statuses 'list)
                                 :messages (coerce messages 'list)
                                 :plan_strings (coerce plan-strings 'list))
          (roslisp:make-response :status -1
                                 :individual_status (coerce statuses 'list)
                                 :messages (coerce messages 'list)
                                 :plan_strings (coerce plan-strings 'list)))))))


(defun prac2cram-server (plan-matchings)
  ;; put the debugging/test plan into the plan-matchings alist to ensure it's there.
  (let* ((plan-matchings (acons "dbg-prac2cram" (list #'test-plan #'get-test-plan-args) plan-matchings)))
    (setf *plan-matchings* plan-matchings)
       (roslisp:register-service "prac2cram" 
                                 'Prac2Cram)
       (roslisp:ros-info (basics-system) "This is the prac2cram server. I'm ready to start a CRAM query. Known plan matchings ~a~%" *plan-matchings*)))



