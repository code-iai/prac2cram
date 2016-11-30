;;;
;;; Copyright (c) 2016, Mihai Pomarlan <blandc@cs.uni-bremen.com>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :prac2cram)

(defparameter plan-running (cpl:make-fluent :value nil))
(defparameter plan-error (cpl:make-fluent :value nil))

;;; Make a test plan for debugging purposes.
;;; Note that for each plan we want prac2cram to be able to call we need:
;;;   the plan itself;
;;;   a function to generate arguments for the plan, and get an info string about the plan about to be run.
;;; We can also employ the plan info string to be an error report, if needed.

(cpl:def-top-level-cram-function test-plan (obj-desig)
  ;; TODO: also add calls to logging here. Because if we test functionality, let's be thorough :P
  (format t "Hello world! I just got this object (pseudo-)designator through the prac2cram connection:~%~a~%isn't it awesome?~%" obj-desig))

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
        (values 0
                (list (list 'an 'object (list (list 'test-flag (second (car action-roles))))))
                (format nil "test-plan with parameter obj-desig:((test-flag ~a))" (second (car action-roles)))
                (format nil "(testplan (test-flag ~a))" (second (car action-roles))))
        (values -1 
                nil 
                (format nil "expected test-flag as action role, instead got ~a" (car (car action-roles)))
                (format nil "")))
      (values -1 
              nil
              (format nil "expected exactly one role, got ~a" action-role-count)
              (format nil "")))))

;;;; PRAC2CRAM interface (the one used for the ACAT review demo)

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
                                       plan-fns plan-arg-fns args-list)
                               (setf (cpl:value plan-running) nil))))
    (values started statuses messages plan-strings)))

(roslisp:def-service-callback Prac2Cram (tasks)
  (let* ((tasks (coerce tasks 'list)))
    (roslisp:ros-info (basics-system) "Received service call with these parameters: Tasks: ~a" tasks)
    (if plan-running
      (roslisp:make-response :status -1
                             :individual_status (vector)
                             :messages (vector "Another cram plan is running. Wait for it to finish and send the request again.")
                             :plan_strings (vector ))
      (multiple-value-bind (plan-started statuses messages plan-strings)
                           (find-and-start-plans tasks *plan-matchings*)  ;; identify plan based on action_core_name
        (if plan-started
          (progn
            (setf (cpl:value plan-running) T)
            (roslisp:make-response :status 0
                                   :individual_status (coerce statuses 'list)
                                   :messages (coerce messages 'list)
                                   :plan_strings (coerce plan-strings 'list)))
          (roslisp:make-response :status -1
                                 :individual_status (coerce statuses 'list)
                                 :messages (coerce messages 'list)
                                 :plan_strings (coerce plan-strings 'list)))))))
(defparameter *pub-tick* nil)
(defun ensure-tick-publisher ()
  (if *pub-tick*
    *pub-tick*
    (progn
      (setf *pub-tick* (roslisp:advertise "cramticks" "prac2cram/CRAMTick" :latch nil))
      (roslisp:wait-duration 2.0)
      *pub-tick*)))

(defun destroy-tick-publisher ()
  (setf *pub-tick* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-tick-publisher)

(defun send-tick ()
  (loop
    (let* ((done (not (cpl:value plan-running)))
           (error (cpl:value plan-error))
           (done (if done 1 0))
           (error (if error 1 0)))
      (roslisp:publish (ensure-tick-publisher)
                       (roslisp:make-message "prac2cram/CRAMTick"
                                             :done done
                                             :error error))
      (roslisp:wait-duration 1))))

(defun prac2cram-server (plan-matchings &optional (prac-url "http://localhost:1234"))
  ;; put the debugging/test plan into the plan-matchings alist to ensure it's there.
  (let* ((plan-matchings (acons "dbg-prac2cram" (list #'test-plan #'get-test-plan-args) plan-matchings)))
    (setf *prac-url* prac-url)
    (setf (cpl:value plan-running) nil)
    ;; Subscribe to clock to create a tick-thread-- notify the ROS world that this prac2cram server is still running
    (sb-thread:make-thread (lambda () (send-tick)))
    (if (not (roslisp:wait-for-service "/prac2cram_http_bridge" 0.1))
      (sb-thread:make-thread (lambda ()
                               ;;(sb-ext:run-program "rosrun" (list "prac2cram" "prac2cram_HTTP_bridge"))
                               (format t "WARNING: parameter substitution request service not available. Better not fall into that branch, eh?~%"))))
    (setf *plan-matchings* plan-matchings)
       (roslisp:register-service "prac2cram" 
                                 'Prac2Cram)
       (roslisp:ros-info (basics-system) "This is the prac2cram server. I'm ready to start a CRAM query. Known plan matchings ~a~%" *plan-matchings*)))



