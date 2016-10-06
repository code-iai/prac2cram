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

(declaim (ftype function generate-ptr-code-for-action-core))
(defun generate-ptr-code-if-branch (branch plan-matchings error-log plan-string tab-string)
  (let* (
         (branchal (alexandria:hash-table-alist branch))
         (branch-condition (cdr (assoc "condition" branchal :test #'equal)))
         (branch-body (cdr (assoc "body" branchal :test #'equal)))
         ;;(branch-condition (gethash "condition" branch))
         ;;(branch-body (gethash "body" branch))
         (dummy (setf (plan-string plan-string) (format nil "~a~a(" (plan-string plan-string) tab-string)))
         (fn-app-list (if branch-condition
                        (let* ((fn-cond (generate-ptr-code-for-action-core branch-condition plan-matchings error-log plan-string ""))
                               (fn-app-list (mapcar (lambda (step)
                                                      (generate-ptr-code-for-action-core step plan-matchings error-log plan-string (format nil "~a   " tab-string)))
                                                    branch-body)))
                          (cons fn-cond fn-app-list))
                        nil))
         (dummy2 (setf (plan-string plan-string) (format nil "~a~a)~%" (plan-string plan-string) tab-string))))
    (declare (ignore dummy) (ignore dummy2))
;;(format t "BRANCH CONDITION ~a ~%" branch-condition)
;;(format t "    Branch data ~a~%" (alexandria:hash-table-alist branch))
;;(format t "    Test function ~a~%" (hash-table-test branch))
    (if branch-condition
      (progn
        (make-fn-app (lambda ()
                       (let* ((fn-app-list (make-instance 'function-application-list :fn-list fn-app-list)))
                         (ptr-seq fn-app-list)))))
      (progn
        (setf (plan-string plan-string) (format nil "~a~a<NO PLAN FOR EMPTY CONDITION>~%" (plan-string plan-string) tab-string))
        (setf (errors error-log) 
              (cons (make-instance 'prac2cram-error
                                   :message (format nil "IF-branch has empty condition role."))
                    (errors error-log)))
        (make-fn-app (lambda ()))))))

(defun generate-ptr-code-if (action-core plan-matchings error-log plan-string tab-string)
  (let* ((action-core (specialize-action-core action-core))
         (action-core-arguments (alexandria:hash-table-alist (action-core-arguments action-core)))
         (branches (cdr (assoc "branches" action-core-arguments :test #'equal)))
         (dummy (if branches 
                  (setf (plan-string plan-string) 
                        (format nil "~a~a(COND~%" (plan-string plan-string) tab-string))))
         (fn-app-list (mapcar (lambda (branch)
                                (generate-ptr-code-if-branch branch plan-matchings error-log plan-string (format nil "~a  " tab-string)))
                              branches))
         (dummy2 (if branches
                   (setf (plan-string plan-string)
                         (format nil "~a~a)~%" (plan-string plan-string) tab-string))))
         (fn-app-list (make-instance 'function-application-list :fn-list fn-app-list)))
    (declare (ignore dummy) (ignore dummy2))
    (if branches
      (progn
        (make-fn-app (lambda ()
                       (ptr-try-all fn-app-list))))
      (progn
        (setf (plan-string plan-string) (format nil "~a~a<NO PLAN FOR EMPTY CONDITIONAL>~%" (plan-string plan-string) tab-string))
        (setf (errors error-log) 
              (cons (make-instance 'prac2cram-error
                                   :message (format nil "Action-core IF has no role values for branches."))
                    (errors error-log)))
        (make-fn-app (lambda ()))))))

(defun generate-ptr-code-loop (action-core plan-matchings error-log plan-string tab-string)
  (let* ((action-core (specialize-action-core action-core))
         (action-core-arguments (alexandria:hash-table-alist (action-core-arguments action-core)))
         (action (cdr (assoc "action" action-core-arguments :test #'equal)))
         (condition (cdr (assoc "condition" action-core-arguments :test #'equal)))
         (loop-type (cdr (assoc "type" action-core-arguments :test #'equal)))
         (ran-once (cpl-impl:make-fluent :value nil))
         (must-run-once (equal loop-type "do-until"))
         (all-ok (and condition loop-type action))
         (dummy (if all-ok
                  (setf (plan-string plan-string)
                        (format nil "~a~a(~a~%"
                                    (plan-string plan-string) tab-string (string-upcase loop-type)))
                  (progn
                    (setf (plan-string plan-string) (format nil "~a~a<NO PLAN BECAUSE MISSING LOOP ROLE>~%" (plan-string plan-string) tab-string))
                    (setf (errors error-log)
                          (cons (make-instance 'prac2cram-error
                                               :message (format nil "Action-core LOOP has at least one missing role value."))
                                (errors error-log))))))
         (condition (when all-ok (generate-ptr-code-for-action-core condition plan-matchings error-log plan-string (format nil "~a   " tab-string))))
         (action (when all-ok (generate-ptr-code-for-action-core action plan-matchings error-log plan-string (format nil "~a   " tab-string))))
         (dummy2 (if all-ok
                   (setf (plan-string plan-string)
                         (format nil "~a~a)~%" (plan-string plan-string) tab-string))))
         (all-ok (and all-ok (not (errors error-log)))))
    (declare (ignore dummy) (ignore dummy2))
    (if all-ok
      (make-fn-app (lambda ()
                     (cpl-impl:pursue
                       ;; code branch
                       (loop
                         (funcall (function-application/function-object action))
                         (setf (cpl-impl:value ran-once) t))
                       ;; monitor branch
                       (progn
                         (if must-run-once
                           (cpl-impl:wait-for ran-once))
                         (funcall (function-application/function-object condition))))))
      (make-fn-app (lambda ())))))

(defun generate-ptr-code-default (action-core plan-matchings error-log plan-string tab-string)
  (let* ((action-core (specialize-action-core action-core))
         (action-core-name (action-core-name action-core))
         (action-core-arguments (action-core-argument-string action-core))
         (matching (get-matching action-core-name plan-matchings))
         (matching-plan (first matching))
         (matching-arg-fn (second matching))
         (matching-plan-name (third matching)))
    (if (and matching-plan matching-arg-fn)
      (progn
        (setf (plan-string plan-string) (format nil "~a~a(~a ~a)~%" (plan-string plan-string) tab-string matching-plan-name action-core-arguments))
        (make-fn-app (lambda ()
                       (let* ((action-core-arguments (action-core-arguments action-core))
                              (tried-values (make-hash-table :test #'equal)))
                         (cpl-impl:with-failure-handling
                           ((prac2cram-argument-invalid-error (e)
                              (let* ((failed-role-name (role-name e))
                                     (failed-role-value (role-value e))
                                     (failed-action-core (action-core e)))
                                (setf (gethash failed-role-name tried-values) (cons failed-role-value (gethash failed-role-name tried-values)))
                                (setf (gethash failed-role-name action-core-arguments)
                                      (query-for-substitute failed-action-core failed-role-name (gethash failed-role-name tried-values)))
                                (cpl-impl:retry))))
                           (apply matching-plan (funcall matching-arg-fn action-core-arguments)))))))
      (progn
        (setf (plan-string plan-string) (format nil "~a~a<NO PLAN FOUND FOR ~a>~%" (plan-string plan-string) tab-string action-core-name))
        (setf (errors error-log) 
              (cons (make-instance 'prac2cram-error
                                   :message (format nil "Couldn't find matching plan and arg-fn for action core ~a." action-core-name))
                    (errors error-log)))
        (make-fn-app (lambda ()))))))

(defun generate-ptr-code-for-action-core (action-core plan-matchings error-log plan-string tab-string)
  (let* ((action-core-name (action-core-name action-core))
         (action-keyword-loop (equal "loop" (string-downcase action-core-name)))
         (action-keyword-if (equal "if" (string-downcase action-core-name))))
    (cond
      (action-keyword-loop
        (generate-ptr-code-loop action-core plan-matchings error-log plan-string tab-string))
      (action-keyword-if
        (generate-ptr-code-if action-core plan-matchings error-log plan-string tab-string))
      (T ;; action-core is not a keyword
        (generate-ptr-code-default action-core plan-matchings error-log plan-string tab-string)))))

(defun generate-ptr-code (tasks plan-matchings error-log plan-string)
  (let* ((fn-app-list (mapcar (lambda (task)
                                (generate-ptr-code-for-action-core task plan-matchings error-log plan-string ""))
                              tasks)))
    fn-app-list))


(cpl:def-top-level-cram-function prac2cram2-toplevel (fn-app-list)
  (let* ((fn-app-list (make-instance 'function-application-list :fn-list fn-app-list)))
    (ptr-seq fn-app-list)))


