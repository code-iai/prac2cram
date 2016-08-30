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

(roslisp:def-service-callback Prac2Cram2 (tasks)
  (let* ((tasks-str tasks)
         (tasks (json-decode tasks)))
    (roslisp:ros-info (basics-system) "Received service call with these parameters: Tasks: ~a" tasks-str)
    (if cpl:*task-tree*   ;; test if another cram plan is running; *task-tree* will be non-NIL if so
      (roslisp:make-response :status -1
                             :individual_status (vector)
                             :messages (vector "Another cram plan is running. Wait for it to finish and send the request again.")
                             :plan_strings (vector ))
      (let* ((error-log (make-instance 'prac2cram-error-log))
             (plan-string (make-instance 'prac2cram-plan-string))
             (fn-app-list (generate-ptr-code tasks *plan-matchings* error-log plan-string))
             (plan-started (equal (errors error-log) nil)))
        (if plan-started
          (progn
            (sb-thread:make-thread (lambda ()
                                     (prac2cram2-toplevel fn-app-list)))
            (roslisp:make-response :status 0
                                   :plan_string (plan-string plan-string)))
          (roslisp:make-response :status -1
                                 :plan_string (plan-string plan-string)
                                 :errors (coerce (errors error-log) 'vector)))))))


(defun prac2cram2-server (plan-matchings &optional (prac-url "http://localhost:1234"))
  (let* ((plan-matchings plan-matchings))
    (setf *prac-url* prac-url)
    (if (not (roslisp:wait-for-service "/prac2cram_http_bridge" 0.1))
      (sb-thread:make-thread (lambda ()
                               (sb-ext:run-program "rosrun" (list "prac2cram" "prac2cram_HTTP_bridge")))))
    (setf *plan-matchings* plan-matchings)
    (roslisp:register-service "prac2cram2" 
                              'Prac2Cram2)
    (roslisp:ros-info (basics-system) "This is the prac2cram2 server. I'm ready to start a CRAM query. Known plan matchings: ~a. Also recognize keywords: (if).~%" *plan-matchings*)))
