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

(defun query-for-substitute (failed-action-core failed-role-name tried-values)
  (let* ((url (format nil "~a/find_substitute_for_role/" (string-right-trim "/" *prac-url*)))
         (param-names (list "action-core" "role" "previous-values"))
         (param-values (list failed-action-core failed-role-name (json-encode tried-values))))
    (roslisp:with-fields (result)
                         (roslisp:call-service "/prac2cram_http_bridge"
                                               :url url
                                               :param_names param-names
                                               :param_values param-values)
      (json-decode result))))

;;(defun query-for-substitutes (action-core action-role rejected-values suggested-values)
;;  (let* ((url (format nil "~a/find_substitute_for_role/" (string-right-trim "/" *prac-url*)))
;;         (param-names (list "action-core" "role" "previous-values" "suggested-values"))
;;         (param-values (list action-core action-role (format nil "~a" rejected-values) (format nil "~a" suggested-values))))
;;    (roslisp:call-service "/prac2cram_http_bridge"
;;                          :url url
;;                          :param_names param-names
;;                          :param_values param-values)))


