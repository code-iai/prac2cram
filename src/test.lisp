;; Parameters/global vars

(defparameter *current-color* "clear")
(defparameter *actual-contents* "magnesium")
(defparameter *concentration* 5)

(defparameter test-error-log (make-instance 'prac2cram::prac2cram-error-log))
(defparameter test-plan-string (make-instance 'prac2cram::prac2cram-plan-string))

;; "Identification of metal cations"
;;
;; Pour 5 drops of sodium hydroxide in the test container.
;; If a brown precipitate appears in the test container, the
;; robot says "the solution contains iron".
;; If a white precipitate appears in the test container, pour
;; 5ml of sodium hydroxide in the test container.
;; If the test container becomes clear, the robot says "the
;; solution contains aluminium".
;; If the precipitate in the test container remains, the robot
;; says "the solution contains magnesium".

(defun update-color (concentrated-naoh)
  (cond
    ((equal *actual-contents* "iron")
       (format t "The solution turns brown.~%")
       (setf *current-color* "brown"))
    ((not concentrated-naoh)
       (format t "The solution turns white.~%")
       (setf *current-color* "white"))
    ((and concentrated-naoh (equal *actual-contents* "aluminium"))
       (format t "The solution turns clear.~%")
       (setf *current-color* "clear"))
    ((and concentrated-naoh (equal *actual-contents* "magnesium"))
       (format t "The solution turns white.~%")
       (setf *current-color* "white"))
    (T
      (format t "The lab disappears in a puff of logic.")
      nil)))

(defun test-pipette (content goal amount unit)
  (format T "ROBOT PIPETTES ~a~a of ~a into ~a.~%" amount unit content goal)
  (update-color nil))

(defun test-pipette-argfn (arg-hash)
  (let* ((content (gethash "content" arg-hash))
         (goal (gethash "goal" arg-hash))
         (amount (gethash "amount" arg-hash))
         (unit (gethash "unit" arg-hash)))
    (list content goal amount unit)))

(defun test-pour (content goal amount unit)
  (format T "ROBOT POURS ~a~a of ~a into ~a.~%" amount unit content goal)
  (update-color T))

(defun test-pour-argfn (arg-hash)
  (let* ((content (gethash "content" arg-hash))
         (goal (gethash "goal" arg-hash))
         (amount (gethash "amount" arg-hash))
         (unit (gethash "unit" arg-hash)))
    (list content goal amount unit)))

(defun test-say (message)
  (format t "ROBOT SAYS: ~a.~%" message))

(defun test-say-argfn (arg-hash)
  (let* ((message (gethash "message" arg-hash)))
    (list message)))

(defun test-expect-color (color)
  (roslisp:wait-duration 1.0)
  (format t "Actual color ~a Expected ~a~%" *current-color* color)
  (roslisp:wait-duration 1.0)
  (if (equal color *current-color*)
    (format t "Observed color ~a.~%" color)
    (loop (roslisp:wait-duration 1.0))))
;;;    (signal "Didn't observe expected color.")))

(defun test-expect-color-argfn (arg-hash)
  (let* ((color (gethash "color" arg-hash)))
    (list color)))

(defparameter pour-NaOH-drops
              (alexandria:alist-hash-table (list (cons "name" "use-pipette") 
                                                 (cons "roles" (alexandria:alist-hash-table '(("amount" . "5")
                                                                                              ("unit" . "drops")
                                                                                              ("content" . "NatriumHydroxide")
                                                                                              ("goal" . "UnknownSubstance")) :test #'equal)))
                                           :test #'equal))

(defparameter pour-NaOH-mls
              (alexandria:alist-hash-table (list (cons "name" "use-measuring-cup") 
                                                 (cons "roles" (alexandria:alist-hash-table '(("amount" . "5")
                                                                                              ("unit" . "ml")
                                                                                              ("content" . "NatriumHydroxide")
                                                                                              ("goal" . "UnknownSubstance")) :test #'equal)))
                                           :test #'equal))

(defparameter say-iron
              (alexandria:alist-hash-table (list (cons "name" "say")
                                                 (cons "roles" (alexandria:alist-hash-table '(("message" . "the substance contains Iron (Fe3+)")) :test #'equal)))
                                           :test #'equal))

(defparameter say-aluminium
              (alexandria:alist-hash-table (list (cons "name" "say")
                                                 (cons "roles" (alexandria:alist-hash-table '(("message" . "the substance contains Aluminium (Al3+)")) :test #'equal)))
                                           :test #'equal))

(defparameter say-magnesium
              (alexandria:alist-hash-table (list (cons "name" "say")
                                                 (cons "roles" (alexandria:alist-hash-table '(("message" . "the substance contains Magnesium (Mg2+)")) :test #'equal)))
                                           :test #'equal))

(defparameter check-brown
              (alexandria:alist-hash-table (list (cons "name" "expect-color")
                                                 (cons "roles" (alexandria:alist-hash-table '(("color" . "brown")) :test #'equal)))
                                           :test #'equal))

(defparameter check-white
              (alexandria:alist-hash-table (list (cons "name" "expect-color")
                                                 (cons "roles" (alexandria:alist-hash-table '(("color" . "white")) :test #'equal)))
                                           :test #'equal))

(defparameter check-clear
              (alexandria:alist-hash-table (list (cons "name" "expect-color")
                                                 (cons "roles" (alexandria:alist-hash-table '(("color" . "clear")) :test #'equal)))
                                           :test #'equal))

(defparameter iron-branch 
              (alexandria:alist-hash-table (list (cons "condition" (list check-brown))
                                                 (cons "body" (list (list say-iron))))
                                           :test #'equal))
              
(defparameter aluminium-branch 
              (alexandria:alist-hash-table (list (cons "condition" (list check-clear))
                                                 (cons "body" (list (list say-aluminium))))
                                           :test #'equal))
              
(defparameter magnesium-branch 
              (alexandria:alist-hash-table (list (cons "condition" (list check-white))
                                                 (cons "body" (list (list say-magnesium))))
                                           :test #'equal))
              
(defparameter if-on-concentrated-NaOH
              (alexandria:alist-hash-table (list (cons "name" "if") 
                                                 (cons "roles" (alexandria:alist-hash-table 
                                                                 (list (cons "branches" 
                                                                             (list aluminium-branch magnesium-branch))) :test #'equal)))
                                           :test #'equal))

(defparameter aluminium-magnesium-branch 
              (alexandria:alist-hash-table (list (cons "condition" (list check-white))
                                                 (cons "body" (list (list pour-NaOH-mls)
                                                                    (list if-on-concentrated-NaOH))))
                                           :test #'equal))
              
(defparameter if-on-diluted-NaOH
              (alexandria:alist-hash-table (list (cons "name" "if") 
                                                 (cons "roles" (alexandria:alist-hash-table 
                                                                 (list (cons "branches" 
                                                                             (list iron-branch aluminium-magnesium-branch))) :test #'equal)))
                                           :test #'equal))

(defparameter identify-metal-cations
              (list (list pour-NaOH-drops)
                    (list if-on-diluted-NaOH)))

;; "Titration"
;;
;; Pour a drop of NaOH into the test solution until the 
;; test solution is clear.

(defun update-concentration ()
  (setf *concentration* (- *concentration* 1))
  (if (< *concentration* 0)
    (setf *concentration* 0))
  (if (equal *concentration* 0)
    (setf *current-color* "clear")))

(defun pour-drop (what where)
  (roslisp:wait-duration 4.0)
  (format t "Pouring one drop of ~a into ~a ...~%" what where)
  (update-concentration))

(defun pour-drop-argfn (arg-hash)
  (let* ((content (gethash "content" arg-hash))
         (goal (gethash "goal" arg-hash)))
    (list content goal)))

(defun test-check-color (color)
  (format t "Actual color ~a Expected ~a~%" *current-color* color)
  (loop while (not (equal color *current-color*)) do
    (progn
      (format t "Actual color ~a Expected ~a~%" *current-color* color)
      (roslisp:wait-duration 1.0)))
  (format t "Actual color ~a Expected ~a~%" *current-color* color))

(defparameter titration-check-clear
              (alexandria:alist-hash-table (list (cons "name" "wait-color")
                                                 (cons "roles" (alexandria:alist-hash-table '(("color" . "clear")) :test #'equal)))
                                           :test #'equal))

(defparameter pour-drop-action
              (alexandria:alist-hash-table (list (cons "name" "pour-drop") 
                                                 (cons "roles" (alexandria:alist-hash-table '(("content" . "NatriumHydroxide")
                                                                                              ("goal" . "TestSolution")) :test #'equal)))
                                           :test #'equal))

(defparameter titration-loop
              (alexandria:alist-hash-table (list (cons "name" "loop") 
                                                 (cons "roles" (alexandria:alist-hash-table `(("type" . "loop-until")
                                                                                              ("condition" . ,(list titration-check-clear))
                                                                                              ("action" . ,(list pour-drop-action))) :test #'equal)))
                                           :test #'equal))

(defparameter titration
  (list (list titration-loop)))

;; Plan matchings

(defparameter test-plan-matchings
              (list (list "use-pipette" #'test-pipette #'test-pipette-argfn "pipette")
                    (list "use-measuring-cup" #'test-pour #'test-pour-argfn "pour")
                    (list "say" #'test-say #'test-say-argfn "say")
                    (list "wait-color" #'test-check-color #'test-expect-color-argfn "wait-color")
                    (list "pour-drop" #'pour-drop #'pour-drop-argfn "pour-drop")
                    (list "expect-color" #'test-expect-color #'test-expect-color-argfn "expect-color")))

