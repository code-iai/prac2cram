(in-package :prac2cram)

;;; TODO: make a plan with declare-goal and  def-goal for every action core that could arrive  and call the plan from the server and call it with a designator containing the action roles 

(def-service-callback Prac2Cram (action_cores)
  (ros-info (basics-system) "Printing parameters: ~a " action_cores)
  (make-response :status 0 :plan "Test String"))

;;; TODO Fehler abfangen, data structure erklären 

(defun prac2cram-server () 
  (with-ros-node ("prac2cram_server" :spin t)
     (register-service "prac2cram" 'Prac2Cram)
     (ros-info (basics-system) "This is the prac2cram server. I'm ready to start a CRAM query.")))
