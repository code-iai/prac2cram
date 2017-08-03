(defsystem prac2cram
    :depends-on (roslisp roslisp-utilities cram-language alexandria yason std_msgs-msg prac2cram-srv prac2cram-msg rosgraph_msgs-msg)
    :components
    ((:module "src"
              :components
              ((:file "package")
               (:file "conditions" :depends-on ("package"))
               (:file "ptr-language" :depends-on ("package"))
               (:file "parameters" :depends-on ("package"))
               (:file "utils" :depends-on ("package" "parameters"))
               (:file "substitute-query" :depends-on ("package" "parameters" "utils"))
               (:file "code-generation" :depends-on ("package" "parameters" "utils" "substitute-query" "ptr-language" "conditions"))
               (:file "prac2cram-server" :depends-on ("package" "parameters" "utils" "substitute-query"))
               (:file "prac2cram2-server" :depends-on ("package" "parameters" "utils" "substitute-query" "ptr-language" "conditions" "code-generation"))))))

