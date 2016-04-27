(defsystem prac2cram
    :depends-on (roslisp cram-language designators std_msgs-msg prac2cram-srv)
    :components
    ((:module "src"
              :components
              ((:file "package")
               (:file "prac2cram-server" :depends-on ("package"))))))

