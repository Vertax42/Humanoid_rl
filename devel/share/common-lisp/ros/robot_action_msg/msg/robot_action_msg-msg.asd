
(cl:in-package :asdf)

(defsystem "robot_action_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "robot_acts" :depends-on ("_package_robot_acts"))
    (:file "_package_robot_acts" :depends-on ("_package"))
  ))