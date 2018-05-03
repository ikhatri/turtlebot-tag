
(cl:in-package :asdf)

(defsystem "cobot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CobotDriveMsg" :depends-on ("_package_CobotDriveMsg"))
    (:file "_package_CobotDriveMsg" :depends-on ("_package"))
  ))