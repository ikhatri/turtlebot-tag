
(cl:in-package :asdf)

(defsystem "compsci403_assignment4-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObstacleMsg" :depends-on ("_package_ObstacleMsg"))
    (:file "_package_ObstacleMsg" :depends-on ("_package"))
  ))