
(cl:in-package :asdf)

(defsystem "compsci403_assignment4-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetFreePathSrv" :depends-on ("_package_GetFreePathSrv"))
    (:file "_package_GetFreePathSrv" :depends-on ("_package"))
    (:file "GetCommandVelSrv" :depends-on ("_package_GetCommandVelSrv"))
    (:file "_package_GetCommandVelSrv" :depends-on ("_package"))
    (:file "CheckPointSrv" :depends-on ("_package_CheckPointSrv"))
    (:file "_package_CheckPointSrv" :depends-on ("_package"))
  ))