; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment4-srv)


;//! \htmlinclude CheckPointSrv-request.msg.html

(cl:defclass <CheckPointSrv-request> (roslisp-msg-protocol:ros-message)
  ((P
    :reader P
    :initarg :P
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0)
   (w
    :reader w
    :initarg :w
    :type cl:float
    :initform 0.0))
)

(cl:defclass CheckPointSrv-request (<CheckPointSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPointSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPointSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment4-srv:<CheckPointSrv-request> is deprecated: use compsci403_assignment4-srv:CheckPointSrv-request instead.")))

(cl:ensure-generic-function 'P-val :lambda-list '(m))
(cl:defmethod P-val ((m <CheckPointSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:P-val is deprecated.  Use compsci403_assignment4-srv:P instead.")
  (P m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <CheckPointSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:v-val is deprecated.  Use compsci403_assignment4-srv:v instead.")
  (v m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <CheckPointSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:w-val is deprecated.  Use compsci403_assignment4-srv:w instead.")
  (w m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPointSrv-request>) ostream)
  "Serializes a message object of type '<CheckPointSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'P) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPointSrv-request>) istream)
  "Deserializes a message object of type '<CheckPointSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'P) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPointSrv-request>)))
  "Returns string type for a service object of type '<CheckPointSrv-request>"
  "compsci403_assignment4/CheckPointSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPointSrv-request)))
  "Returns string type for a service object of type 'CheckPointSrv-request"
  "compsci403_assignment4/CheckPointSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPointSrv-request>)))
  "Returns md5sum for a message object of type '<CheckPointSrv-request>"
  "942f1fcf986c084288bf027e6e35f386")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPointSrv-request)))
  "Returns md5sum for a message object of type 'CheckPointSrv-request"
  "942f1fcf986c084288bf027e6e35f386")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPointSrv-request>)))
  "Returns full string definition for message of type '<CheckPointSrv-request>"
  (cl:format cl:nil "geometry_msgs/Point32 P~%float32 v~%float32 w~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPointSrv-request)))
  "Returns full string definition for message of type 'CheckPointSrv-request"
  (cl:format cl:nil "geometry_msgs/Point32 P~%float32 v~%float32 w~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPointSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'P))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPointSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPointSrv-request
    (cl:cons ':P (P msg))
    (cl:cons ':v (v msg))
    (cl:cons ':w (w msg))
))
;//! \htmlinclude CheckPointSrv-response.msg.html

(cl:defclass <CheckPointSrv-response> (roslisp-msg-protocol:ros-message)
  ((is_obstacle
    :reader is_obstacle
    :initarg :is_obstacle
    :type cl:boolean
    :initform cl:nil)
   (free_path_length
    :reader free_path_length
    :initarg :free_path_length
    :type cl:float
    :initform 0.0))
)

(cl:defclass CheckPointSrv-response (<CheckPointSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPointSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPointSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment4-srv:<CheckPointSrv-response> is deprecated: use compsci403_assignment4-srv:CheckPointSrv-response instead.")))

(cl:ensure-generic-function 'is_obstacle-val :lambda-list '(m))
(cl:defmethod is_obstacle-val ((m <CheckPointSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:is_obstacle-val is deprecated.  Use compsci403_assignment4-srv:is_obstacle instead.")
  (is_obstacle m))

(cl:ensure-generic-function 'free_path_length-val :lambda-list '(m))
(cl:defmethod free_path_length-val ((m <CheckPointSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:free_path_length-val is deprecated.  Use compsci403_assignment4-srv:free_path_length instead.")
  (free_path_length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPointSrv-response>) ostream)
  "Serializes a message object of type '<CheckPointSrv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_obstacle) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'free_path_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPointSrv-response>) istream)
  "Deserializes a message object of type '<CheckPointSrv-response>"
    (cl:setf (cl:slot-value msg 'is_obstacle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'free_path_length) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPointSrv-response>)))
  "Returns string type for a service object of type '<CheckPointSrv-response>"
  "compsci403_assignment4/CheckPointSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPointSrv-response)))
  "Returns string type for a service object of type 'CheckPointSrv-response"
  "compsci403_assignment4/CheckPointSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPointSrv-response>)))
  "Returns md5sum for a message object of type '<CheckPointSrv-response>"
  "942f1fcf986c084288bf027e6e35f386")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPointSrv-response)))
  "Returns md5sum for a message object of type 'CheckPointSrv-response"
  "942f1fcf986c084288bf027e6e35f386")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPointSrv-response>)))
  "Returns full string definition for message of type '<CheckPointSrv-response>"
  (cl:format cl:nil "bool is_obstacle~%float32 free_path_length~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPointSrv-response)))
  "Returns full string definition for message of type 'CheckPointSrv-response"
  (cl:format cl:nil "bool is_obstacle~%float32 free_path_length~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPointSrv-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPointSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPointSrv-response
    (cl:cons ':is_obstacle (is_obstacle msg))
    (cl:cons ':free_path_length (free_path_length msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckPointSrv)))
  'CheckPointSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckPointSrv)))
  'CheckPointSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPointSrv)))
  "Returns string type for a service object of type '<CheckPointSrv>"
  "compsci403_assignment4/CheckPointSrv")