; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment4-srv)


;//! \htmlinclude GetCommandVelSrv-request.msg.html

(cl:defclass <GetCommandVelSrv-request> (roslisp-msg-protocol:ros-message)
  ((v_0
    :reader v_0
    :initarg :v_0
    :type cl:float
    :initform 0.0)
   (w_0
    :reader w_0
    :initarg :w_0
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetCommandVelSrv-request (<GetCommandVelSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCommandVelSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCommandVelSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment4-srv:<GetCommandVelSrv-request> is deprecated: use compsci403_assignment4-srv:GetCommandVelSrv-request instead.")))

(cl:ensure-generic-function 'v_0-val :lambda-list '(m))
(cl:defmethod v_0-val ((m <GetCommandVelSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:v_0-val is deprecated.  Use compsci403_assignment4-srv:v_0 instead.")
  (v_0 m))

(cl:ensure-generic-function 'w_0-val :lambda-list '(m))
(cl:defmethod w_0-val ((m <GetCommandVelSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:w_0-val is deprecated.  Use compsci403_assignment4-srv:w_0 instead.")
  (w_0 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCommandVelSrv-request>) ostream)
  "Serializes a message object of type '<GetCommandVelSrv-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v_0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'w_0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCommandVelSrv-request>) istream)
  "Deserializes a message object of type '<GetCommandVelSrv-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w_0) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCommandVelSrv-request>)))
  "Returns string type for a service object of type '<GetCommandVelSrv-request>"
  "compsci403_assignment4/GetCommandVelSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCommandVelSrv-request)))
  "Returns string type for a service object of type 'GetCommandVelSrv-request"
  "compsci403_assignment4/GetCommandVelSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCommandVelSrv-request>)))
  "Returns md5sum for a message object of type '<GetCommandVelSrv-request>"
  "73fef1a96aacf31f2c19a0e67105926e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCommandVelSrv-request)))
  "Returns md5sum for a message object of type 'GetCommandVelSrv-request"
  "73fef1a96aacf31f2c19a0e67105926e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCommandVelSrv-request>)))
  "Returns full string definition for message of type '<GetCommandVelSrv-request>"
  (cl:format cl:nil "float32 v_0~%float32 w_0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCommandVelSrv-request)))
  "Returns full string definition for message of type 'GetCommandVelSrv-request"
  (cl:format cl:nil "float32 v_0~%float32 w_0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCommandVelSrv-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCommandVelSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCommandVelSrv-request
    (cl:cons ':v_0 (v_0 msg))
    (cl:cons ':w_0 (w_0 msg))
))
;//! \htmlinclude GetCommandVelSrv-response.msg.html

(cl:defclass <GetCommandVelSrv-response> (roslisp-msg-protocol:ros-message)
  ((C_v
    :reader C_v
    :initarg :C_v
    :type cl:float
    :initform 0.0)
   (C_w
    :reader C_w
    :initarg :C_w
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetCommandVelSrv-response (<GetCommandVelSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCommandVelSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCommandVelSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment4-srv:<GetCommandVelSrv-response> is deprecated: use compsci403_assignment4-srv:GetCommandVelSrv-response instead.")))

(cl:ensure-generic-function 'C_v-val :lambda-list '(m))
(cl:defmethod C_v-val ((m <GetCommandVelSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:C_v-val is deprecated.  Use compsci403_assignment4-srv:C_v instead.")
  (C_v m))

(cl:ensure-generic-function 'C_w-val :lambda-list '(m))
(cl:defmethod C_w-val ((m <GetCommandVelSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:C_w-val is deprecated.  Use compsci403_assignment4-srv:C_w instead.")
  (C_w m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCommandVelSrv-response>) ostream)
  "Serializes a message object of type '<GetCommandVelSrv-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'C_v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'C_w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCommandVelSrv-response>) istream)
  "Deserializes a message object of type '<GetCommandVelSrv-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'C_v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'C_w) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCommandVelSrv-response>)))
  "Returns string type for a service object of type '<GetCommandVelSrv-response>"
  "compsci403_assignment4/GetCommandVelSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCommandVelSrv-response)))
  "Returns string type for a service object of type 'GetCommandVelSrv-response"
  "compsci403_assignment4/GetCommandVelSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCommandVelSrv-response>)))
  "Returns md5sum for a message object of type '<GetCommandVelSrv-response>"
  "73fef1a96aacf31f2c19a0e67105926e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCommandVelSrv-response)))
  "Returns md5sum for a message object of type 'GetCommandVelSrv-response"
  "73fef1a96aacf31f2c19a0e67105926e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCommandVelSrv-response>)))
  "Returns full string definition for message of type '<GetCommandVelSrv-response>"
  (cl:format cl:nil "float32 C_v~%float32 C_w~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCommandVelSrv-response)))
  "Returns full string definition for message of type 'GetCommandVelSrv-response"
  (cl:format cl:nil "float32 C_v~%float32 C_w~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCommandVelSrv-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCommandVelSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCommandVelSrv-response
    (cl:cons ':C_v (C_v msg))
    (cl:cons ':C_w (C_w msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCommandVelSrv)))
  'GetCommandVelSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCommandVelSrv)))
  'GetCommandVelSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCommandVelSrv)))
  "Returns string type for a service object of type '<GetCommandVelSrv>"
  "compsci403_assignment4/GetCommandVelSrv")