; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment4-srv)


;//! \htmlinclude GetFreePathSrv-request.msg.html

(cl:defclass <GetFreePathSrv-request> (roslisp-msg-protocol:ros-message)
  ((laser_scan
    :reader laser_scan
    :initarg :laser_scan
    :type sensor_msgs-msg:LaserScan
    :initform (cl:make-instance 'sensor_msgs-msg:LaserScan))
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

(cl:defclass GetFreePathSrv-request (<GetFreePathSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetFreePathSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetFreePathSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment4-srv:<GetFreePathSrv-request> is deprecated: use compsci403_assignment4-srv:GetFreePathSrv-request instead.")))

(cl:ensure-generic-function 'laser_scan-val :lambda-list '(m))
(cl:defmethod laser_scan-val ((m <GetFreePathSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:laser_scan-val is deprecated.  Use compsci403_assignment4-srv:laser_scan instead.")
  (laser_scan m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <GetFreePathSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:v-val is deprecated.  Use compsci403_assignment4-srv:v instead.")
  (v m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <GetFreePathSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:w-val is deprecated.  Use compsci403_assignment4-srv:w instead.")
  (w m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetFreePathSrv-request>) ostream)
  "Serializes a message object of type '<GetFreePathSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'laser_scan) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetFreePathSrv-request>) istream)
  "Deserializes a message object of type '<GetFreePathSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'laser_scan) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetFreePathSrv-request>)))
  "Returns string type for a service object of type '<GetFreePathSrv-request>"
  "compsci403_assignment4/GetFreePathSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFreePathSrv-request)))
  "Returns string type for a service object of type 'GetFreePathSrv-request"
  "compsci403_assignment4/GetFreePathSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetFreePathSrv-request>)))
  "Returns md5sum for a message object of type '<GetFreePathSrv-request>"
  "4dbf2c0b49713b0f6d232f2c3eb14392")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetFreePathSrv-request)))
  "Returns md5sum for a message object of type 'GetFreePathSrv-request"
  "4dbf2c0b49713b0f6d232f2c3eb14392")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetFreePathSrv-request>)))
  "Returns full string definition for message of type '<GetFreePathSrv-request>"
  (cl:format cl:nil "sensor_msgs/LaserScan laser_scan~%float32 v~%float32 w~%~%================================================================================~%MSG: sensor_msgs/LaserScan~%# Single scan from a planar laser range-finder~%#~%# If you have another ranging device with different behavior (e.g. a sonar~%# array), please find or create a different message, since applications~%# will make fairly laser-specific assumptions about this data~%~%Header header            # timestamp in the header is the acquisition time of ~%                         # the first ray in the scan.~%                         #~%                         # in frame frame_id, angles are measured around ~%                         # the positive Z axis (counterclockwise, if Z is up)~%                         # with zero angle being forward along the x axis~%                         ~%float32 angle_min        # start angle of the scan [rad]~%float32 angle_max        # end angle of the scan [rad]~%float32 angle_increment  # angular distance between measurements [rad]~%~%float32 time_increment   # time between measurements [seconds] - if your scanner~%                         # is moving, this will be used in interpolating position~%                         # of 3d points~%float32 scan_time        # time between scans [seconds]~%~%float32 range_min        # minimum range value [m]~%float32 range_max        # maximum range value [m]~%~%float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)~%float32[] intensities    # intensity data [device-specific units].  If your~%                         # device does not provide intensities, please leave~%                         # the array empty.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetFreePathSrv-request)))
  "Returns full string definition for message of type 'GetFreePathSrv-request"
  (cl:format cl:nil "sensor_msgs/LaserScan laser_scan~%float32 v~%float32 w~%~%================================================================================~%MSG: sensor_msgs/LaserScan~%# Single scan from a planar laser range-finder~%#~%# If you have another ranging device with different behavior (e.g. a sonar~%# array), please find or create a different message, since applications~%# will make fairly laser-specific assumptions about this data~%~%Header header            # timestamp in the header is the acquisition time of ~%                         # the first ray in the scan.~%                         #~%                         # in frame frame_id, angles are measured around ~%                         # the positive Z axis (counterclockwise, if Z is up)~%                         # with zero angle being forward along the x axis~%                         ~%float32 angle_min        # start angle of the scan [rad]~%float32 angle_max        # end angle of the scan [rad]~%float32 angle_increment  # angular distance between measurements [rad]~%~%float32 time_increment   # time between measurements [seconds] - if your scanner~%                         # is moving, this will be used in interpolating position~%                         # of 3d points~%float32 scan_time        # time between scans [seconds]~%~%float32 range_min        # minimum range value [m]~%float32 range_max        # maximum range value [m]~%~%float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)~%float32[] intensities    # intensity data [device-specific units].  If your~%                         # device does not provide intensities, please leave~%                         # the array empty.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetFreePathSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'laser_scan))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetFreePathSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetFreePathSrv-request
    (cl:cons ':laser_scan (laser_scan msg))
    (cl:cons ':v (v msg))
    (cl:cons ':w (w msg))
))
;//! \htmlinclude GetFreePathSrv-response.msg.html

(cl:defclass <GetFreePathSrv-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetFreePathSrv-response (<GetFreePathSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetFreePathSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetFreePathSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment4-srv:<GetFreePathSrv-response> is deprecated: use compsci403_assignment4-srv:GetFreePathSrv-response instead.")))

(cl:ensure-generic-function 'is_obstacle-val :lambda-list '(m))
(cl:defmethod is_obstacle-val ((m <GetFreePathSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:is_obstacle-val is deprecated.  Use compsci403_assignment4-srv:is_obstacle instead.")
  (is_obstacle m))

(cl:ensure-generic-function 'free_path_length-val :lambda-list '(m))
(cl:defmethod free_path_length-val ((m <GetFreePathSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-srv:free_path_length-val is deprecated.  Use compsci403_assignment4-srv:free_path_length instead.")
  (free_path_length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetFreePathSrv-response>) ostream)
  "Serializes a message object of type '<GetFreePathSrv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_obstacle) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'free_path_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetFreePathSrv-response>) istream)
  "Deserializes a message object of type '<GetFreePathSrv-response>"
    (cl:setf (cl:slot-value msg 'is_obstacle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'free_path_length) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetFreePathSrv-response>)))
  "Returns string type for a service object of type '<GetFreePathSrv-response>"
  "compsci403_assignment4/GetFreePathSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFreePathSrv-response)))
  "Returns string type for a service object of type 'GetFreePathSrv-response"
  "compsci403_assignment4/GetFreePathSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetFreePathSrv-response>)))
  "Returns md5sum for a message object of type '<GetFreePathSrv-response>"
  "4dbf2c0b49713b0f6d232f2c3eb14392")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetFreePathSrv-response)))
  "Returns md5sum for a message object of type 'GetFreePathSrv-response"
  "4dbf2c0b49713b0f6d232f2c3eb14392")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetFreePathSrv-response>)))
  "Returns full string definition for message of type '<GetFreePathSrv-response>"
  (cl:format cl:nil "bool is_obstacle~%float32 free_path_length~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetFreePathSrv-response)))
  "Returns full string definition for message of type 'GetFreePathSrv-response"
  (cl:format cl:nil "bool is_obstacle~%float32 free_path_length~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetFreePathSrv-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetFreePathSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetFreePathSrv-response
    (cl:cons ':is_obstacle (is_obstacle msg))
    (cl:cons ':free_path_length (free_path_length msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetFreePathSrv)))
  'GetFreePathSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetFreePathSrv)))
  'GetFreePathSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFreePathSrv)))
  "Returns string type for a service object of type '<GetFreePathSrv>"
  "compsci403_assignment4/GetFreePathSrv")