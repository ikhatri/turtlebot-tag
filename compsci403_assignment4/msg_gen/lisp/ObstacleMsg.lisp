; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment4-msg)


;//! \htmlinclude ObstacleMsg.msg.html

(cl:defclass <ObstacleMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacle_points
    :reader obstacle_points
    :initarg :obstacle_points
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32))))
)

(cl:defclass ObstacleMsg (<ObstacleMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment4-msg:<ObstacleMsg> is deprecated: use compsci403_assignment4-msg:ObstacleMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObstacleMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-msg:header-val is deprecated.  Use compsci403_assignment4-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacle_points-val :lambda-list '(m))
(cl:defmethod obstacle_points-val ((m <ObstacleMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment4-msg:obstacle_points-val is deprecated.  Use compsci403_assignment4-msg:obstacle_points instead.")
  (obstacle_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleMsg>) ostream)
  "Serializes a message object of type '<ObstacleMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacle_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacle_points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleMsg>) istream)
  "Deserializes a message object of type '<ObstacleMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacle_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacle_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleMsg>)))
  "Returns string type for a message object of type '<ObstacleMsg>"
  "compsci403_assignment4/ObstacleMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleMsg)))
  "Returns string type for a message object of type 'ObstacleMsg"
  "compsci403_assignment4/ObstacleMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleMsg>)))
  "Returns md5sum for a message object of type '<ObstacleMsg>"
  "a29f7b4409b527b2f8154194df18d3b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleMsg)))
  "Returns md5sum for a message object of type 'ObstacleMsg"
  "a29f7b4409b527b2f8154194df18d3b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleMsg>)))
  "Returns full string definition for message of type '<ObstacleMsg>"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Point32[] obstacle_points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleMsg)))
  "Returns full string definition for message of type 'ObstacleMsg"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Point32[] obstacle_points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacle_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleMsg
    (cl:cons ':header (header msg))
    (cl:cons ':obstacle_points (obstacle_points msg))
))
