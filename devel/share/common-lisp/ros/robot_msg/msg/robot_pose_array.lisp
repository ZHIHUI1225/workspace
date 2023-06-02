; Auto-generated. Do not edit!


(cl:in-package robot_msg-msg)


;//! \htmlinclude robot_pose_array.msg.html

(cl:defclass <robot_pose_array> (roslisp-msg-protocol:ros-message)
  ((robot_pose_array
    :reader robot_pose_array
    :initarg :robot_pose_array
    :type (cl:vector robot_msg-msg:robot_pose)
   :initform (cl:make-array 0 :element-type 'robot_msg-msg:robot_pose :initial-element (cl:make-instance 'robot_msg-msg:robot_pose))))
)

(cl:defclass robot_pose_array (<robot_pose_array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_pose_array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_pose_array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msg-msg:<robot_pose_array> is deprecated: use robot_msg-msg:robot_pose_array instead.")))

(cl:ensure-generic-function 'robot_pose_array-val :lambda-list '(m))
(cl:defmethod robot_pose_array-val ((m <robot_pose_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-msg:robot_pose_array-val is deprecated.  Use robot_msg-msg:robot_pose_array instead.")
  (robot_pose_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_pose_array>) ostream)
  "Serializes a message object of type '<robot_pose_array>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'robot_pose_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'robot_pose_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_pose_array>) istream)
  "Deserializes a message object of type '<robot_pose_array>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'robot_pose_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'robot_pose_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'robot_msg-msg:robot_pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_pose_array>)))
  "Returns string type for a message object of type '<robot_pose_array>"
  "robot_msg/robot_pose_array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_pose_array)))
  "Returns string type for a message object of type 'robot_pose_array"
  "robot_msg/robot_pose_array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_pose_array>)))
  "Returns md5sum for a message object of type '<robot_pose_array>"
  "b5025b9975c3a323219809cc70bf1c63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_pose_array)))
  "Returns md5sum for a message object of type 'robot_pose_array"
  "b5025b9975c3a323219809cc70bf1c63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_pose_array>)))
  "Returns full string definition for message of type '<robot_pose_array>"
  (cl:format cl:nil "robot_msg/robot_pose[] robot_pose_array~%~%================================================================================~%MSG: robot_msg/robot_pose~%std_msgs/Int8 ID~%geometry_msgs/Point position~%float32 yaw~%~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_pose_array)))
  "Returns full string definition for message of type 'robot_pose_array"
  (cl:format cl:nil "robot_msg/robot_pose[] robot_pose_array~%~%================================================================================~%MSG: robot_msg/robot_pose~%std_msgs/Int8 ID~%geometry_msgs/Point position~%float32 yaw~%~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_pose_array>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_pose_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_pose_array>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_pose_array
    (cl:cons ':robot_pose_array (robot_pose_array msg))
))
