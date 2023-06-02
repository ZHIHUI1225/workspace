; Auto-generated. Do not edit!


(cl:in-package robot_msg-msg)


;//! \htmlinclude robot_pose.msg.html

(cl:defclass <robot_pose> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass robot_pose (<robot_pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msg-msg:<robot_pose> is deprecated: use robot_msg-msg:robot_pose instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <robot_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-msg:ID-val is deprecated.  Use robot_msg-msg:ID instead.")
  (ID m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <robot_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-msg:position-val is deprecated.  Use robot_msg-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <robot_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-msg:yaw-val is deprecated.  Use robot_msg-msg:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_pose>) ostream)
  "Serializes a message object of type '<robot_pose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ID) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_pose>) istream)
  "Deserializes a message object of type '<robot_pose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ID) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_pose>)))
  "Returns string type for a message object of type '<robot_pose>"
  "robot_msg/robot_pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_pose)))
  "Returns string type for a message object of type 'robot_pose"
  "robot_msg/robot_pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_pose>)))
  "Returns md5sum for a message object of type '<robot_pose>"
  "c2b2afa426c38cad763bf7a208a1669f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_pose)))
  "Returns md5sum for a message object of type 'robot_pose"
  "c2b2afa426c38cad763bf7a208a1669f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_pose>)))
  "Returns full string definition for message of type '<robot_pose>"
  (cl:format cl:nil "std_msgs/Int8 ID~%geometry_msgs/Point position~%float32 yaw~%~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_pose)))
  "Returns full string definition for message of type 'robot_pose"
  (cl:format cl:nil "std_msgs/Int8 ID~%geometry_msgs/Point position~%float32 yaw~%~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_pose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ID))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_pose>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_pose
    (cl:cons ':ID (ID msg))
    (cl:cons ':position (position msg))
    (cl:cons ':yaw (yaw msg))
))
