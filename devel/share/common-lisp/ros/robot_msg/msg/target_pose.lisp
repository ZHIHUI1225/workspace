; Auto-generated. Do not edit!


(cl:in-package robot_msg-msg)


;//! \htmlinclude target_pose.msg.html

(cl:defclass <target_pose> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass target_pose (<target_pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <target_pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'target_pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msg-msg:<target_pose> is deprecated: use robot_msg-msg:target_pose instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <target_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-msg:ID-val is deprecated.  Use robot_msg-msg:ID instead.")
  (ID m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <target_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-msg:position-val is deprecated.  Use robot_msg-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <target_pose>) ostream)
  "Serializes a message object of type '<target_pose>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <target_pose>) istream)
  "Deserializes a message object of type '<target_pose>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<target_pose>)))
  "Returns string type for a message object of type '<target_pose>"
  "robot_msg/target_pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'target_pose)))
  "Returns string type for a message object of type 'target_pose"
  "robot_msg/target_pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<target_pose>)))
  "Returns md5sum for a message object of type '<target_pose>"
  "90b8db49b6b04a35daa272b200b90cf3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'target_pose)))
  "Returns md5sum for a message object of type 'target_pose"
  "90b8db49b6b04a35daa272b200b90cf3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<target_pose>)))
  "Returns full string definition for message of type '<target_pose>"
  (cl:format cl:nil "int8 ID~%geometry_msgs/Point position~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'target_pose)))
  "Returns full string definition for message of type 'target_pose"
  (cl:format cl:nil "int8 ID~%geometry_msgs/Point position~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <target_pose>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <target_pose>))
  "Converts a ROS message object to a list"
  (cl:list 'target_pose
    (cl:cons ':ID (ID msg))
    (cl:cons ':position (position msg))
))
