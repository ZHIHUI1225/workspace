;; Auto-generated. Do not edit!


(when (boundp 'robot_msg::robot_pose)
  (if (not (find-package "ROBOT_MSG"))
    (make-package "ROBOT_MSG"))
  (shadow 'robot_pose (find-package "ROBOT_MSG")))
(unless (find-package "ROBOT_MSG::ROBOT_POSE")
  (make-package "ROBOT_MSG::ROBOT_POSE"))

(in-package "ROS")
;;//! \htmlinclude robot_pose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robot_msg::robot_pose
  :super ros::object
  :slots (_ID _position _yaw ))

(defmethod robot_msg::robot_pose
  (:init
   (&key
    ((:ID __ID) (instance std_msgs::Int8 :init))
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _ID __ID)
   (setq _position __position)
   (setq _yaw (float __yaw))
   self)
  (:ID
   (&rest __ID)
   (if (keywordp (car __ID))
       (send* _ID __ID)
     (progn
       (if __ID (setq _ID (car __ID)))
       _ID)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Int8 _ID
    (send _ID :serialization-length)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; float32 _yaw
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Int8 _ID
       (send _ID :serialize s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Int8 _ID
     (send _ID :deserialize buf ptr-) (incf ptr- (send _ID :serialization-length))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_msg::robot_pose :md5sum-) "c2b2afa426c38cad763bf7a208a1669f")
(setf (get robot_msg::robot_pose :datatype-) "robot_msg/robot_pose")
(setf (get robot_msg::robot_pose :definition-)
      "std_msgs/Int8 ID
geometry_msgs/Point position
float32 yaw

================================================================================
MSG: std_msgs/Int8
int8 data

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :robot_msg/robot_pose "c2b2afa426c38cad763bf7a208a1669f")


