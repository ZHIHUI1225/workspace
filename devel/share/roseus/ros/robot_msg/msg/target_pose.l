;; Auto-generated. Do not edit!


(when (boundp 'robot_msg::target_pose)
  (if (not (find-package "ROBOT_MSG"))
    (make-package "ROBOT_MSG"))
  (shadow 'target_pose (find-package "ROBOT_MSG")))
(unless (find-package "ROBOT_MSG::TARGET_POSE")
  (make-package "ROBOT_MSG::TARGET_POSE"))

(in-package "ROS")
;;//! \htmlinclude target_pose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass robot_msg::target_pose
  :super ros::object
  :slots (_ID _position ))

(defmethod robot_msg::target_pose
  (:init
   (&key
    ((:ID __ID) 0)
    ((:position __position) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _ID (round __ID))
   (setq _position __position)
   self)
  (:ID
   (&optional __ID)
   (if __ID (setq _ID __ID)) _ID)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:serialization-length
   ()
   (+
    ;; int8 _ID
    1
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _ID
       (write-byte _ID s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _ID
     (setq _ID (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ID 127) (setq _ID (- _ID 256)))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;;
   self)
  )

(setf (get robot_msg::target_pose :md5sum-) "90b8db49b6b04a35daa272b200b90cf3")
(setf (get robot_msg::target_pose :datatype-) "robot_msg/target_pose")
(setf (get robot_msg::target_pose :definition-)
      "int8 ID
geometry_msgs/Point position

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :robot_msg/target_pose "90b8db49b6b04a35daa272b200b90cf3")


