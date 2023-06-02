;; Auto-generated. Do not edit!


(when (boundp 'robot_msg::robot_pose_array)
  (if (not (find-package "ROBOT_MSG"))
    (make-package "ROBOT_MSG"))
  (shadow 'robot_pose_array (find-package "ROBOT_MSG")))
(unless (find-package "ROBOT_MSG::ROBOT_POSE_ARRAY")
  (make-package "ROBOT_MSG::ROBOT_POSE_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude robot_pose_array.msg.html


(defclass robot_msg::robot_pose_array
  :super ros::object
  :slots (_robot_pose_array ))

(defmethod robot_msg::robot_pose_array
  (:init
   (&key
    ((:robot_pose_array __robot_pose_array) ())
    )
   (send-super :init)
   (setq _robot_pose_array __robot_pose_array)
   self)
  (:robot_pose_array
   (&rest __robot_pose_array)
   (if (keywordp (car __robot_pose_array))
       (send* _robot_pose_array __robot_pose_array)
     (progn
       (if __robot_pose_array (setq _robot_pose_array (car __robot_pose_array)))
       _robot_pose_array)))
  (:serialization-length
   ()
   (+
    ;; robot_msg/robot_pose[] _robot_pose_array
    (apply #'+ (send-all _robot_pose_array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robot_msg/robot_pose[] _robot_pose_array
     (write-long (length _robot_pose_array) s)
     (dolist (elem _robot_pose_array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; robot_msg/robot_pose[] _robot_pose_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robot_pose_array (let (r) (dotimes (i n) (push (instance robot_msg::robot_pose :init) r)) r))
     (dolist (elem- _robot_pose_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get robot_msg::robot_pose_array :md5sum-) "b5025b9975c3a323219809cc70bf1c63")
(setf (get robot_msg::robot_pose_array :datatype-) "robot_msg/robot_pose_array")
(setf (get robot_msg::robot_pose_array :definition-)
      "robot_msg/robot_pose[] robot_pose_array

================================================================================
MSG: robot_msg/robot_pose
std_msgs/Int8 ID
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



(provide :robot_msg/robot_pose_array "b5025b9975c3a323219809cc70bf1c63")


