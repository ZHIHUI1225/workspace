;; Auto-generated. Do not edit!


(when (boundp 'robot_msg::target_pose_array)
  (if (not (find-package "ROBOT_MSG"))
    (make-package "ROBOT_MSG"))
  (shadow 'target_pose_array (find-package "ROBOT_MSG")))
(unless (find-package "ROBOT_MSG::TARGET_POSE_ARRAY")
  (make-package "ROBOT_MSG::TARGET_POSE_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude target_pose_array.msg.html


(defclass robot_msg::target_pose_array
  :super ros::object
  :slots (_target_pose_array ))

(defmethod robot_msg::target_pose_array
  (:init
   (&key
    ((:target_pose_array __target_pose_array) ())
    )
   (send-super :init)
   (setq _target_pose_array __target_pose_array)
   self)
  (:target_pose_array
   (&rest __target_pose_array)
   (if (keywordp (car __target_pose_array))
       (send* _target_pose_array __target_pose_array)
     (progn
       (if __target_pose_array (setq _target_pose_array (car __target_pose_array)))
       _target_pose_array)))
  (:serialization-length
   ()
   (+
    ;; robot_msg/target_pose[] _target_pose_array
    (apply #'+ (send-all _target_pose_array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robot_msg/target_pose[] _target_pose_array
     (write-long (length _target_pose_array) s)
     (dolist (elem _target_pose_array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; robot_msg/target_pose[] _target_pose_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _target_pose_array (let (r) (dotimes (i n) (push (instance robot_msg::target_pose :init) r)) r))
     (dolist (elem- _target_pose_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get robot_msg::target_pose_array :md5sum-) "7368196c844e512e8b512a71481181c8")
(setf (get robot_msg::target_pose_array :datatype-) "robot_msg/target_pose_array")
(setf (get robot_msg::target_pose_array :definition-)
      "robot_msg/target_pose[] target_pose_array

================================================================================
MSG: robot_msg/target_pose
int8 ID
geometry_msgs/Point position

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :robot_msg/target_pose_array "7368196c844e512e8b512a71481181c8")


