
(cl:in-package :asdf)

(defsystem "robot_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "robot_pose" :depends-on ("_package_robot_pose"))
    (:file "_package_robot_pose" :depends-on ("_package"))
    (:file "robot_pose_array" :depends-on ("_package_robot_pose_array"))
    (:file "_package_robot_pose_array" :depends-on ("_package"))
    (:file "target_pose" :depends-on ("_package_target_pose"))
    (:file "_package_target_pose" :depends-on ("_package"))
    (:file "target_pose_array" :depends-on ("_package_target_pose_array"))
    (:file "_package_target_pose_array" :depends-on ("_package"))
  ))