
(cl:in-package :asdf)

(defsystem "transbot_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :transbot_msgs-msg
)
  :components ((:file "_package")
    (:file "Buzzer" :depends-on ("_package_Buzzer"))
    (:file "_package_Buzzer" :depends-on ("_package"))
    (:file "CamDevice" :depends-on ("_package_CamDevice"))
    (:file "_package_CamDevice" :depends-on ("_package"))
    (:file "Headlight" :depends-on ("_package_Headlight"))
    (:file "_package_Headlight" :depends-on ("_package"))
    (:file "Patrol" :depends-on ("_package_Patrol"))
    (:file "_package_Patrol" :depends-on ("_package"))
    (:file "RGBLight" :depends-on ("_package_RGBLight"))
    (:file "_package_RGBLight" :depends-on ("_package"))
    (:file "RobotArm" :depends-on ("_package_RobotArm"))
    (:file "_package_RobotArm" :depends-on ("_package"))
  ))