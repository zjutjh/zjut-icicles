;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::RobotArm)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'RobotArm (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::ROBOTARM")
  (make-package "TRANSBOT_MSGS::ROBOTARM"))
(unless (find-package "TRANSBOT_MSGS::ROBOTARMREQUEST")
  (make-package "TRANSBOT_MSGS::ROBOTARMREQUEST"))
(unless (find-package "TRANSBOT_MSGS::ROBOTARMRESPONSE")
  (make-package "TRANSBOT_MSGS::ROBOTARMRESPONSE"))

(in-package "ROS")





(defclass transbot_msgs::RobotArmRequest
  :super ros::object
  :slots (_apply ))

(defmethod transbot_msgs::RobotArmRequest
  (:init
   (&key
    ((:apply __apply) "")
    )
   (send-super :init)
   (setq _apply (string __apply))
   self)
  (:apply
   (&optional __apply)
   (if __apply (setq _apply __apply)) _apply)
  (:serialization-length
   ()
   (+
    ;; string _apply
    4 (length _apply)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _apply
       (write-long (length _apply) s) (princ _apply s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _apply
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _apply (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass transbot_msgs::RobotArmResponse
  :super ros::object
  :slots (_RobotArm ))

(defmethod transbot_msgs::RobotArmResponse
  (:init
   (&key
    ((:RobotArm __RobotArm) (instance transbot_msgs::Arm :init))
    )
   (send-super :init)
   (setq _RobotArm __RobotArm)
   self)
  (:RobotArm
   (&rest __RobotArm)
   (if (keywordp (car __RobotArm))
       (send* _RobotArm __RobotArm)
     (progn
       (if __RobotArm (setq _RobotArm (car __RobotArm)))
       _RobotArm)))
  (:serialization-length
   ()
   (+
    ;; transbot_msgs/Arm _RobotArm
    (send _RobotArm :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; transbot_msgs/Arm _RobotArm
       (send _RobotArm :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; transbot_msgs/Arm _RobotArm
     (send _RobotArm :deserialize buf ptr-) (incf ptr- (send _RobotArm :serialization-length))
   ;;
   self)
  )

(defclass transbot_msgs::RobotArm
  :super ros::object
  :slots ())

(setf (get transbot_msgs::RobotArm :md5sum-) "02b16e175f9698037e15289959eba75b")
(setf (get transbot_msgs::RobotArm :datatype-) "transbot_msgs/RobotArm")
(setf (get transbot_msgs::RobotArm :request) transbot_msgs::RobotArmRequest)
(setf (get transbot_msgs::RobotArm :response) transbot_msgs::RobotArmResponse)

(defmethod transbot_msgs::RobotArmRequest
  (:response () (instance transbot_msgs::RobotArmResponse :init)))

(setf (get transbot_msgs::RobotArmRequest :md5sum-) "02b16e175f9698037e15289959eba75b")
(setf (get transbot_msgs::RobotArmRequest :datatype-) "transbot_msgs/RobotArmRequest")
(setf (get transbot_msgs::RobotArmRequest :definition-)
      "#request
string apply
---
#response
Arm RobotArm


================================================================================
MSG: transbot_msgs/Arm
Joint[] joint

================================================================================
MSG: transbot_msgs/Joint
int32 id
int32 run_time
float32 angle
")

(setf (get transbot_msgs::RobotArmResponse :md5sum-) "02b16e175f9698037e15289959eba75b")
(setf (get transbot_msgs::RobotArmResponse :datatype-) "transbot_msgs/RobotArmResponse")
(setf (get transbot_msgs::RobotArmResponse :definition-)
      "#request
string apply
---
#response
Arm RobotArm


================================================================================
MSG: transbot_msgs/Arm
Joint[] joint

================================================================================
MSG: transbot_msgs/Joint
int32 id
int32 run_time
float32 angle
")



(provide :transbot_msgs/RobotArm "02b16e175f9698037e15289959eba75b")


