;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::PWMServo)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'PWMServo (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::PWMSERVO")
  (make-package "TRANSBOT_MSGS::PWMSERVO"))

(in-package "ROS")
;;//! \htmlinclude PWMServo.msg.html


(defclass transbot_msgs::PWMServo
  :super ros::object
  :slots (_id _angle ))

(defmethod transbot_msgs::PWMServo
  (:init
   (&key
    ((:id __id) 0)
    ((:angle __angle) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _angle (round __angle))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; int32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; int32 _angle
       (write-long _angle s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _angle
     (setq _angle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get transbot_msgs::PWMServo :md5sum-) "c5a368d31c65388d88289de5a105a271")
(setf (get transbot_msgs::PWMServo :datatype-) "transbot_msgs/PWMServo")
(setf (get transbot_msgs::PWMServo :definition-)
      "int32 id
int32 angle

")



(provide :transbot_msgs/PWMServo "c5a368d31c65388d88289de5a105a271")


