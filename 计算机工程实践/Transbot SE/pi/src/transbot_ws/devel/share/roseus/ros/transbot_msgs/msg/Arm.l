;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::Arm)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'Arm (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::ARM")
  (make-package "TRANSBOT_MSGS::ARM"))

(in-package "ROS")
;;//! \htmlinclude Arm.msg.html


(defclass transbot_msgs::Arm
  :super ros::object
  :slots (_joint ))

(defmethod transbot_msgs::Arm
  (:init
   (&key
    ((:joint __joint) (let (r) (dotimes (i 0) (push (instance transbot_msgs::Joint :init) r)) r))
    )
   (send-super :init)
   (setq _joint __joint)
   self)
  (:joint
   (&rest __joint)
   (if (keywordp (car __joint))
       (send* _joint __joint)
     (progn
       (if __joint (setq _joint (car __joint)))
       _joint)))
  (:serialization-length
   ()
   (+
    ;; transbot_msgs/Joint[] _joint
    (apply #'+ (send-all _joint :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; transbot_msgs/Joint[] _joint
     (write-long (length _joint) s)
     (dolist (elem _joint)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; transbot_msgs/Joint[] _joint
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint (let (r) (dotimes (i n) (push (instance transbot_msgs::Joint :init) r)) r))
     (dolist (elem- _joint)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get transbot_msgs::Arm :md5sum-) "b46d7c4342769b3898ee5c56a7392dd2")
(setf (get transbot_msgs::Arm :datatype-) "transbot_msgs/Arm")
(setf (get transbot_msgs::Arm :definition-)
      "Joint[] joint

================================================================================
MSG: transbot_msgs/Joint
int32 id
int32 run_time
float32 angle

")



(provide :transbot_msgs/Arm "b46d7c4342769b3898ee5c56a7392dd2")


