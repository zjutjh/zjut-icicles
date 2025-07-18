;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::LaserAvoid)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'LaserAvoid (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::LASERAVOID")
  (make-package "TRANSBOT_MSGS::LASERAVOID"))

(in-package "ROS")
;;//! \htmlinclude LaserAvoid.msg.html


(defclass transbot_msgs::LaserAvoid
  :super ros::object
  :slots (_Angle_range _ResponseDist _speed ))

(defmethod transbot_msgs::LaserAvoid
  (:init
   (&key
    ((:Angle_range __Angle_range) 0)
    ((:ResponseDist __ResponseDist) 0.0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _Angle_range (round __Angle_range))
   (setq _ResponseDist (float __ResponseDist))
   (setq _speed (float __speed))
   self)
  (:Angle_range
   (&optional __Angle_range)
   (if __Angle_range (setq _Angle_range __Angle_range)) _Angle_range)
  (:ResponseDist
   (&optional __ResponseDist)
   (if __ResponseDist (setq _ResponseDist __ResponseDist)) _ResponseDist)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int32 _Angle_range
    4
    ;; float32 _ResponseDist
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _Angle_range
       (write-long _Angle_range s)
     ;; float32 _ResponseDist
       (sys::poke _ResponseDist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _Angle_range
     (setq _Angle_range (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _ResponseDist
     (setq _ResponseDist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get transbot_msgs::LaserAvoid :md5sum-) "6872d2ec650238739f9b3c8aab8ed9b1")
(setf (get transbot_msgs::LaserAvoid :datatype-) "transbot_msgs/LaserAvoid")
(setf (get transbot_msgs::LaserAvoid :definition-)
      "int32 Angle_range
float32 ResponseDist
float32 speed

")



(provide :transbot_msgs/LaserAvoid "6872d2ec650238739f9b3c8aab8ed9b1")


