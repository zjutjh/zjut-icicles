;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::PatrolWarning)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'PatrolWarning (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::PATROLWARNING")
  (make-package "TRANSBOT_MSGS::PATROLWARNING"))

(in-package "ROS")
;;//! \htmlinclude PatrolWarning.msg.html


(defclass transbot_msgs::PatrolWarning
  :super ros::object
  :slots (_speed _Function _ResponseDist _LaserAngle ))

(defmethod transbot_msgs::PatrolWarning
  (:init
   (&key
    ((:speed __speed) 0.0)
    ((:Function __Function) "")
    ((:ResponseDist __ResponseDist) 0.0)
    ((:LaserAngle __LaserAngle) 0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   (setq _Function (string __Function))
   (setq _ResponseDist (float __ResponseDist))
   (setq _LaserAngle (round __LaserAngle))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:Function
   (&optional __Function)
   (if __Function (setq _Function __Function)) _Function)
  (:ResponseDist
   (&optional __ResponseDist)
   (if __ResponseDist (setq _ResponseDist __ResponseDist)) _ResponseDist)
  (:LaserAngle
   (&optional __LaserAngle)
   (if __LaserAngle (setq _LaserAngle __LaserAngle)) _LaserAngle)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ;; string _Function
    4 (length _Function)
    ;; float32 _ResponseDist
    4
    ;; int32 _LaserAngle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _Function
       (write-long (length _Function) s) (princ _Function s)
     ;; float32 _ResponseDist
       (sys::poke _ResponseDist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _LaserAngle
       (write-long _LaserAngle s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _Function
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Function (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _ResponseDist
     (setq _ResponseDist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _LaserAngle
     (setq _LaserAngle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get transbot_msgs::PatrolWarning :md5sum-) "3c2aedf4e9d9a1d5ce206d948829c9bc")
(setf (get transbot_msgs::PatrolWarning :datatype-) "transbot_msgs/PatrolWarning")
(setf (get transbot_msgs::PatrolWarning :definition-)
      "float32 speed
string Function
float32 ResponseDist
int32   LaserAngle
")



(provide :transbot_msgs/PatrolWarning "3c2aedf4e9d9a1d5ce206d948829c9bc")


