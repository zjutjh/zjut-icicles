;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::Patrol)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'Patrol (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::PATROL")
  (make-package "TRANSBOT_MSGS::PATROL"))
(unless (find-package "TRANSBOT_MSGS::PATROLREQUEST")
  (make-package "TRANSBOT_MSGS::PATROLREQUEST"))
(unless (find-package "TRANSBOT_MSGS::PATROLRESPONSE")
  (make-package "TRANSBOT_MSGS::PATROLRESPONSE"))

(in-package "ROS")





(defclass transbot_msgs::PatrolRequest
  :super ros::object
  :slots (_Commond _LineScaling _RotationScaling ))

(defmethod transbot_msgs::PatrolRequest
  (:init
   (&key
    ((:Commond __Commond) "")
    ((:LineScaling __LineScaling) 0.0)
    ((:RotationScaling __RotationScaling) 0.0)
    )
   (send-super :init)
   (setq _Commond (string __Commond))
   (setq _LineScaling (float __LineScaling))
   (setq _RotationScaling (float __RotationScaling))
   self)
  (:Commond
   (&optional __Commond)
   (if __Commond (setq _Commond __Commond)) _Commond)
  (:LineScaling
   (&optional __LineScaling)
   (if __LineScaling (setq _LineScaling __LineScaling)) _LineScaling)
  (:RotationScaling
   (&optional __RotationScaling)
   (if __RotationScaling (setq _RotationScaling __RotationScaling)) _RotationScaling)
  (:serialization-length
   ()
   (+
    ;; string _Commond
    4 (length _Commond)
    ;; float32 _LineScaling
    4
    ;; float32 _RotationScaling
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Commond
       (write-long (length _Commond) s) (princ _Commond s)
     ;; float32 _LineScaling
       (sys::poke _LineScaling (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _RotationScaling
       (sys::poke _RotationScaling (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Commond
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Commond (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _LineScaling
     (setq _LineScaling (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _RotationScaling
     (setq _RotationScaling (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass transbot_msgs::PatrolResponse
  :super ros::object
  :slots (_result ))

(defmethod transbot_msgs::PatrolResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass transbot_msgs::Patrol
  :super ros::object
  :slots ())

(setf (get transbot_msgs::Patrol :md5sum-) "725a414bc8766f0cf2b2c0b5f17047e6")
(setf (get transbot_msgs::Patrol :datatype-) "transbot_msgs/Patrol")
(setf (get transbot_msgs::Patrol :request) transbot_msgs::PatrolRequest)
(setf (get transbot_msgs::Patrol :response) transbot_msgs::PatrolResponse)

(defmethod transbot_msgs::PatrolRequest
  (:response () (instance transbot_msgs::PatrolResponse :init)))

(setf (get transbot_msgs::PatrolRequest :md5sum-) "725a414bc8766f0cf2b2c0b5f17047e6")
(setf (get transbot_msgs::PatrolRequest :datatype-) "transbot_msgs/PatrolRequest")
(setf (get transbot_msgs::PatrolRequest :definition-)
      "#request
string  Commond
float32 LineScaling
float32 RotationScaling
---
#response
bool result
")

(setf (get transbot_msgs::PatrolResponse :md5sum-) "725a414bc8766f0cf2b2c0b5f17047e6")
(setf (get transbot_msgs::PatrolResponse :datatype-) "transbot_msgs/PatrolResponse")
(setf (get transbot_msgs::PatrolResponse :definition-)
      "#request
string  Commond
float32 LineScaling
float32 RotationScaling
---
#response
bool result
")



(provide :transbot_msgs/Patrol "725a414bc8766f0cf2b2c0b5f17047e6")


