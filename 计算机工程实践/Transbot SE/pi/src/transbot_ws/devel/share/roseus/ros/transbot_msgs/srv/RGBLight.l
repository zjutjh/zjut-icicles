;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::RGBLight)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'RGBLight (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::RGBLIGHT")
  (make-package "TRANSBOT_MSGS::RGBLIGHT"))
(unless (find-package "TRANSBOT_MSGS::RGBLIGHTREQUEST")
  (make-package "TRANSBOT_MSGS::RGBLIGHTREQUEST"))
(unless (find-package "TRANSBOT_MSGS::RGBLIGHTRESPONSE")
  (make-package "TRANSBOT_MSGS::RGBLIGHTRESPONSE"))

(in-package "ROS")





(defclass transbot_msgs::RGBLightRequest
  :super ros::object
  :slots (_effect _speed ))

(defmethod transbot_msgs::RGBLightRequest
  (:init
   (&key
    ((:effect __effect) 0)
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _effect (round __effect))
   (setq _speed (round __speed))
   self)
  (:effect
   (&optional __effect)
   (if __effect (setq _effect __effect)) _effect)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int32 _effect
    4
    ;; int32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _effect
       (write-long _effect s)
     ;; int32 _speed
       (write-long _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _effect
     (setq _effect (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass transbot_msgs::RGBLightResponse
  :super ros::object
  :slots (_result ))

(defmethod transbot_msgs::RGBLightResponse
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

(defclass transbot_msgs::RGBLight
  :super ros::object
  :slots ())

(setf (get transbot_msgs::RGBLight :md5sum-) "4abffe91f7c3570fa771519fedadcf37")
(setf (get transbot_msgs::RGBLight :datatype-) "transbot_msgs/RGBLight")
(setf (get transbot_msgs::RGBLight :request) transbot_msgs::RGBLightRequest)
(setf (get transbot_msgs::RGBLight :response) transbot_msgs::RGBLightResponse)

(defmethod transbot_msgs::RGBLightRequest
  (:response () (instance transbot_msgs::RGBLightResponse :init)))

(setf (get transbot_msgs::RGBLightRequest :md5sum-) "4abffe91f7c3570fa771519fedadcf37")
(setf (get transbot_msgs::RGBLightRequest :datatype-) "transbot_msgs/RGBLightRequest")
(setf (get transbot_msgs::RGBLightRequest :definition-)
      "#request
int32 effect
int32 speed
---
#response
bool result

")

(setf (get transbot_msgs::RGBLightResponse :md5sum-) "4abffe91f7c3570fa771519fedadcf37")
(setf (get transbot_msgs::RGBLightResponse :datatype-) "transbot_msgs/RGBLightResponse")
(setf (get transbot_msgs::RGBLightResponse :definition-)
      "#request
int32 effect
int32 speed
---
#response
bool result

")



(provide :transbot_msgs/RGBLight "4abffe91f7c3570fa771519fedadcf37")


