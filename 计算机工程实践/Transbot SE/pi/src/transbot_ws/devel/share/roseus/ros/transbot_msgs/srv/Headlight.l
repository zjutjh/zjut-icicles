;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::Headlight)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'Headlight (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::HEADLIGHT")
  (make-package "TRANSBOT_MSGS::HEADLIGHT"))
(unless (find-package "TRANSBOT_MSGS::HEADLIGHTREQUEST")
  (make-package "TRANSBOT_MSGS::HEADLIGHTREQUEST"))
(unless (find-package "TRANSBOT_MSGS::HEADLIGHTRESPONSE")
  (make-package "TRANSBOT_MSGS::HEADLIGHTRESPONSE"))

(in-package "ROS")





(defclass transbot_msgs::HeadlightRequest
  :super ros::object
  :slots (_Headlight ))

(defmethod transbot_msgs::HeadlightRequest
  (:init
   (&key
    ((:Headlight __Headlight) 0)
    )
   (send-super :init)
   (setq _Headlight (round __Headlight))
   self)
  (:Headlight
   (&optional __Headlight)
   (if __Headlight (setq _Headlight __Headlight)) _Headlight)
  (:serialization-length
   ()
   (+
    ;; int32 _Headlight
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _Headlight
       (write-long _Headlight s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _Headlight
     (setq _Headlight (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass transbot_msgs::HeadlightResponse
  :super ros::object
  :slots (_result ))

(defmethod transbot_msgs::HeadlightResponse
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

(defclass transbot_msgs::Headlight
  :super ros::object
  :slots ())

(setf (get transbot_msgs::Headlight :md5sum-) "39cb7e9dbd56dfa74d38f52f3463c89d")
(setf (get transbot_msgs::Headlight :datatype-) "transbot_msgs/Headlight")
(setf (get transbot_msgs::Headlight :request) transbot_msgs::HeadlightRequest)
(setf (get transbot_msgs::Headlight :response) transbot_msgs::HeadlightResponse)

(defmethod transbot_msgs::HeadlightRequest
  (:response () (instance transbot_msgs::HeadlightResponse :init)))

(setf (get transbot_msgs::HeadlightRequest :md5sum-) "39cb7e9dbd56dfa74d38f52f3463c89d")
(setf (get transbot_msgs::HeadlightRequest :datatype-) "transbot_msgs/HeadlightRequest")
(setf (get transbot_msgs::HeadlightRequest :definition-)
      "#request
int32 Headlight
---
#response
bool result
")

(setf (get transbot_msgs::HeadlightResponse :md5sum-) "39cb7e9dbd56dfa74d38f52f3463c89d")
(setf (get transbot_msgs::HeadlightResponse :datatype-) "transbot_msgs/HeadlightResponse")
(setf (get transbot_msgs::HeadlightResponse :definition-)
      "#request
int32 Headlight
---
#response
bool result
")



(provide :transbot_msgs/Headlight "39cb7e9dbd56dfa74d38f52f3463c89d")


