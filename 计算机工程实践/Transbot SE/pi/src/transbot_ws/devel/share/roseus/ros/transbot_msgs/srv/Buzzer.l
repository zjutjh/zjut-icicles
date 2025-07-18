;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::Buzzer)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'Buzzer (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::BUZZER")
  (make-package "TRANSBOT_MSGS::BUZZER"))
(unless (find-package "TRANSBOT_MSGS::BUZZERREQUEST")
  (make-package "TRANSBOT_MSGS::BUZZERREQUEST"))
(unless (find-package "TRANSBOT_MSGS::BUZZERRESPONSE")
  (make-package "TRANSBOT_MSGS::BUZZERRESPONSE"))

(in-package "ROS")





(defclass transbot_msgs::BuzzerRequest
  :super ros::object
  :slots (_buzzer ))

(defmethod transbot_msgs::BuzzerRequest
  (:init
   (&key
    ((:buzzer __buzzer) 0)
    )
   (send-super :init)
   (setq _buzzer (round __buzzer))
   self)
  (:buzzer
   (&optional __buzzer)
   (if __buzzer (setq _buzzer __buzzer)) _buzzer)
  (:serialization-length
   ()
   (+
    ;; int32 _buzzer
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _buzzer
       (write-long _buzzer s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _buzzer
     (setq _buzzer (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass transbot_msgs::BuzzerResponse
  :super ros::object
  :slots (_result ))

(defmethod transbot_msgs::BuzzerResponse
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

(defclass transbot_msgs::Buzzer
  :super ros::object
  :slots ())

(setf (get transbot_msgs::Buzzer :md5sum-) "32ecc8168750cdefd185aff218d2ce5e")
(setf (get transbot_msgs::Buzzer :datatype-) "transbot_msgs/Buzzer")
(setf (get transbot_msgs::Buzzer :request) transbot_msgs::BuzzerRequest)
(setf (get transbot_msgs::Buzzer :response) transbot_msgs::BuzzerResponse)

(defmethod transbot_msgs::BuzzerRequest
  (:response () (instance transbot_msgs::BuzzerResponse :init)))

(setf (get transbot_msgs::BuzzerRequest :md5sum-) "32ecc8168750cdefd185aff218d2ce5e")
(setf (get transbot_msgs::BuzzerRequest :datatype-) "transbot_msgs/BuzzerRequest")
(setf (get transbot_msgs::BuzzerRequest :definition-)
      "#request
int32 buzzer
---
#response
bool result
")

(setf (get transbot_msgs::BuzzerResponse :md5sum-) "32ecc8168750cdefd185aff218d2ce5e")
(setf (get transbot_msgs::BuzzerResponse :datatype-) "transbot_msgs/BuzzerResponse")
(setf (get transbot_msgs::BuzzerResponse :definition-)
      "#request
int32 buzzer
---
#response
bool result
")



(provide :transbot_msgs/Buzzer "32ecc8168750cdefd185aff218d2ce5e")


