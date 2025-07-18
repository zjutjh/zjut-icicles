;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::Battery)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'Battery (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::BATTERY")
  (make-package "TRANSBOT_MSGS::BATTERY"))

(in-package "ROS")
;;//! \htmlinclude Battery.msg.html


(defclass transbot_msgs::Battery
  :super ros::object
  :slots (_Voltage ))

(defmethod transbot_msgs::Battery
  (:init
   (&key
    ((:Voltage __Voltage) 0.0)
    )
   (send-super :init)
   (setq _Voltage (float __Voltage))
   self)
  (:Voltage
   (&optional __Voltage)
   (if __Voltage (setq _Voltage __Voltage)) _Voltage)
  (:serialization-length
   ()
   (+
    ;; float32 _Voltage
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _Voltage
       (sys::poke _Voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _Voltage
     (setq _Voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get transbot_msgs::Battery :md5sum-) "cb42914b7d362060569576ceed0643ce")
(setf (get transbot_msgs::Battery :datatype-) "transbot_msgs/Battery")
(setf (get transbot_msgs::Battery :definition-)
      "float32 Voltage
")



(provide :transbot_msgs/Battery "cb42914b7d362060569576ceed0643ce")


