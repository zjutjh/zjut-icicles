;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::Edition)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'Edition (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::EDITION")
  (make-package "TRANSBOT_MSGS::EDITION"))

(in-package "ROS")
;;//! \htmlinclude Edition.msg.html


(defclass transbot_msgs::Edition
  :super ros::object
  :slots (_edition ))

(defmethod transbot_msgs::Edition
  (:init
   (&key
    ((:edition __edition) 0.0)
    )
   (send-super :init)
   (setq _edition (float __edition))
   self)
  (:edition
   (&optional __edition)
   (if __edition (setq _edition __edition)) _edition)
  (:serialization-length
   ()
   (+
    ;; float32 _edition
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _edition
       (sys::poke _edition (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _edition
     (setq _edition (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get transbot_msgs::Edition :md5sum-) "373df2b35ba40a1a8b8afa0bf078b756")
(setf (get transbot_msgs::Edition :datatype-) "transbot_msgs/Edition")
(setf (get transbot_msgs::Edition :definition-)
      "float32 edition
")



(provide :transbot_msgs/Edition "373df2b35ba40a1a8b8afa0bf078b756")


