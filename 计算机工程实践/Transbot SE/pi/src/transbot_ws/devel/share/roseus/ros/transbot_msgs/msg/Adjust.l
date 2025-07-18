;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::Adjust)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'Adjust (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::ADJUST")
  (make-package "TRANSBOT_MSGS::ADJUST"))

(in-package "ROS")
;;//! \htmlinclude Adjust.msg.html


(defclass transbot_msgs::Adjust
  :super ros::object
  :slots (_adjust ))

(defmethod transbot_msgs::Adjust
  (:init
   (&key
    ((:adjust __adjust) nil)
    )
   (send-super :init)
   (setq _adjust __adjust)
   self)
  (:adjust
   (&optional __adjust)
   (if __adjust (setq _adjust __adjust)) _adjust)
  (:serialization-length
   ()
   (+
    ;; bool _adjust
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _adjust
       (if _adjust (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _adjust
     (setq _adjust (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get transbot_msgs::Adjust :md5sum-) "686be2de32be2d650746cf5e906439fb")
(setf (get transbot_msgs::Adjust :datatype-) "transbot_msgs/Adjust")
(setf (get transbot_msgs::Adjust :definition-)
      "bool adjust
")



(provide :transbot_msgs/Adjust "686be2de32be2d650746cf5e906439fb")


