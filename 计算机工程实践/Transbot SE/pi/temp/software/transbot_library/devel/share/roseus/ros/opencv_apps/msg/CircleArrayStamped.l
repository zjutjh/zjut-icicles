;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::CircleArrayStamped)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'CircleArrayStamped (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::CIRCLEARRAYSTAMPED")
  (make-package "OPENCV_APPS::CIRCLEARRAYSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude CircleArrayStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass opencv_apps::CircleArrayStamped
  :super ros::object
  :slots (_header _circles ))

(defmethod opencv_apps::CircleArrayStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:circles __circles) (let (r) (dotimes (i 0) (push (instance opencv_apps::Circle :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _circles __circles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:circles
   (&rest __circles)
   (if (keywordp (car __circles))
       (send* _circles __circles)
     (progn
       (if __circles (setq _circles (car __circles)))
       _circles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; opencv_apps/Circle[] _circles
    (apply #'+ (send-all _circles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; opencv_apps/Circle[] _circles
     (write-long (length _circles) s)
     (dolist (elem _circles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; opencv_apps/Circle[] _circles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _circles (let (r) (dotimes (i n) (push (instance opencv_apps::Circle :init) r)) r))
     (dolist (elem- _circles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::CircleArrayStamped :md5sum-) "430ffa6c2b0a36b7e81feff1ce79c3c4")
(setf (get opencv_apps::CircleArrayStamped :datatype-) "opencv_apps/CircleArrayStamped")
(setf (get opencv_apps::CircleArrayStamped :definition-)
      "Header header
Circle[] circles


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: opencv_apps/Circle
Point2D center
float64 radius


================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/CircleArrayStamped "430ffa6c2b0a36b7e81feff1ce79c3c4")


