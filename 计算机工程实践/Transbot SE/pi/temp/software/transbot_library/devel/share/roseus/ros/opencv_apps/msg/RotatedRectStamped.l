;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::RotatedRectStamped)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'RotatedRectStamped (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::ROTATEDRECTSTAMPED")
  (make-package "OPENCV_APPS::ROTATEDRECTSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude RotatedRectStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass opencv_apps::RotatedRectStamped
  :super ros::object
  :slots (_header _rect ))

(defmethod opencv_apps::RotatedRectStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rect __rect) (instance opencv_apps::RotatedRect :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _rect __rect)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rect
   (&rest __rect)
   (if (keywordp (car __rect))
       (send* _rect __rect)
     (progn
       (if __rect (setq _rect (car __rect)))
       _rect)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; opencv_apps/RotatedRect _rect
    (send _rect :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; opencv_apps/RotatedRect _rect
       (send _rect :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; opencv_apps/RotatedRect _rect
     (send _rect :deserialize buf ptr-) (incf ptr- (send _rect :serialization-length))
   ;;
   self)
  )

(setf (get opencv_apps::RotatedRectStamped :md5sum-) "ba2d76a1968e4f77570c01223781fe15")
(setf (get opencv_apps::RotatedRectStamped :datatype-) "opencv_apps/RotatedRectStamped")
(setf (get opencv_apps::RotatedRectStamped :definition-)
      "Header header
RotatedRect rect



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
MSG: opencv_apps/RotatedRect
float64 angle
Point2D center
Size size

================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


================================================================================
MSG: opencv_apps/Size
float64 width
float64 height


")



(provide :opencv_apps/RotatedRectStamped "ba2d76a1968e4f77570c01223781fe15")


