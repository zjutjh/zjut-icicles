;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::RotatedRectArrayStamped)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'RotatedRectArrayStamped (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::ROTATEDRECTARRAYSTAMPED")
  (make-package "OPENCV_APPS::ROTATEDRECTARRAYSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude RotatedRectArrayStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass opencv_apps::RotatedRectArrayStamped
  :super ros::object
  :slots (_header _rects ))

(defmethod opencv_apps::RotatedRectArrayStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rects __rects) (let (r) (dotimes (i 0) (push (instance opencv_apps::RotatedRect :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _rects __rects)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rects
   (&rest __rects)
   (if (keywordp (car __rects))
       (send* _rects __rects)
     (progn
       (if __rects (setq _rects (car __rects)))
       _rects)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; opencv_apps/RotatedRect[] _rects
    (apply #'+ (send-all _rects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; opencv_apps/RotatedRect[] _rects
     (write-long (length _rects) s)
     (dolist (elem _rects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; opencv_apps/RotatedRect[] _rects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rects (let (r) (dotimes (i n) (push (instance opencv_apps::RotatedRect :init) r)) r))
     (dolist (elem- _rects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::RotatedRectArrayStamped :md5sum-) "89a2d4a7db2d2945ca46c25a3bd8c7c5")
(setf (get opencv_apps::RotatedRectArrayStamped :datatype-) "opencv_apps/RotatedRectArrayStamped")
(setf (get opencv_apps::RotatedRectArrayStamped :definition-)
      "Header header
RotatedRect[] rects


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



(provide :opencv_apps/RotatedRectArrayStamped "89a2d4a7db2d2945ca46c25a3bd8c7c5")


