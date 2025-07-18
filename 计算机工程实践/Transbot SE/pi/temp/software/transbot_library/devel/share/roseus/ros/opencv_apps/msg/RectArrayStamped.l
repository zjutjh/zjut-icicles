;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::RectArrayStamped)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'RectArrayStamped (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::RECTARRAYSTAMPED")
  (make-package "OPENCV_APPS::RECTARRAYSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude RectArrayStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass opencv_apps::RectArrayStamped
  :super ros::object
  :slots (_header _rects ))

(defmethod opencv_apps::RectArrayStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rects __rects) (let (r) (dotimes (i 0) (push (instance opencv_apps::Rect :init) r)) r))
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
    ;; opencv_apps/Rect[] _rects
    (apply #'+ (send-all _rects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; opencv_apps/Rect[] _rects
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
   ;; opencv_apps/Rect[] _rects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rects (let (r) (dotimes (i n) (push (instance opencv_apps::Rect :init) r)) r))
     (dolist (elem- _rects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::RectArrayStamped :md5sum-) "f29b08b33e061c73d7d9fc35142631d0")
(setf (get opencv_apps::RectArrayStamped :datatype-) "opencv_apps/RectArrayStamped")
(setf (get opencv_apps::RectArrayStamped :definition-)
      "Header header
Rect[] rects



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
MSG: opencv_apps/Rect
# opencv Rect data type, x-y is center point
float64 x
float64 y
float64 width
float64 height


")



(provide :opencv_apps/RectArrayStamped "f29b08b33e061c73d7d9fc35142631d0")


