;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::LineArrayStamped)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'LineArrayStamped (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::LINEARRAYSTAMPED")
  (make-package "OPENCV_APPS::LINEARRAYSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude LineArrayStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass opencv_apps::LineArrayStamped
  :super ros::object
  :slots (_header _lines ))

(defmethod opencv_apps::LineArrayStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lines __lines) (let (r) (dotimes (i 0) (push (instance opencv_apps::Line :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _lines __lines)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lines
   (&rest __lines)
   (if (keywordp (car __lines))
       (send* _lines __lines)
     (progn
       (if __lines (setq _lines (car __lines)))
       _lines)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; opencv_apps/Line[] _lines
    (apply #'+ (send-all _lines :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; opencv_apps/Line[] _lines
     (write-long (length _lines) s)
     (dolist (elem _lines)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; opencv_apps/Line[] _lines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lines (let (r) (dotimes (i n) (push (instance opencv_apps::Line :init) r)) r))
     (dolist (elem- _lines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::LineArrayStamped :md5sum-) "8ad5d104256b4f6774479453d1118f74")
(setf (get opencv_apps::LineArrayStamped :datatype-) "opencv_apps/LineArrayStamped")
(setf (get opencv_apps::LineArrayStamped :definition-)
      "Header header
Line[] lines

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
MSG: opencv_apps/Line
Point2D pt1
Point2D pt2


================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/LineArrayStamped "8ad5d104256b4f6774479453d1118f74")


