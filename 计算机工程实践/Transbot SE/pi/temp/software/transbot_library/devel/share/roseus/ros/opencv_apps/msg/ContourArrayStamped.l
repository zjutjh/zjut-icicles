;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::ContourArrayStamped)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'ContourArrayStamped (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::CONTOURARRAYSTAMPED")
  (make-package "OPENCV_APPS::CONTOURARRAYSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude ContourArrayStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass opencv_apps::ContourArrayStamped
  :super ros::object
  :slots (_header _contours ))

(defmethod opencv_apps::ContourArrayStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:contours __contours) (let (r) (dotimes (i 0) (push (instance opencv_apps::Contour :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _contours __contours)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:contours
   (&rest __contours)
   (if (keywordp (car __contours))
       (send* _contours __contours)
     (progn
       (if __contours (setq _contours (car __contours)))
       _contours)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; opencv_apps/Contour[] _contours
    (apply #'+ (send-all _contours :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; opencv_apps/Contour[] _contours
     (write-long (length _contours) s)
     (dolist (elem _contours)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; opencv_apps/Contour[] _contours
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _contours (let (r) (dotimes (i n) (push (instance opencv_apps::Contour :init) r)) r))
     (dolist (elem- _contours)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::ContourArrayStamped :md5sum-) "6bcf2733566be102cf11fc89685fd962")
(setf (get opencv_apps::ContourArrayStamped :datatype-) "opencv_apps/ContourArrayStamped")
(setf (get opencv_apps::ContourArrayStamped :definition-)
      "Header header
Contour[] contours


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
MSG: opencv_apps/Contour
Point2D[] points

================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/ContourArrayStamped "6bcf2733566be102cf11fc89685fd962")


