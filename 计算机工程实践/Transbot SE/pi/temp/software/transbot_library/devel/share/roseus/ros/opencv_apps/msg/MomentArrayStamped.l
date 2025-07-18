;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::MomentArrayStamped)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'MomentArrayStamped (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::MOMENTARRAYSTAMPED")
  (make-package "OPENCV_APPS::MOMENTARRAYSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude MomentArrayStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass opencv_apps::MomentArrayStamped
  :super ros::object
  :slots (_header _moments ))

(defmethod opencv_apps::MomentArrayStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:moments __moments) (let (r) (dotimes (i 0) (push (instance opencv_apps::Moment :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _moments __moments)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:moments
   (&rest __moments)
   (if (keywordp (car __moments))
       (send* _moments __moments)
     (progn
       (if __moments (setq _moments (car __moments)))
       _moments)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; opencv_apps/Moment[] _moments
    (apply #'+ (send-all _moments :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; opencv_apps/Moment[] _moments
     (write-long (length _moments) s)
     (dolist (elem _moments)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; opencv_apps/Moment[] _moments
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _moments (let (r) (dotimes (i n) (push (instance opencv_apps::Moment :init) r)) r))
     (dolist (elem- _moments)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::MomentArrayStamped :md5sum-) "28ac0beb70b037acf76c3bed71b679a9")
(setf (get opencv_apps::MomentArrayStamped :datatype-) "opencv_apps/MomentArrayStamped")
(setf (get opencv_apps::MomentArrayStamped :definition-)
      "Header header
Moment[] moments

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
MSG: opencv_apps/Moment
# spatial moments
float64 m00
float64 m10
float64 m01
float64 m20
float64 m11
float64 m02
float64 m30
float64 m21
float64 m12
float64 m03

# central moments
float64 mu20
float64 mu11
float64 mu02
float64 mu30
float64 mu21
float64 mu12
float64 mu03

# central normalized moments
float64 nu20
float64 nu11
float64 nu02
float64 nu30
float64 nu21
float64 nu12
float64 nu03

# center of mass m10/m00, m01/m00
Point2D center
float64 length
float64 area

================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/MomentArrayStamped "28ac0beb70b037acf76c3bed71b679a9")


