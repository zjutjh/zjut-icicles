;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::FlowArrayStamped)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'FlowArrayStamped (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::FLOWARRAYSTAMPED")
  (make-package "OPENCV_APPS::FLOWARRAYSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude FlowArrayStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass opencv_apps::FlowArrayStamped
  :super ros::object
  :slots (_header _flow ))

(defmethod opencv_apps::FlowArrayStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:flow __flow) (let (r) (dotimes (i 0) (push (instance opencv_apps::Flow :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _flow __flow)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:flow
   (&rest __flow)
   (if (keywordp (car __flow))
       (send* _flow __flow)
     (progn
       (if __flow (setq _flow (car __flow)))
       _flow)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; opencv_apps/Flow[] _flow
    (apply #'+ (send-all _flow :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; opencv_apps/Flow[] _flow
     (write-long (length _flow) s)
     (dolist (elem _flow)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; opencv_apps/Flow[] _flow
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _flow (let (r) (dotimes (i n) (push (instance opencv_apps::Flow :init) r)) r))
     (dolist (elem- _flow)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::FlowArrayStamped :md5sum-) "b55faf909449963372b92417925b68cc")
(setf (get opencv_apps::FlowArrayStamped :datatype-) "opencv_apps/FlowArrayStamped")
(setf (get opencv_apps::FlowArrayStamped :definition-)
      "Header header
Flow[] flow

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
MSG: opencv_apps/Flow
Point2D point
Point2D velocity

================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/FlowArrayStamped "b55faf909449963372b92417925b68cc")


