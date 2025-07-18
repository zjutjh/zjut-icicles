;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::FlowStamped)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'FlowStamped (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::FLOWSTAMPED")
  (make-package "OPENCV_APPS::FLOWSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude FlowStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass opencv_apps::FlowStamped
  :super ros::object
  :slots (_header _flow ))

(defmethod opencv_apps::FlowStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:flow __flow) (instance opencv_apps::Flow :init))
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
    ;; opencv_apps/Flow _flow
    (send _flow :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; opencv_apps/Flow _flow
       (send _flow :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; opencv_apps/Flow _flow
     (send _flow :deserialize buf ptr-) (incf ptr- (send _flow :serialization-length))
   ;;
   self)
  )

(setf (get opencv_apps::FlowStamped :md5sum-) "b55faf909449963372b92417925b68cc")
(setf (get opencv_apps::FlowStamped :datatype-) "opencv_apps/FlowStamped")
(setf (get opencv_apps::FlowStamped :definition-)
      "Header header
Flow flow

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



(provide :opencv_apps/FlowStamped "b55faf909449963372b92417925b68cc")


