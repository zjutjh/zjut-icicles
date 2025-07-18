;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::FaceArrayStamped)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'FaceArrayStamped (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::FACEARRAYSTAMPED")
  (make-package "OPENCV_APPS::FACEARRAYSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude FaceArrayStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass opencv_apps::FaceArrayStamped
  :super ros::object
  :slots (_header _faces ))

(defmethod opencv_apps::FaceArrayStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:faces __faces) (let (r) (dotimes (i 0) (push (instance opencv_apps::Face :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _faces __faces)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:faces
   (&rest __faces)
   (if (keywordp (car __faces))
       (send* _faces __faces)
     (progn
       (if __faces (setq _faces (car __faces)))
       _faces)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; opencv_apps/Face[] _faces
    (apply #'+ (send-all _faces :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; opencv_apps/Face[] _faces
     (write-long (length _faces) s)
     (dolist (elem _faces)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; opencv_apps/Face[] _faces
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _faces (let (r) (dotimes (i n) (push (instance opencv_apps::Face :init) r)) r))
     (dolist (elem- _faces)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::FaceArrayStamped :md5sum-) "a43dedd70c7b2338df14a8f4de0940ef")
(setf (get opencv_apps::FaceArrayStamped :datatype-) "opencv_apps/FaceArrayStamped")
(setf (get opencv_apps::FaceArrayStamped :definition-)
      "Header header
Face[] faces


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
MSG: opencv_apps/Face
Rect face
Rect[] eyes
string label
float64 confidence

================================================================================
MSG: opencv_apps/Rect
# opencv Rect data type, x-y is center point
float64 x
float64 y
float64 width
float64 height


")



(provide :opencv_apps/FaceArrayStamped "a43dedd70c7b2338df14a8f4de0940ef")


