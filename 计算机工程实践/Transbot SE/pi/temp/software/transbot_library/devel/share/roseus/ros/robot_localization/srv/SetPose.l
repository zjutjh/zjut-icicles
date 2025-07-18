;; Auto-generated. Do not edit!


(when (boundp 'robot_localization::SetPose)
  (if (not (find-package "ROBOT_LOCALIZATION"))
    (make-package "ROBOT_LOCALIZATION"))
  (shadow 'SetPose (find-package "ROBOT_LOCALIZATION")))
(unless (find-package "ROBOT_LOCALIZATION::SETPOSE")
  (make-package "ROBOT_LOCALIZATION::SETPOSE"))
(unless (find-package "ROBOT_LOCALIZATION::SETPOSEREQUEST")
  (make-package "ROBOT_LOCALIZATION::SETPOSEREQUEST"))
(unless (find-package "ROBOT_LOCALIZATION::SETPOSERESPONSE")
  (make-package "ROBOT_LOCALIZATION::SETPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass robot_localization::SetPoseRequest
  :super ros::object
  :slots (_pose ))

(defmethod robot_localization::SetPoseRequest
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::PoseWithCovarianceStamped :init))
    )
   (send-super :init)
   (setq _pose __pose)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseWithCovarianceStamped _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseWithCovarianceStamped _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseWithCovarianceStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass robot_localization::SetPoseResponse
  :super ros::object
  :slots ())

(defmethod robot_localization::SetPoseResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass robot_localization::SetPose
  :super ros::object
  :slots ())

(setf (get robot_localization::SetPose :md5sum-) "4f3e0bbe7a24e1f929488cd1970222d3")
(setf (get robot_localization::SetPose :datatype-) "robot_localization/SetPose")
(setf (get robot_localization::SetPose :request) robot_localization::SetPoseRequest)
(setf (get robot_localization::SetPose :response) robot_localization::SetPoseResponse)

(defmethod robot_localization::SetPoseRequest
  (:response () (instance robot_localization::SetPoseResponse :init)))

(setf (get robot_localization::SetPoseRequest :md5sum-) "4f3e0bbe7a24e1f929488cd1970222d3")
(setf (get robot_localization::SetPoseRequest :datatype-) "robot_localization/SetPoseRequest")
(setf (get robot_localization::SetPoseRequest :definition-)
      "geometry_msgs/PoseWithCovarianceStamped pose

================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

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
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---

")

(setf (get robot_localization::SetPoseResponse :md5sum-) "4f3e0bbe7a24e1f929488cd1970222d3")
(setf (get robot_localization::SetPoseResponse :datatype-) "robot_localization/SetPoseResponse")
(setf (get robot_localization::SetPoseResponse :definition-)
      "geometry_msgs/PoseWithCovarianceStamped pose

================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

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
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---

")



(provide :robot_localization/SetPose "4f3e0bbe7a24e1f929488cd1970222d3")


