; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude Point2DStamped.msg.html

(cl:defclass <Point2DStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (point
    :reader point
    :initarg :point
    :type opencv_apps-msg:Point2D
    :initform (cl:make-instance 'opencv_apps-msg:Point2D)))
)

(cl:defclass Point2DStamped (<Point2DStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Point2DStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Point2DStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<Point2DStamped> is deprecated: use opencv_apps-msg:Point2DStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Point2DStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:header-val is deprecated.  Use opencv_apps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <Point2DStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:point-val is deprecated.  Use opencv_apps-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Point2DStamped>) ostream)
  "Serializes a message object of type '<Point2DStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Point2DStamped>) istream)
  "Deserializes a message object of type '<Point2DStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Point2DStamped>)))
  "Returns string type for a message object of type '<Point2DStamped>"
  "opencv_apps/Point2DStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Point2DStamped)))
  "Returns string type for a message object of type 'Point2DStamped"
  "opencv_apps/Point2DStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Point2DStamped>)))
  "Returns md5sum for a message object of type '<Point2DStamped>"
  "9f7db918fde9989a73131d0d083d049d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Point2DStamped)))
  "Returns md5sum for a message object of type 'Point2DStamped"
  "9f7db918fde9989a73131d0d083d049d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Point2DStamped>)))
  "Returns full string definition for message of type '<Point2DStamped>"
  (cl:format cl:nil "Header header~%Point2D point~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Point2DStamped)))
  "Returns full string definition for message of type 'Point2DStamped"
  (cl:format cl:nil "Header header~%Point2D point~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Point2DStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Point2DStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'Point2DStamped
    (cl:cons ':header (header msg))
    (cl:cons ':point (point msg))
))
