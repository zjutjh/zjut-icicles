; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude FlowStamped.msg.html

(cl:defclass <FlowStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (flow
    :reader flow
    :initarg :flow
    :type opencv_apps-msg:Flow
    :initform (cl:make-instance 'opencv_apps-msg:Flow)))
)

(cl:defclass FlowStamped (<FlowStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlowStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlowStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<FlowStamped> is deprecated: use opencv_apps-msg:FlowStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FlowStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:header-val is deprecated.  Use opencv_apps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'flow-val :lambda-list '(m))
(cl:defmethod flow-val ((m <FlowStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:flow-val is deprecated.  Use opencv_apps-msg:flow instead.")
  (flow m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlowStamped>) ostream)
  "Serializes a message object of type '<FlowStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'flow) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlowStamped>) istream)
  "Deserializes a message object of type '<FlowStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'flow) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlowStamped>)))
  "Returns string type for a message object of type '<FlowStamped>"
  "opencv_apps/FlowStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlowStamped)))
  "Returns string type for a message object of type 'FlowStamped"
  "opencv_apps/FlowStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlowStamped>)))
  "Returns md5sum for a message object of type '<FlowStamped>"
  "b55faf909449963372b92417925b68cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlowStamped)))
  "Returns md5sum for a message object of type 'FlowStamped"
  "b55faf909449963372b92417925b68cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlowStamped>)))
  "Returns full string definition for message of type '<FlowStamped>"
  (cl:format cl:nil "Header header~%Flow flow~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Flow~%Point2D point~%Point2D velocity~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlowStamped)))
  "Returns full string definition for message of type 'FlowStamped"
  (cl:format cl:nil "Header header~%Flow flow~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Flow~%Point2D point~%Point2D velocity~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlowStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'flow))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlowStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'FlowStamped
    (cl:cons ':header (header msg))
    (cl:cons ':flow (flow msg))
))
