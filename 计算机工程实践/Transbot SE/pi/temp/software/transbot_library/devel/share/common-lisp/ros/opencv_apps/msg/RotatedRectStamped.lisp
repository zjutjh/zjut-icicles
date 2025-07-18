; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude RotatedRectStamped.msg.html

(cl:defclass <RotatedRectStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rect
    :reader rect
    :initarg :rect
    :type opencv_apps-msg:RotatedRect
    :initform (cl:make-instance 'opencv_apps-msg:RotatedRect)))
)

(cl:defclass RotatedRectStamped (<RotatedRectStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RotatedRectStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RotatedRectStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<RotatedRectStamped> is deprecated: use opencv_apps-msg:RotatedRectStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RotatedRectStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:header-val is deprecated.  Use opencv_apps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rect-val :lambda-list '(m))
(cl:defmethod rect-val ((m <RotatedRectStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:rect-val is deprecated.  Use opencv_apps-msg:rect instead.")
  (rect m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RotatedRectStamped>) ostream)
  "Serializes a message object of type '<RotatedRectStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rect) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RotatedRectStamped>) istream)
  "Deserializes a message object of type '<RotatedRectStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rect) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RotatedRectStamped>)))
  "Returns string type for a message object of type '<RotatedRectStamped>"
  "opencv_apps/RotatedRectStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotatedRectStamped)))
  "Returns string type for a message object of type 'RotatedRectStamped"
  "opencv_apps/RotatedRectStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RotatedRectStamped>)))
  "Returns md5sum for a message object of type '<RotatedRectStamped>"
  "ba2d76a1968e4f77570c01223781fe15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RotatedRectStamped)))
  "Returns md5sum for a message object of type 'RotatedRectStamped"
  "ba2d76a1968e4f77570c01223781fe15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RotatedRectStamped>)))
  "Returns full string definition for message of type '<RotatedRectStamped>"
  (cl:format cl:nil "Header header~%RotatedRect rect~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/RotatedRect~%float64 angle~%Point2D center~%Size size~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: opencv_apps/Size~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RotatedRectStamped)))
  "Returns full string definition for message of type 'RotatedRectStamped"
  (cl:format cl:nil "Header header~%RotatedRect rect~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/RotatedRect~%float64 angle~%Point2D center~%Size size~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: opencv_apps/Size~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RotatedRectStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rect))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RotatedRectStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'RotatedRectStamped
    (cl:cons ':header (header msg))
    (cl:cons ':rect (rect msg))
))
