; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude RotatedRectArrayStamped.msg.html

(cl:defclass <RotatedRectArrayStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rects
    :reader rects
    :initarg :rects
    :type (cl:vector opencv_apps-msg:RotatedRect)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:RotatedRect :initial-element (cl:make-instance 'opencv_apps-msg:RotatedRect))))
)

(cl:defclass RotatedRectArrayStamped (<RotatedRectArrayStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RotatedRectArrayStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RotatedRectArrayStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<RotatedRectArrayStamped> is deprecated: use opencv_apps-msg:RotatedRectArrayStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RotatedRectArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:header-val is deprecated.  Use opencv_apps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rects-val :lambda-list '(m))
(cl:defmethod rects-val ((m <RotatedRectArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:rects-val is deprecated.  Use opencv_apps-msg:rects instead.")
  (rects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RotatedRectArrayStamped>) ostream)
  "Serializes a message object of type '<RotatedRectArrayStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'rects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RotatedRectArrayStamped>) istream)
  "Deserializes a message object of type '<RotatedRectArrayStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:RotatedRect))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RotatedRectArrayStamped>)))
  "Returns string type for a message object of type '<RotatedRectArrayStamped>"
  "opencv_apps/RotatedRectArrayStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotatedRectArrayStamped)))
  "Returns string type for a message object of type 'RotatedRectArrayStamped"
  "opencv_apps/RotatedRectArrayStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RotatedRectArrayStamped>)))
  "Returns md5sum for a message object of type '<RotatedRectArrayStamped>"
  "89a2d4a7db2d2945ca46c25a3bd8c7c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RotatedRectArrayStamped)))
  "Returns md5sum for a message object of type 'RotatedRectArrayStamped"
  "89a2d4a7db2d2945ca46c25a3bd8c7c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RotatedRectArrayStamped>)))
  "Returns full string definition for message of type '<RotatedRectArrayStamped>"
  (cl:format cl:nil "Header header~%RotatedRect[] rects~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/RotatedRect~%float64 angle~%Point2D center~%Size size~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: opencv_apps/Size~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RotatedRectArrayStamped)))
  "Returns full string definition for message of type 'RotatedRectArrayStamped"
  (cl:format cl:nil "Header header~%RotatedRect[] rects~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/RotatedRect~%float64 angle~%Point2D center~%Size size~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: opencv_apps/Size~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RotatedRectArrayStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RotatedRectArrayStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'RotatedRectArrayStamped
    (cl:cons ':header (header msg))
    (cl:cons ':rects (rects msg))
))
