; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude CircleArrayStamped.msg.html

(cl:defclass <CircleArrayStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (circles
    :reader circles
    :initarg :circles
    :type (cl:vector opencv_apps-msg:Circle)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Circle :initial-element (cl:make-instance 'opencv_apps-msg:Circle))))
)

(cl:defclass CircleArrayStamped (<CircleArrayStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CircleArrayStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CircleArrayStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<CircleArrayStamped> is deprecated: use opencv_apps-msg:CircleArrayStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CircleArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:header-val is deprecated.  Use opencv_apps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'circles-val :lambda-list '(m))
(cl:defmethod circles-val ((m <CircleArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:circles-val is deprecated.  Use opencv_apps-msg:circles instead.")
  (circles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CircleArrayStamped>) ostream)
  "Serializes a message object of type '<CircleArrayStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'circles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'circles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CircleArrayStamped>) istream)
  "Deserializes a message object of type '<CircleArrayStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'circles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'circles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Circle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CircleArrayStamped>)))
  "Returns string type for a message object of type '<CircleArrayStamped>"
  "opencv_apps/CircleArrayStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CircleArrayStamped)))
  "Returns string type for a message object of type 'CircleArrayStamped"
  "opencv_apps/CircleArrayStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CircleArrayStamped>)))
  "Returns md5sum for a message object of type '<CircleArrayStamped>"
  "430ffa6c2b0a36b7e81feff1ce79c3c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CircleArrayStamped)))
  "Returns md5sum for a message object of type 'CircleArrayStamped"
  "430ffa6c2b0a36b7e81feff1ce79c3c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CircleArrayStamped>)))
  "Returns full string definition for message of type '<CircleArrayStamped>"
  (cl:format cl:nil "Header header~%Circle[] circles~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Circle~%Point2D center~%float64 radius~%~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CircleArrayStamped)))
  "Returns full string definition for message of type 'CircleArrayStamped"
  (cl:format cl:nil "Header header~%Circle[] circles~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Circle~%Point2D center~%float64 radius~%~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CircleArrayStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'circles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CircleArrayStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'CircleArrayStamped
    (cl:cons ':header (header msg))
    (cl:cons ':circles (circles msg))
))
