; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude FlowArrayStamped.msg.html

(cl:defclass <FlowArrayStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (flow
    :reader flow
    :initarg :flow
    :type (cl:vector opencv_apps-msg:Flow)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Flow :initial-element (cl:make-instance 'opencv_apps-msg:Flow))))
)

(cl:defclass FlowArrayStamped (<FlowArrayStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlowArrayStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlowArrayStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<FlowArrayStamped> is deprecated: use opencv_apps-msg:FlowArrayStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FlowArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:header-val is deprecated.  Use opencv_apps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'flow-val :lambda-list '(m))
(cl:defmethod flow-val ((m <FlowArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:flow-val is deprecated.  Use opencv_apps-msg:flow instead.")
  (flow m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlowArrayStamped>) ostream)
  "Serializes a message object of type '<FlowArrayStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'flow))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'flow))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlowArrayStamped>) istream)
  "Deserializes a message object of type '<FlowArrayStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'flow) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'flow)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Flow))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlowArrayStamped>)))
  "Returns string type for a message object of type '<FlowArrayStamped>"
  "opencv_apps/FlowArrayStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlowArrayStamped)))
  "Returns string type for a message object of type 'FlowArrayStamped"
  "opencv_apps/FlowArrayStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlowArrayStamped>)))
  "Returns md5sum for a message object of type '<FlowArrayStamped>"
  "b55faf909449963372b92417925b68cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlowArrayStamped)))
  "Returns md5sum for a message object of type 'FlowArrayStamped"
  "b55faf909449963372b92417925b68cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlowArrayStamped>)))
  "Returns full string definition for message of type '<FlowArrayStamped>"
  (cl:format cl:nil "Header header~%Flow[] flow~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Flow~%Point2D point~%Point2D velocity~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlowArrayStamped)))
  "Returns full string definition for message of type 'FlowArrayStamped"
  (cl:format cl:nil "Header header~%Flow[] flow~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Flow~%Point2D point~%Point2D velocity~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlowArrayStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'flow) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlowArrayStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'FlowArrayStamped
    (cl:cons ':header (header msg))
    (cl:cons ':flow (flow msg))
))
