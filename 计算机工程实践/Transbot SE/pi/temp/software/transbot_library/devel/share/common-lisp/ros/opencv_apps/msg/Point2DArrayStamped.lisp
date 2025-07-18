; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude Point2DArrayStamped.msg.html

(cl:defclass <Point2DArrayStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (points
    :reader points
    :initarg :points
    :type (cl:vector opencv_apps-msg:Point2D)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Point2D :initial-element (cl:make-instance 'opencv_apps-msg:Point2D))))
)

(cl:defclass Point2DArrayStamped (<Point2DArrayStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Point2DArrayStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Point2DArrayStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<Point2DArrayStamped> is deprecated: use opencv_apps-msg:Point2DArrayStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Point2DArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:header-val is deprecated.  Use opencv_apps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <Point2DArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:points-val is deprecated.  Use opencv_apps-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Point2DArrayStamped>) ostream)
  "Serializes a message object of type '<Point2DArrayStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Point2DArrayStamped>) istream)
  "Deserializes a message object of type '<Point2DArrayStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Point2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Point2DArrayStamped>)))
  "Returns string type for a message object of type '<Point2DArrayStamped>"
  "opencv_apps/Point2DArrayStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Point2DArrayStamped)))
  "Returns string type for a message object of type 'Point2DArrayStamped"
  "opencv_apps/Point2DArrayStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Point2DArrayStamped>)))
  "Returns md5sum for a message object of type '<Point2DArrayStamped>"
  "06c8694bd7b07dbc04014c86ef9991a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Point2DArrayStamped)))
  "Returns md5sum for a message object of type 'Point2DArrayStamped"
  "06c8694bd7b07dbc04014c86ef9991a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Point2DArrayStamped>)))
  "Returns full string definition for message of type '<Point2DArrayStamped>"
  (cl:format cl:nil "Header header~%Point2D[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Point2DArrayStamped)))
  "Returns full string definition for message of type 'Point2DArrayStamped"
  (cl:format cl:nil "Header header~%Point2D[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Point2DArrayStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Point2DArrayStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'Point2DArrayStamped
    (cl:cons ':header (header msg))
    (cl:cons ':points (points msg))
))
