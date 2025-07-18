; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude MomentArrayStamped.msg.html

(cl:defclass <MomentArrayStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (moments
    :reader moments
    :initarg :moments
    :type (cl:vector opencv_apps-msg:Moment)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Moment :initial-element (cl:make-instance 'opencv_apps-msg:Moment))))
)

(cl:defclass MomentArrayStamped (<MomentArrayStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MomentArrayStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MomentArrayStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<MomentArrayStamped> is deprecated: use opencv_apps-msg:MomentArrayStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MomentArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:header-val is deprecated.  Use opencv_apps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'moments-val :lambda-list '(m))
(cl:defmethod moments-val ((m <MomentArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:moments-val is deprecated.  Use opencv_apps-msg:moments instead.")
  (moments m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MomentArrayStamped>) ostream)
  "Serializes a message object of type '<MomentArrayStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'moments))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'moments))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MomentArrayStamped>) istream)
  "Deserializes a message object of type '<MomentArrayStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'moments) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'moments)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Moment))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MomentArrayStamped>)))
  "Returns string type for a message object of type '<MomentArrayStamped>"
  "opencv_apps/MomentArrayStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MomentArrayStamped)))
  "Returns string type for a message object of type 'MomentArrayStamped"
  "opencv_apps/MomentArrayStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MomentArrayStamped>)))
  "Returns md5sum for a message object of type '<MomentArrayStamped>"
  "28ac0beb70b037acf76c3bed71b679a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MomentArrayStamped)))
  "Returns md5sum for a message object of type 'MomentArrayStamped"
  "28ac0beb70b037acf76c3bed71b679a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MomentArrayStamped>)))
  "Returns full string definition for message of type '<MomentArrayStamped>"
  (cl:format cl:nil "Header header~%Moment[] moments~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Moment~%# spatial moments~%float64 m00~%float64 m10~%float64 m01~%float64 m20~%float64 m11~%float64 m02~%float64 m30~%float64 m21~%float64 m12~%float64 m03~%~%# central moments~%float64 mu20~%float64 mu11~%float64 mu02~%float64 mu30~%float64 mu21~%float64 mu12~%float64 mu03~%~%# central normalized moments~%float64 nu20~%float64 nu11~%float64 nu02~%float64 nu30~%float64 nu21~%float64 nu12~%float64 nu03~%~%# center of mass m10/m00, m01/m00~%Point2D center~%float64 length~%float64 area~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MomentArrayStamped)))
  "Returns full string definition for message of type 'MomentArrayStamped"
  (cl:format cl:nil "Header header~%Moment[] moments~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Moment~%# spatial moments~%float64 m00~%float64 m10~%float64 m01~%float64 m20~%float64 m11~%float64 m02~%float64 m30~%float64 m21~%float64 m12~%float64 m03~%~%# central moments~%float64 mu20~%float64 mu11~%float64 mu02~%float64 mu30~%float64 mu21~%float64 mu12~%float64 mu03~%~%# central normalized moments~%float64 nu20~%float64 nu11~%float64 nu02~%float64 nu30~%float64 nu21~%float64 nu12~%float64 nu03~%~%# center of mass m10/m00, m01/m00~%Point2D center~%float64 length~%float64 area~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MomentArrayStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'moments) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MomentArrayStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'MomentArrayStamped
    (cl:cons ':header (header msg))
    (cl:cons ':moments (moments msg))
))
