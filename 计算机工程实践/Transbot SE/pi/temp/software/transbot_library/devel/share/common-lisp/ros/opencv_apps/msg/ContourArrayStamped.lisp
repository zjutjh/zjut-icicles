; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude ContourArrayStamped.msg.html

(cl:defclass <ContourArrayStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (contours
    :reader contours
    :initarg :contours
    :type (cl:vector opencv_apps-msg:Contour)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Contour :initial-element (cl:make-instance 'opencv_apps-msg:Contour))))
)

(cl:defclass ContourArrayStamped (<ContourArrayStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ContourArrayStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ContourArrayStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<ContourArrayStamped> is deprecated: use opencv_apps-msg:ContourArrayStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ContourArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:header-val is deprecated.  Use opencv_apps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'contours-val :lambda-list '(m))
(cl:defmethod contours-val ((m <ContourArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:contours-val is deprecated.  Use opencv_apps-msg:contours instead.")
  (contours m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ContourArrayStamped>) ostream)
  "Serializes a message object of type '<ContourArrayStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'contours))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'contours))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ContourArrayStamped>) istream)
  "Deserializes a message object of type '<ContourArrayStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'contours) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'contours)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Contour))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ContourArrayStamped>)))
  "Returns string type for a message object of type '<ContourArrayStamped>"
  "opencv_apps/ContourArrayStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ContourArrayStamped)))
  "Returns string type for a message object of type 'ContourArrayStamped"
  "opencv_apps/ContourArrayStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ContourArrayStamped>)))
  "Returns md5sum for a message object of type '<ContourArrayStamped>"
  "6bcf2733566be102cf11fc89685fd962")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ContourArrayStamped)))
  "Returns md5sum for a message object of type 'ContourArrayStamped"
  "6bcf2733566be102cf11fc89685fd962")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ContourArrayStamped>)))
  "Returns full string definition for message of type '<ContourArrayStamped>"
  (cl:format cl:nil "Header header~%Contour[] contours~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Contour~%Point2D[] points~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ContourArrayStamped)))
  "Returns full string definition for message of type 'ContourArrayStamped"
  (cl:format cl:nil "Header header~%Contour[] contours~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Contour~%Point2D[] points~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ContourArrayStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'contours) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ContourArrayStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'ContourArrayStamped
    (cl:cons ':header (header msg))
    (cl:cons ':contours (contours msg))
))
