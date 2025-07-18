; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude CircleArray.msg.html

(cl:defclass <CircleArray> (roslisp-msg-protocol:ros-message)
  ((circles
    :reader circles
    :initarg :circles
    :type (cl:vector opencv_apps-msg:Circle)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Circle :initial-element (cl:make-instance 'opencv_apps-msg:Circle))))
)

(cl:defclass CircleArray (<CircleArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CircleArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CircleArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<CircleArray> is deprecated: use opencv_apps-msg:CircleArray instead.")))

(cl:ensure-generic-function 'circles-val :lambda-list '(m))
(cl:defmethod circles-val ((m <CircleArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:circles-val is deprecated.  Use opencv_apps-msg:circles instead.")
  (circles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CircleArray>) ostream)
  "Serializes a message object of type '<CircleArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'circles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'circles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CircleArray>) istream)
  "Deserializes a message object of type '<CircleArray>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CircleArray>)))
  "Returns string type for a message object of type '<CircleArray>"
  "opencv_apps/CircleArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CircleArray)))
  "Returns string type for a message object of type 'CircleArray"
  "opencv_apps/CircleArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CircleArray>)))
  "Returns md5sum for a message object of type '<CircleArray>"
  "1970b146e338dd024c765e522039a727")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CircleArray)))
  "Returns md5sum for a message object of type 'CircleArray"
  "1970b146e338dd024c765e522039a727")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CircleArray>)))
  "Returns full string definition for message of type '<CircleArray>"
  (cl:format cl:nil "Circle[] circles~%~%~%================================================================================~%MSG: opencv_apps/Circle~%Point2D center~%float64 radius~%~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CircleArray)))
  "Returns full string definition for message of type 'CircleArray"
  (cl:format cl:nil "Circle[] circles~%~%~%================================================================================~%MSG: opencv_apps/Circle~%Point2D center~%float64 radius~%~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CircleArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'circles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CircleArray>))
  "Converts a ROS message object to a list"
  (cl:list 'CircleArray
    (cl:cons ':circles (circles msg))
))
