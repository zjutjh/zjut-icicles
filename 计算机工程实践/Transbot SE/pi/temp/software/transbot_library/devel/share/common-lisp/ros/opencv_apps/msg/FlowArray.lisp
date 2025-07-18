; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude FlowArray.msg.html

(cl:defclass <FlowArray> (roslisp-msg-protocol:ros-message)
  ((flow
    :reader flow
    :initarg :flow
    :type (cl:vector opencv_apps-msg:Flow)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Flow :initial-element (cl:make-instance 'opencv_apps-msg:Flow))))
)

(cl:defclass FlowArray (<FlowArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlowArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlowArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<FlowArray> is deprecated: use opencv_apps-msg:FlowArray instead.")))

(cl:ensure-generic-function 'flow-val :lambda-list '(m))
(cl:defmethod flow-val ((m <FlowArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:flow-val is deprecated.  Use opencv_apps-msg:flow instead.")
  (flow m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlowArray>) ostream)
  "Serializes a message object of type '<FlowArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'flow))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'flow))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlowArray>) istream)
  "Deserializes a message object of type '<FlowArray>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlowArray>)))
  "Returns string type for a message object of type '<FlowArray>"
  "opencv_apps/FlowArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlowArray)))
  "Returns string type for a message object of type 'FlowArray"
  "opencv_apps/FlowArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlowArray>)))
  "Returns md5sum for a message object of type '<FlowArray>"
  "fe292dca56eb3673cd698ea9ef841962")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlowArray)))
  "Returns md5sum for a message object of type 'FlowArray"
  "fe292dca56eb3673cd698ea9ef841962")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlowArray>)))
  "Returns full string definition for message of type '<FlowArray>"
  (cl:format cl:nil "Flow[] flow~%~%================================================================================~%MSG: opencv_apps/Flow~%Point2D point~%Point2D velocity~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlowArray)))
  "Returns full string definition for message of type 'FlowArray"
  (cl:format cl:nil "Flow[] flow~%~%================================================================================~%MSG: opencv_apps/Flow~%Point2D point~%Point2D velocity~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlowArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'flow) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlowArray>))
  "Converts a ROS message object to a list"
  (cl:list 'FlowArray
    (cl:cons ':flow (flow msg))
))
