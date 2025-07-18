; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude RotatedRectArray.msg.html

(cl:defclass <RotatedRectArray> (roslisp-msg-protocol:ros-message)
  ((rects
    :reader rects
    :initarg :rects
    :type (cl:vector opencv_apps-msg:RotatedRect)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:RotatedRect :initial-element (cl:make-instance 'opencv_apps-msg:RotatedRect))))
)

(cl:defclass RotatedRectArray (<RotatedRectArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RotatedRectArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RotatedRectArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<RotatedRectArray> is deprecated: use opencv_apps-msg:RotatedRectArray instead.")))

(cl:ensure-generic-function 'rects-val :lambda-list '(m))
(cl:defmethod rects-val ((m <RotatedRectArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:rects-val is deprecated.  Use opencv_apps-msg:rects instead.")
  (rects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RotatedRectArray>) ostream)
  "Serializes a message object of type '<RotatedRectArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'rects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RotatedRectArray>) istream)
  "Deserializes a message object of type '<RotatedRectArray>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RotatedRectArray>)))
  "Returns string type for a message object of type '<RotatedRectArray>"
  "opencv_apps/RotatedRectArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotatedRectArray)))
  "Returns string type for a message object of type 'RotatedRectArray"
  "opencv_apps/RotatedRectArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RotatedRectArray>)))
  "Returns md5sum for a message object of type '<RotatedRectArray>"
  "a27e397ed2b5b1a633561d324f64d2a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RotatedRectArray)))
  "Returns md5sum for a message object of type 'RotatedRectArray"
  "a27e397ed2b5b1a633561d324f64d2a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RotatedRectArray>)))
  "Returns full string definition for message of type '<RotatedRectArray>"
  (cl:format cl:nil "RotatedRect[] rects~%~%================================================================================~%MSG: opencv_apps/RotatedRect~%float64 angle~%Point2D center~%Size size~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: opencv_apps/Size~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RotatedRectArray)))
  "Returns full string definition for message of type 'RotatedRectArray"
  (cl:format cl:nil "RotatedRect[] rects~%~%================================================================================~%MSG: opencv_apps/RotatedRect~%float64 angle~%Point2D center~%Size size~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: opencv_apps/Size~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RotatedRectArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RotatedRectArray>))
  "Converts a ROS message object to a list"
  (cl:list 'RotatedRectArray
    (cl:cons ':rects (rects msg))
))
