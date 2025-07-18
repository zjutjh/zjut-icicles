; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude RotatedRect.msg.html

(cl:defclass <RotatedRect> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (center
    :reader center
    :initarg :center
    :type opencv_apps-msg:Point2D
    :initform (cl:make-instance 'opencv_apps-msg:Point2D))
   (size
    :reader size
    :initarg :size
    :type opencv_apps-msg:Size
    :initform (cl:make-instance 'opencv_apps-msg:Size)))
)

(cl:defclass RotatedRect (<RotatedRect>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RotatedRect>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RotatedRect)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<RotatedRect> is deprecated: use opencv_apps-msg:RotatedRect instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <RotatedRect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:angle-val is deprecated.  Use opencv_apps-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <RotatedRect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:center-val is deprecated.  Use opencv_apps-msg:center instead.")
  (center m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <RotatedRect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:size-val is deprecated.  Use opencv_apps-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RotatedRect>) ostream)
  "Serializes a message object of type '<RotatedRect>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RotatedRect>) istream)
  "Deserializes a message object of type '<RotatedRect>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RotatedRect>)))
  "Returns string type for a message object of type '<RotatedRect>"
  "opencv_apps/RotatedRect")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotatedRect)))
  "Returns string type for a message object of type 'RotatedRect"
  "opencv_apps/RotatedRect")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RotatedRect>)))
  "Returns md5sum for a message object of type '<RotatedRect>"
  "0ae60505c52f020176686d0689b8d390")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RotatedRect)))
  "Returns md5sum for a message object of type 'RotatedRect"
  "0ae60505c52f020176686d0689b8d390")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RotatedRect>)))
  "Returns full string definition for message of type '<RotatedRect>"
  (cl:format cl:nil "float64 angle~%Point2D center~%Size size~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: opencv_apps/Size~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RotatedRect)))
  "Returns full string definition for message of type 'RotatedRect"
  (cl:format cl:nil "float64 angle~%Point2D center~%Size size~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: opencv_apps/Size~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RotatedRect>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RotatedRect>))
  "Converts a ROS message object to a list"
  (cl:list 'RotatedRect
    (cl:cons ':angle (angle msg))
    (cl:cons ':center (center msg))
    (cl:cons ':size (size msg))
))
