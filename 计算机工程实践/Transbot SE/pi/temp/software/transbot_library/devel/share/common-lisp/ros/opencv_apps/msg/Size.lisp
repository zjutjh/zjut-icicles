; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude Size.msg.html

(cl:defclass <Size> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0)
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0))
)

(cl:defclass Size (<Size>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Size>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Size)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<Size> is deprecated: use opencv_apps-msg:Size instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <Size>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:width-val is deprecated.  Use opencv_apps-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <Size>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:height-val is deprecated.  Use opencv_apps-msg:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Size>) ostream)
  "Serializes a message object of type '<Size>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Size>) istream)
  "Deserializes a message object of type '<Size>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Size>)))
  "Returns string type for a message object of type '<Size>"
  "opencv_apps/Size")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Size)))
  "Returns string type for a message object of type 'Size"
  "opencv_apps/Size")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Size>)))
  "Returns md5sum for a message object of type '<Size>"
  "f86522e647336fb10b55359fe003f673")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Size)))
  "Returns md5sum for a message object of type 'Size"
  "f86522e647336fb10b55359fe003f673")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Size>)))
  "Returns full string definition for message of type '<Size>"
  (cl:format cl:nil "float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Size)))
  "Returns full string definition for message of type 'Size"
  (cl:format cl:nil "float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Size>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Size>))
  "Converts a ROS message object to a list"
  (cl:list 'Size
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
