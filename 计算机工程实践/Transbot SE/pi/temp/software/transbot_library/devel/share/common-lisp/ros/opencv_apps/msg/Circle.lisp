; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude Circle.msg.html

(cl:defclass <Circle> (roslisp-msg-protocol:ros-message)
  ((center
    :reader center
    :initarg :center
    :type opencv_apps-msg:Point2D
    :initform (cl:make-instance 'opencv_apps-msg:Point2D))
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0))
)

(cl:defclass Circle (<Circle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Circle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Circle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<Circle> is deprecated: use opencv_apps-msg:Circle instead.")))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <Circle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:center-val is deprecated.  Use opencv_apps-msg:center instead.")
  (center m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <Circle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:radius-val is deprecated.  Use opencv_apps-msg:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Circle>) ostream)
  "Serializes a message object of type '<Circle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Circle>) istream)
  "Deserializes a message object of type '<Circle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Circle>)))
  "Returns string type for a message object of type '<Circle>"
  "opencv_apps/Circle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Circle)))
  "Returns string type for a message object of type 'Circle"
  "opencv_apps/Circle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Circle>)))
  "Returns md5sum for a message object of type '<Circle>"
  "4f6847051b4fe493b5af8caad66201d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Circle)))
  "Returns md5sum for a message object of type 'Circle"
  "4f6847051b4fe493b5af8caad66201d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Circle>)))
  "Returns full string definition for message of type '<Circle>"
  (cl:format cl:nil "Point2D center~%float64 radius~%~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Circle)))
  "Returns full string definition for message of type 'Circle"
  (cl:format cl:nil "Point2D center~%float64 radius~%~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Circle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Circle>))
  "Converts a ROS message object to a list"
  (cl:list 'Circle
    (cl:cons ':center (center msg))
    (cl:cons ':radius (radius msg))
))
