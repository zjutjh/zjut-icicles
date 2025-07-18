; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude Flow.msg.html

(cl:defclass <Flow> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type opencv_apps-msg:Point2D
    :initform (cl:make-instance 'opencv_apps-msg:Point2D))
   (velocity
    :reader velocity
    :initarg :velocity
    :type opencv_apps-msg:Point2D
    :initform (cl:make-instance 'opencv_apps-msg:Point2D)))
)

(cl:defclass Flow (<Flow>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Flow>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Flow)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<Flow> is deprecated: use opencv_apps-msg:Flow instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <Flow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:point-val is deprecated.  Use opencv_apps-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <Flow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:velocity-val is deprecated.  Use opencv_apps-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Flow>) ostream)
  "Serializes a message object of type '<Flow>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Flow>) istream)
  "Deserializes a message object of type '<Flow>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Flow>)))
  "Returns string type for a message object of type '<Flow>"
  "opencv_apps/Flow")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Flow)))
  "Returns string type for a message object of type 'Flow"
  "opencv_apps/Flow")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Flow>)))
  "Returns md5sum for a message object of type '<Flow>"
  "dd9a9efd88ba39035e78af697593d751")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Flow)))
  "Returns md5sum for a message object of type 'Flow"
  "dd9a9efd88ba39035e78af697593d751")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Flow>)))
  "Returns full string definition for message of type '<Flow>"
  (cl:format cl:nil "Point2D point~%Point2D velocity~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Flow)))
  "Returns full string definition for message of type 'Flow"
  (cl:format cl:nil "Point2D point~%Point2D velocity~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Flow>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Flow>))
  "Converts a ROS message object to a list"
  (cl:list 'Flow
    (cl:cons ':point (point msg))
    (cl:cons ':velocity (velocity msg))
))
