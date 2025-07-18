; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude Line.msg.html

(cl:defclass <Line> (roslisp-msg-protocol:ros-message)
  ((pt1
    :reader pt1
    :initarg :pt1
    :type opencv_apps-msg:Point2D
    :initform (cl:make-instance 'opencv_apps-msg:Point2D))
   (pt2
    :reader pt2
    :initarg :pt2
    :type opencv_apps-msg:Point2D
    :initform (cl:make-instance 'opencv_apps-msg:Point2D)))
)

(cl:defclass Line (<Line>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Line>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Line)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<Line> is deprecated: use opencv_apps-msg:Line instead.")))

(cl:ensure-generic-function 'pt1-val :lambda-list '(m))
(cl:defmethod pt1-val ((m <Line>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:pt1-val is deprecated.  Use opencv_apps-msg:pt1 instead.")
  (pt1 m))

(cl:ensure-generic-function 'pt2-val :lambda-list '(m))
(cl:defmethod pt2-val ((m <Line>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:pt2-val is deprecated.  Use opencv_apps-msg:pt2 instead.")
  (pt2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Line>) ostream)
  "Serializes a message object of type '<Line>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pt1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pt2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Line>) istream)
  "Deserializes a message object of type '<Line>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pt1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pt2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Line>)))
  "Returns string type for a message object of type '<Line>"
  "opencv_apps/Line")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Line)))
  "Returns string type for a message object of type 'Line"
  "opencv_apps/Line")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Line>)))
  "Returns md5sum for a message object of type '<Line>"
  "a1419010b3fc4549e3f450018363d000")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Line)))
  "Returns md5sum for a message object of type 'Line"
  "a1419010b3fc4549e3f450018363d000")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Line>)))
  "Returns full string definition for message of type '<Line>"
  (cl:format cl:nil "Point2D pt1~%Point2D pt2~%~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Line)))
  "Returns full string definition for message of type 'Line"
  (cl:format cl:nil "Point2D pt1~%Point2D pt2~%~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Line>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pt1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pt2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Line>))
  "Converts a ROS message object to a list"
  (cl:list 'Line
    (cl:cons ':pt1 (pt1 msg))
    (cl:cons ':pt2 (pt2 msg))
))
