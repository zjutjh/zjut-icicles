; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-msg)


;//! \htmlinclude Adjust.msg.html

(cl:defclass <Adjust> (roslisp-msg-protocol:ros-message)
  ((adjust
    :reader adjust
    :initarg :adjust
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Adjust (<Adjust>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Adjust>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Adjust)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-msg:<Adjust> is deprecated: use transbot_msgs-msg:Adjust instead.")))

(cl:ensure-generic-function 'adjust-val :lambda-list '(m))
(cl:defmethod adjust-val ((m <Adjust>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:adjust-val is deprecated.  Use transbot_msgs-msg:adjust instead.")
  (adjust m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Adjust>) ostream)
  "Serializes a message object of type '<Adjust>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'adjust) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Adjust>) istream)
  "Deserializes a message object of type '<Adjust>"
    (cl:setf (cl:slot-value msg 'adjust) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Adjust>)))
  "Returns string type for a message object of type '<Adjust>"
  "transbot_msgs/Adjust")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Adjust)))
  "Returns string type for a message object of type 'Adjust"
  "transbot_msgs/Adjust")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Adjust>)))
  "Returns md5sum for a message object of type '<Adjust>"
  "686be2de32be2d650746cf5e906439fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Adjust)))
  "Returns md5sum for a message object of type 'Adjust"
  "686be2de32be2d650746cf5e906439fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Adjust>)))
  "Returns full string definition for message of type '<Adjust>"
  (cl:format cl:nil "bool adjust~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Adjust)))
  "Returns full string definition for message of type 'Adjust"
  (cl:format cl:nil "bool adjust~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Adjust>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Adjust>))
  "Converts a ROS message object to a list"
  (cl:list 'Adjust
    (cl:cons ':adjust (adjust msg))
))
