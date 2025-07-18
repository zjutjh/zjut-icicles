; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-msg)


;//! \htmlinclude JoyState.msg.html

(cl:defclass <JoyState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass JoyState (<JoyState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoyState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoyState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-msg:<JoyState> is deprecated: use transbot_msgs-msg:JoyState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <JoyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:state-val is deprecated.  Use transbot_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoyState>) ostream)
  "Serializes a message object of type '<JoyState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoyState>) istream)
  "Deserializes a message object of type '<JoyState>"
    (cl:setf (cl:slot-value msg 'state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoyState>)))
  "Returns string type for a message object of type '<JoyState>"
  "transbot_msgs/JoyState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoyState)))
  "Returns string type for a message object of type 'JoyState"
  "transbot_msgs/JoyState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoyState>)))
  "Returns md5sum for a message object of type '<JoyState>"
  "001fde3cab9e313a150416ff09c08ee4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoyState)))
  "Returns md5sum for a message object of type 'JoyState"
  "001fde3cab9e313a150416ff09c08ee4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoyState>)))
  "Returns full string definition for message of type '<JoyState>"
  (cl:format cl:nil "bool state~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoyState)))
  "Returns full string definition for message of type 'JoyState"
  (cl:format cl:nil "bool state~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoyState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoyState>))
  "Converts a ROS message object to a list"
  (cl:list 'JoyState
    (cl:cons ':state (state msg))
))
