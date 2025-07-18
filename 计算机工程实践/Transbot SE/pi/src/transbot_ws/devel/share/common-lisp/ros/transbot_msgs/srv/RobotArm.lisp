; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-srv)


;//! \htmlinclude RobotArm-request.msg.html

(cl:defclass <RobotArm-request> (roslisp-msg-protocol:ros-message)
  ((apply
    :reader apply
    :initarg :apply
    :type cl:string
    :initform ""))
)

(cl:defclass RobotArm-request (<RobotArm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotArm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotArm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<RobotArm-request> is deprecated: use transbot_msgs-srv:RobotArm-request instead.")))

(cl:ensure-generic-function 'apply-val :lambda-list '(m))
(cl:defmethod apply-val ((m <RobotArm-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:apply-val is deprecated.  Use transbot_msgs-srv:apply instead.")
  (apply m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotArm-request>) ostream)
  "Serializes a message object of type '<RobotArm-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'apply))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'apply))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotArm-request>) istream)
  "Deserializes a message object of type '<RobotArm-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'apply) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'apply) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotArm-request>)))
  "Returns string type for a service object of type '<RobotArm-request>"
  "transbot_msgs/RobotArmRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotArm-request)))
  "Returns string type for a service object of type 'RobotArm-request"
  "transbot_msgs/RobotArmRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotArm-request>)))
  "Returns md5sum for a message object of type '<RobotArm-request>"
  "02b16e175f9698037e15289959eba75b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotArm-request)))
  "Returns md5sum for a message object of type 'RobotArm-request"
  "02b16e175f9698037e15289959eba75b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotArm-request>)))
  "Returns full string definition for message of type '<RobotArm-request>"
  (cl:format cl:nil "#request~%string apply~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotArm-request)))
  "Returns full string definition for message of type 'RobotArm-request"
  (cl:format cl:nil "#request~%string apply~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotArm-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'apply))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotArm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotArm-request
    (cl:cons ':apply (apply msg))
))
;//! \htmlinclude RobotArm-response.msg.html

(cl:defclass <RobotArm-response> (roslisp-msg-protocol:ros-message)
  ((RobotArm
    :reader RobotArm
    :initarg :RobotArm
    :type transbot_msgs-msg:Arm
    :initform (cl:make-instance 'transbot_msgs-msg:Arm)))
)

(cl:defclass RobotArm-response (<RobotArm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotArm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotArm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<RobotArm-response> is deprecated: use transbot_msgs-srv:RobotArm-response instead.")))

(cl:ensure-generic-function 'RobotArm-val :lambda-list '(m))
(cl:defmethod RobotArm-val ((m <RobotArm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:RobotArm-val is deprecated.  Use transbot_msgs-srv:RobotArm instead.")
  (RobotArm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotArm-response>) ostream)
  "Serializes a message object of type '<RobotArm-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'RobotArm) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotArm-response>) istream)
  "Deserializes a message object of type '<RobotArm-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'RobotArm) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotArm-response>)))
  "Returns string type for a service object of type '<RobotArm-response>"
  "transbot_msgs/RobotArmResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotArm-response)))
  "Returns string type for a service object of type 'RobotArm-response"
  "transbot_msgs/RobotArmResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotArm-response>)))
  "Returns md5sum for a message object of type '<RobotArm-response>"
  "02b16e175f9698037e15289959eba75b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotArm-response)))
  "Returns md5sum for a message object of type 'RobotArm-response"
  "02b16e175f9698037e15289959eba75b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotArm-response>)))
  "Returns full string definition for message of type '<RobotArm-response>"
  (cl:format cl:nil "#response~%Arm RobotArm~%~%~%================================================================================~%MSG: transbot_msgs/Arm~%Joint[] joint~%~%================================================================================~%MSG: transbot_msgs/Joint~%int32 id~%int32 run_time~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotArm-response)))
  "Returns full string definition for message of type 'RobotArm-response"
  (cl:format cl:nil "#response~%Arm RobotArm~%~%~%================================================================================~%MSG: transbot_msgs/Arm~%Joint[] joint~%~%================================================================================~%MSG: transbot_msgs/Joint~%int32 id~%int32 run_time~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotArm-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'RobotArm))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotArm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotArm-response
    (cl:cons ':RobotArm (RobotArm msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotArm)))
  'RobotArm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotArm)))
  'RobotArm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotArm)))
  "Returns string type for a service object of type '<RobotArm>"
  "transbot_msgs/RobotArm")