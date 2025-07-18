; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-srv)


;//! \htmlinclude Buzzer-request.msg.html

(cl:defclass <Buzzer-request> (roslisp-msg-protocol:ros-message)
  ((buzzer
    :reader buzzer
    :initarg :buzzer
    :type cl:integer
    :initform 0))
)

(cl:defclass Buzzer-request (<Buzzer-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Buzzer-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Buzzer-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<Buzzer-request> is deprecated: use transbot_msgs-srv:Buzzer-request instead.")))

(cl:ensure-generic-function 'buzzer-val :lambda-list '(m))
(cl:defmethod buzzer-val ((m <Buzzer-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:buzzer-val is deprecated.  Use transbot_msgs-srv:buzzer instead.")
  (buzzer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Buzzer-request>) ostream)
  "Serializes a message object of type '<Buzzer-request>"
  (cl:let* ((signed (cl:slot-value msg 'buzzer)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Buzzer-request>) istream)
  "Deserializes a message object of type '<Buzzer-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'buzzer) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Buzzer-request>)))
  "Returns string type for a service object of type '<Buzzer-request>"
  "transbot_msgs/BuzzerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Buzzer-request)))
  "Returns string type for a service object of type 'Buzzer-request"
  "transbot_msgs/BuzzerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Buzzer-request>)))
  "Returns md5sum for a message object of type '<Buzzer-request>"
  "32ecc8168750cdefd185aff218d2ce5e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Buzzer-request)))
  "Returns md5sum for a message object of type 'Buzzer-request"
  "32ecc8168750cdefd185aff218d2ce5e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Buzzer-request>)))
  "Returns full string definition for message of type '<Buzzer-request>"
  (cl:format cl:nil "#request~%int32 buzzer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Buzzer-request)))
  "Returns full string definition for message of type 'Buzzer-request"
  (cl:format cl:nil "#request~%int32 buzzer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Buzzer-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Buzzer-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Buzzer-request
    (cl:cons ':buzzer (buzzer msg))
))
;//! \htmlinclude Buzzer-response.msg.html

(cl:defclass <Buzzer-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Buzzer-response (<Buzzer-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Buzzer-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Buzzer-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<Buzzer-response> is deprecated: use transbot_msgs-srv:Buzzer-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Buzzer-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:result-val is deprecated.  Use transbot_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Buzzer-response>) ostream)
  "Serializes a message object of type '<Buzzer-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Buzzer-response>) istream)
  "Deserializes a message object of type '<Buzzer-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Buzzer-response>)))
  "Returns string type for a service object of type '<Buzzer-response>"
  "transbot_msgs/BuzzerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Buzzer-response)))
  "Returns string type for a service object of type 'Buzzer-response"
  "transbot_msgs/BuzzerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Buzzer-response>)))
  "Returns md5sum for a message object of type '<Buzzer-response>"
  "32ecc8168750cdefd185aff218d2ce5e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Buzzer-response)))
  "Returns md5sum for a message object of type 'Buzzer-response"
  "32ecc8168750cdefd185aff218d2ce5e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Buzzer-response>)))
  "Returns full string definition for message of type '<Buzzer-response>"
  (cl:format cl:nil "#response~%bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Buzzer-response)))
  "Returns full string definition for message of type 'Buzzer-response"
  (cl:format cl:nil "#response~%bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Buzzer-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Buzzer-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Buzzer-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Buzzer)))
  'Buzzer-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Buzzer)))
  'Buzzer-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Buzzer)))
  "Returns string type for a service object of type '<Buzzer>"
  "transbot_msgs/Buzzer")