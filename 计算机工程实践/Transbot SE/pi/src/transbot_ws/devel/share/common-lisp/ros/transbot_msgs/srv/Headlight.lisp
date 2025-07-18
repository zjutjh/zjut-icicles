; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-srv)


;//! \htmlinclude Headlight-request.msg.html

(cl:defclass <Headlight-request> (roslisp-msg-protocol:ros-message)
  ((Headlight
    :reader Headlight
    :initarg :Headlight
    :type cl:integer
    :initform 0))
)

(cl:defclass Headlight-request (<Headlight-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Headlight-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Headlight-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<Headlight-request> is deprecated: use transbot_msgs-srv:Headlight-request instead.")))

(cl:ensure-generic-function 'Headlight-val :lambda-list '(m))
(cl:defmethod Headlight-val ((m <Headlight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:Headlight-val is deprecated.  Use transbot_msgs-srv:Headlight instead.")
  (Headlight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Headlight-request>) ostream)
  "Serializes a message object of type '<Headlight-request>"
  (cl:let* ((signed (cl:slot-value msg 'Headlight)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Headlight-request>) istream)
  "Deserializes a message object of type '<Headlight-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Headlight) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Headlight-request>)))
  "Returns string type for a service object of type '<Headlight-request>"
  "transbot_msgs/HeadlightRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Headlight-request)))
  "Returns string type for a service object of type 'Headlight-request"
  "transbot_msgs/HeadlightRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Headlight-request>)))
  "Returns md5sum for a message object of type '<Headlight-request>"
  "39cb7e9dbd56dfa74d38f52f3463c89d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Headlight-request)))
  "Returns md5sum for a message object of type 'Headlight-request"
  "39cb7e9dbd56dfa74d38f52f3463c89d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Headlight-request>)))
  "Returns full string definition for message of type '<Headlight-request>"
  (cl:format cl:nil "#request~%int32 Headlight~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Headlight-request)))
  "Returns full string definition for message of type 'Headlight-request"
  (cl:format cl:nil "#request~%int32 Headlight~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Headlight-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Headlight-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Headlight-request
    (cl:cons ':Headlight (Headlight msg))
))
;//! \htmlinclude Headlight-response.msg.html

(cl:defclass <Headlight-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Headlight-response (<Headlight-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Headlight-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Headlight-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<Headlight-response> is deprecated: use transbot_msgs-srv:Headlight-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Headlight-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:result-val is deprecated.  Use transbot_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Headlight-response>) ostream)
  "Serializes a message object of type '<Headlight-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Headlight-response>) istream)
  "Deserializes a message object of type '<Headlight-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Headlight-response>)))
  "Returns string type for a service object of type '<Headlight-response>"
  "transbot_msgs/HeadlightResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Headlight-response)))
  "Returns string type for a service object of type 'Headlight-response"
  "transbot_msgs/HeadlightResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Headlight-response>)))
  "Returns md5sum for a message object of type '<Headlight-response>"
  "39cb7e9dbd56dfa74d38f52f3463c89d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Headlight-response)))
  "Returns md5sum for a message object of type 'Headlight-response"
  "39cb7e9dbd56dfa74d38f52f3463c89d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Headlight-response>)))
  "Returns full string definition for message of type '<Headlight-response>"
  (cl:format cl:nil "#response~%bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Headlight-response)))
  "Returns full string definition for message of type 'Headlight-response"
  (cl:format cl:nil "#response~%bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Headlight-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Headlight-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Headlight-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Headlight)))
  'Headlight-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Headlight)))
  'Headlight-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Headlight)))
  "Returns string type for a service object of type '<Headlight>"
  "transbot_msgs/Headlight")