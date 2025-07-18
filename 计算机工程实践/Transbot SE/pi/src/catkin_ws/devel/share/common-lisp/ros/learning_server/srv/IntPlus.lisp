; Auto-generated. Do not edit!


(cl:in-package learning_server-srv)


;//! \htmlinclude IntPlus-request.msg.html

(cl:defclass <IntPlus-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:fixnum
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:fixnum
    :initform 0))
)

(cl:defclass IntPlus-request (<IntPlus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IntPlus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IntPlus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_server-srv:<IntPlus-request> is deprecated: use learning_server-srv:IntPlus-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <IntPlus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader learning_server-srv:a-val is deprecated.  Use learning_server-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <IntPlus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader learning_server-srv:b-val is deprecated.  Use learning_server-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IntPlus-request>) ostream)
  "Serializes a message object of type '<IntPlus-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IntPlus-request>) istream)
  "Deserializes a message object of type '<IntPlus-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IntPlus-request>)))
  "Returns string type for a service object of type '<IntPlus-request>"
  "learning_server/IntPlusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntPlus-request)))
  "Returns string type for a service object of type 'IntPlus-request"
  "learning_server/IntPlusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IntPlus-request>)))
  "Returns md5sum for a message object of type '<IntPlus-request>"
  "32f3e69fccd784eebfbfc5b37be0e2f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IntPlus-request)))
  "Returns md5sum for a message object of type 'IntPlus-request"
  "32f3e69fccd784eebfbfc5b37be0e2f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IntPlus-request>)))
  "Returns full string definition for message of type '<IntPlus-request>"
  (cl:format cl:nil "uint8  a~%uint8  b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IntPlus-request)))
  "Returns full string definition for message of type 'IntPlus-request"
  (cl:format cl:nil "uint8  a~%uint8  b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IntPlus-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IntPlus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IntPlus-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude IntPlus-response.msg.html

(cl:defclass <IntPlus-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass IntPlus-response (<IntPlus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IntPlus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IntPlus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_server-srv:<IntPlus-response> is deprecated: use learning_server-srv:IntPlus-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <IntPlus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader learning_server-srv:result-val is deprecated.  Use learning_server-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IntPlus-response>) ostream)
  "Serializes a message object of type '<IntPlus-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IntPlus-response>) istream)
  "Deserializes a message object of type '<IntPlus-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IntPlus-response>)))
  "Returns string type for a service object of type '<IntPlus-response>"
  "learning_server/IntPlusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntPlus-response)))
  "Returns string type for a service object of type 'IntPlus-response"
  "learning_server/IntPlusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IntPlus-response>)))
  "Returns md5sum for a message object of type '<IntPlus-response>"
  "32f3e69fccd784eebfbfc5b37be0e2f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IntPlus-response)))
  "Returns md5sum for a message object of type 'IntPlus-response"
  "32f3e69fccd784eebfbfc5b37be0e2f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IntPlus-response>)))
  "Returns full string definition for message of type '<IntPlus-response>"
  (cl:format cl:nil "uint8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IntPlus-response)))
  "Returns full string definition for message of type 'IntPlus-response"
  (cl:format cl:nil "uint8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IntPlus-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IntPlus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IntPlus-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IntPlus)))
  'IntPlus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IntPlus)))
  'IntPlus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntPlus)))
  "Returns string type for a service object of type '<IntPlus>"
  "learning_server/IntPlus")