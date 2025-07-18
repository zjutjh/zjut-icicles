; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-srv)


;//! \htmlinclude RGBLight-request.msg.html

(cl:defclass <RGBLight-request> (roslisp-msg-protocol:ros-message)
  ((effect
    :reader effect
    :initarg :effect
    :type cl:integer
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0))
)

(cl:defclass RGBLight-request (<RGBLight-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RGBLight-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RGBLight-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<RGBLight-request> is deprecated: use transbot_msgs-srv:RGBLight-request instead.")))

(cl:ensure-generic-function 'effect-val :lambda-list '(m))
(cl:defmethod effect-val ((m <RGBLight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:effect-val is deprecated.  Use transbot_msgs-srv:effect instead.")
  (effect m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <RGBLight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:speed-val is deprecated.  Use transbot_msgs-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RGBLight-request>) ostream)
  "Serializes a message object of type '<RGBLight-request>"
  (cl:let* ((signed (cl:slot-value msg 'effect)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RGBLight-request>) istream)
  "Deserializes a message object of type '<RGBLight-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'effect) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RGBLight-request>)))
  "Returns string type for a service object of type '<RGBLight-request>"
  "transbot_msgs/RGBLightRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RGBLight-request)))
  "Returns string type for a service object of type 'RGBLight-request"
  "transbot_msgs/RGBLightRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RGBLight-request>)))
  "Returns md5sum for a message object of type '<RGBLight-request>"
  "4abffe91f7c3570fa771519fedadcf37")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RGBLight-request)))
  "Returns md5sum for a message object of type 'RGBLight-request"
  "4abffe91f7c3570fa771519fedadcf37")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RGBLight-request>)))
  "Returns full string definition for message of type '<RGBLight-request>"
  (cl:format cl:nil "#request~%int32 effect~%int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RGBLight-request)))
  "Returns full string definition for message of type 'RGBLight-request"
  (cl:format cl:nil "#request~%int32 effect~%int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RGBLight-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RGBLight-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RGBLight-request
    (cl:cons ':effect (effect msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude RGBLight-response.msg.html

(cl:defclass <RGBLight-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RGBLight-response (<RGBLight-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RGBLight-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RGBLight-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<RGBLight-response> is deprecated: use transbot_msgs-srv:RGBLight-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <RGBLight-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:result-val is deprecated.  Use transbot_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RGBLight-response>) ostream)
  "Serializes a message object of type '<RGBLight-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RGBLight-response>) istream)
  "Deserializes a message object of type '<RGBLight-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RGBLight-response>)))
  "Returns string type for a service object of type '<RGBLight-response>"
  "transbot_msgs/RGBLightResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RGBLight-response)))
  "Returns string type for a service object of type 'RGBLight-response"
  "transbot_msgs/RGBLightResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RGBLight-response>)))
  "Returns md5sum for a message object of type '<RGBLight-response>"
  "4abffe91f7c3570fa771519fedadcf37")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RGBLight-response)))
  "Returns md5sum for a message object of type 'RGBLight-response"
  "4abffe91f7c3570fa771519fedadcf37")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RGBLight-response>)))
  "Returns full string definition for message of type '<RGBLight-response>"
  (cl:format cl:nil "#response~%bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RGBLight-response)))
  "Returns full string definition for message of type 'RGBLight-response"
  (cl:format cl:nil "#response~%bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RGBLight-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RGBLight-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RGBLight-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RGBLight)))
  'RGBLight-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RGBLight)))
  'RGBLight-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RGBLight)))
  "Returns string type for a service object of type '<RGBLight>"
  "transbot_msgs/RGBLight")