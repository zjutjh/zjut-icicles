; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-srv)


;//! \htmlinclude CamDevice-request.msg.html

(cl:defclass <CamDevice-request> (roslisp-msg-protocol:ros-message)
  ((GetGev
    :reader GetGev
    :initarg :GetGev
    :type cl:string
    :initform ""))
)

(cl:defclass CamDevice-request (<CamDevice-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CamDevice-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CamDevice-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<CamDevice-request> is deprecated: use transbot_msgs-srv:CamDevice-request instead.")))

(cl:ensure-generic-function 'GetGev-val :lambda-list '(m))
(cl:defmethod GetGev-val ((m <CamDevice-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:GetGev-val is deprecated.  Use transbot_msgs-srv:GetGev instead.")
  (GetGev m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CamDevice-request>) ostream)
  "Serializes a message object of type '<CamDevice-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'GetGev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'GetGev))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CamDevice-request>) istream)
  "Deserializes a message object of type '<CamDevice-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'GetGev) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'GetGev) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CamDevice-request>)))
  "Returns string type for a service object of type '<CamDevice-request>"
  "transbot_msgs/CamDeviceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CamDevice-request)))
  "Returns string type for a service object of type 'CamDevice-request"
  "transbot_msgs/CamDeviceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CamDevice-request>)))
  "Returns md5sum for a message object of type '<CamDevice-request>"
  "8be1511d89aeca50b4c34cbc069c61f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CamDevice-request)))
  "Returns md5sum for a message object of type 'CamDevice-request"
  "8be1511d89aeca50b4c34cbc069c61f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CamDevice-request>)))
  "Returns full string definition for message of type '<CamDevice-request>"
  (cl:format cl:nil "#request~%string GetGev~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CamDevice-request)))
  "Returns full string definition for message of type 'CamDevice-request"
  (cl:format cl:nil "#request~%string GetGev~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CamDevice-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'GetGev))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CamDevice-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CamDevice-request
    (cl:cons ':GetGev (GetGev msg))
))
;//! \htmlinclude CamDevice-response.msg.html

(cl:defclass <CamDevice-response> (roslisp-msg-protocol:ros-message)
  ((camDevice
    :reader camDevice
    :initarg :camDevice
    :type cl:string
    :initform ""))
)

(cl:defclass CamDevice-response (<CamDevice-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CamDevice-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CamDevice-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<CamDevice-response> is deprecated: use transbot_msgs-srv:CamDevice-response instead.")))

(cl:ensure-generic-function 'camDevice-val :lambda-list '(m))
(cl:defmethod camDevice-val ((m <CamDevice-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:camDevice-val is deprecated.  Use transbot_msgs-srv:camDevice instead.")
  (camDevice m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CamDevice-response>) ostream)
  "Serializes a message object of type '<CamDevice-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'camDevice))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'camDevice))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CamDevice-response>) istream)
  "Deserializes a message object of type '<CamDevice-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camDevice) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'camDevice) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CamDevice-response>)))
  "Returns string type for a service object of type '<CamDevice-response>"
  "transbot_msgs/CamDeviceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CamDevice-response)))
  "Returns string type for a service object of type 'CamDevice-response"
  "transbot_msgs/CamDeviceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CamDevice-response>)))
  "Returns md5sum for a message object of type '<CamDevice-response>"
  "8be1511d89aeca50b4c34cbc069c61f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CamDevice-response)))
  "Returns md5sum for a message object of type 'CamDevice-response"
  "8be1511d89aeca50b4c34cbc069c61f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CamDevice-response>)))
  "Returns full string definition for message of type '<CamDevice-response>"
  (cl:format cl:nil "#response~%string camDevice~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CamDevice-response)))
  "Returns full string definition for message of type 'CamDevice-response"
  (cl:format cl:nil "#response~%string camDevice~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CamDevice-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'camDevice))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CamDevice-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CamDevice-response
    (cl:cons ':camDevice (camDevice msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CamDevice)))
  'CamDevice-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CamDevice)))
  'CamDevice-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CamDevice)))
  "Returns string type for a service object of type '<CamDevice>"
  "transbot_msgs/CamDevice")