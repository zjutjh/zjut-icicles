; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-srv)


;//! \htmlinclude Patrol-request.msg.html

(cl:defclass <Patrol-request> (roslisp-msg-protocol:ros-message)
  ((Commond
    :reader Commond
    :initarg :Commond
    :type cl:string
    :initform "")
   (LineScaling
    :reader LineScaling
    :initarg :LineScaling
    :type cl:float
    :initform 0.0)
   (RotationScaling
    :reader RotationScaling
    :initarg :RotationScaling
    :type cl:float
    :initform 0.0))
)

(cl:defclass Patrol-request (<Patrol-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Patrol-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Patrol-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<Patrol-request> is deprecated: use transbot_msgs-srv:Patrol-request instead.")))

(cl:ensure-generic-function 'Commond-val :lambda-list '(m))
(cl:defmethod Commond-val ((m <Patrol-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:Commond-val is deprecated.  Use transbot_msgs-srv:Commond instead.")
  (Commond m))

(cl:ensure-generic-function 'LineScaling-val :lambda-list '(m))
(cl:defmethod LineScaling-val ((m <Patrol-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:LineScaling-val is deprecated.  Use transbot_msgs-srv:LineScaling instead.")
  (LineScaling m))

(cl:ensure-generic-function 'RotationScaling-val :lambda-list '(m))
(cl:defmethod RotationScaling-val ((m <Patrol-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:RotationScaling-val is deprecated.  Use transbot_msgs-srv:RotationScaling instead.")
  (RotationScaling m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Patrol-request>) ostream)
  "Serializes a message object of type '<Patrol-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Commond))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Commond))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'LineScaling))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'RotationScaling))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Patrol-request>) istream)
  "Deserializes a message object of type '<Patrol-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Commond) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Commond) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LineScaling) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RotationScaling) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Patrol-request>)))
  "Returns string type for a service object of type '<Patrol-request>"
  "transbot_msgs/PatrolRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Patrol-request)))
  "Returns string type for a service object of type 'Patrol-request"
  "transbot_msgs/PatrolRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Patrol-request>)))
  "Returns md5sum for a message object of type '<Patrol-request>"
  "725a414bc8766f0cf2b2c0b5f17047e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Patrol-request)))
  "Returns md5sum for a message object of type 'Patrol-request"
  "725a414bc8766f0cf2b2c0b5f17047e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Patrol-request>)))
  "Returns full string definition for message of type '<Patrol-request>"
  (cl:format cl:nil "#request~%string  Commond~%float32 LineScaling~%float32 RotationScaling~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Patrol-request)))
  "Returns full string definition for message of type 'Patrol-request"
  (cl:format cl:nil "#request~%string  Commond~%float32 LineScaling~%float32 RotationScaling~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Patrol-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Commond))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Patrol-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Patrol-request
    (cl:cons ':Commond (Commond msg))
    (cl:cons ':LineScaling (LineScaling msg))
    (cl:cons ':RotationScaling (RotationScaling msg))
))
;//! \htmlinclude Patrol-response.msg.html

(cl:defclass <Patrol-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Patrol-response (<Patrol-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Patrol-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Patrol-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-srv:<Patrol-response> is deprecated: use transbot_msgs-srv:Patrol-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Patrol-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-srv:result-val is deprecated.  Use transbot_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Patrol-response>) ostream)
  "Serializes a message object of type '<Patrol-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Patrol-response>) istream)
  "Deserializes a message object of type '<Patrol-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Patrol-response>)))
  "Returns string type for a service object of type '<Patrol-response>"
  "transbot_msgs/PatrolResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Patrol-response)))
  "Returns string type for a service object of type 'Patrol-response"
  "transbot_msgs/PatrolResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Patrol-response>)))
  "Returns md5sum for a message object of type '<Patrol-response>"
  "725a414bc8766f0cf2b2c0b5f17047e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Patrol-response)))
  "Returns md5sum for a message object of type 'Patrol-response"
  "725a414bc8766f0cf2b2c0b5f17047e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Patrol-response>)))
  "Returns full string definition for message of type '<Patrol-response>"
  (cl:format cl:nil "#response~%bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Patrol-response)))
  "Returns full string definition for message of type 'Patrol-response"
  (cl:format cl:nil "#response~%bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Patrol-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Patrol-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Patrol-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Patrol)))
  'Patrol-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Patrol)))
  'Patrol-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Patrol)))
  "Returns string type for a service object of type '<Patrol>"
  "transbot_msgs/Patrol")