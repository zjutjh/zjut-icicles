; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-msg)


;//! \htmlinclude PWMServo.msg.html

(cl:defclass <PWMServo> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:integer
    :initform 0))
)

(cl:defclass PWMServo (<PWMServo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PWMServo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PWMServo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-msg:<PWMServo> is deprecated: use transbot_msgs-msg:PWMServo instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <PWMServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:id-val is deprecated.  Use transbot_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <PWMServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:angle-val is deprecated.  Use transbot_msgs-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PWMServo>) ostream)
  "Serializes a message object of type '<PWMServo>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PWMServo>) istream)
  "Deserializes a message object of type '<PWMServo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PWMServo>)))
  "Returns string type for a message object of type '<PWMServo>"
  "transbot_msgs/PWMServo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PWMServo)))
  "Returns string type for a message object of type 'PWMServo"
  "transbot_msgs/PWMServo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PWMServo>)))
  "Returns md5sum for a message object of type '<PWMServo>"
  "c5a368d31c65388d88289de5a105a271")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PWMServo)))
  "Returns md5sum for a message object of type 'PWMServo"
  "c5a368d31c65388d88289de5a105a271")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PWMServo>)))
  "Returns full string definition for message of type '<PWMServo>"
  (cl:format cl:nil "int32 id~%int32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PWMServo)))
  "Returns full string definition for message of type 'PWMServo"
  (cl:format cl:nil "int32 id~%int32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PWMServo>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PWMServo>))
  "Converts a ROS message object to a list"
  (cl:list 'PWMServo
    (cl:cons ':id (id msg))
    (cl:cons ':angle (angle msg))
))
