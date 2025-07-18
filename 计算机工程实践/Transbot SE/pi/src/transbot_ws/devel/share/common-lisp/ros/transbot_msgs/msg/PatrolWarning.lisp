; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-msg)


;//! \htmlinclude PatrolWarning.msg.html

(cl:defclass <PatrolWarning> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (Function
    :reader Function
    :initarg :Function
    :type cl:string
    :initform "")
   (ResponseDist
    :reader ResponseDist
    :initarg :ResponseDist
    :type cl:float
    :initform 0.0)
   (LaserAngle
    :reader LaserAngle
    :initarg :LaserAngle
    :type cl:integer
    :initform 0))
)

(cl:defclass PatrolWarning (<PatrolWarning>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PatrolWarning>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PatrolWarning)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-msg:<PatrolWarning> is deprecated: use transbot_msgs-msg:PatrolWarning instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <PatrolWarning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:speed-val is deprecated.  Use transbot_msgs-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'Function-val :lambda-list '(m))
(cl:defmethod Function-val ((m <PatrolWarning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:Function-val is deprecated.  Use transbot_msgs-msg:Function instead.")
  (Function m))

(cl:ensure-generic-function 'ResponseDist-val :lambda-list '(m))
(cl:defmethod ResponseDist-val ((m <PatrolWarning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:ResponseDist-val is deprecated.  Use transbot_msgs-msg:ResponseDist instead.")
  (ResponseDist m))

(cl:ensure-generic-function 'LaserAngle-val :lambda-list '(m))
(cl:defmethod LaserAngle-val ((m <PatrolWarning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:LaserAngle-val is deprecated.  Use transbot_msgs-msg:LaserAngle instead.")
  (LaserAngle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PatrolWarning>) ostream)
  "Serializes a message object of type '<PatrolWarning>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Function))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Function))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ResponseDist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'LaserAngle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PatrolWarning>) istream)
  "Deserializes a message object of type '<PatrolWarning>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Function) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Function) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ResponseDist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'LaserAngle) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PatrolWarning>)))
  "Returns string type for a message object of type '<PatrolWarning>"
  "transbot_msgs/PatrolWarning")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PatrolWarning)))
  "Returns string type for a message object of type 'PatrolWarning"
  "transbot_msgs/PatrolWarning")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PatrolWarning>)))
  "Returns md5sum for a message object of type '<PatrolWarning>"
  "3c2aedf4e9d9a1d5ce206d948829c9bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PatrolWarning)))
  "Returns md5sum for a message object of type 'PatrolWarning"
  "3c2aedf4e9d9a1d5ce206d948829c9bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PatrolWarning>)))
  "Returns full string definition for message of type '<PatrolWarning>"
  (cl:format cl:nil "float32 speed~%string Function~%float32 ResponseDist~%int32   LaserAngle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PatrolWarning)))
  "Returns full string definition for message of type 'PatrolWarning"
  (cl:format cl:nil "float32 speed~%string Function~%float32 ResponseDist~%int32   LaserAngle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PatrolWarning>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'Function))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PatrolWarning>))
  "Converts a ROS message object to a list"
  (cl:list 'PatrolWarning
    (cl:cons ':speed (speed msg))
    (cl:cons ':Function (Function msg))
    (cl:cons ':ResponseDist (ResponseDist msg))
    (cl:cons ':LaserAngle (LaserAngle msg))
))
