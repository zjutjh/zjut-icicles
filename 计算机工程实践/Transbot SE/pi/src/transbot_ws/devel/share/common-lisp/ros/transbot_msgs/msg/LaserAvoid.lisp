; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-msg)


;//! \htmlinclude LaserAvoid.msg.html

(cl:defclass <LaserAvoid> (roslisp-msg-protocol:ros-message)
  ((Angle_range
    :reader Angle_range
    :initarg :Angle_range
    :type cl:integer
    :initform 0)
   (ResponseDist
    :reader ResponseDist
    :initarg :ResponseDist
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass LaserAvoid (<LaserAvoid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LaserAvoid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LaserAvoid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-msg:<LaserAvoid> is deprecated: use transbot_msgs-msg:LaserAvoid instead.")))

(cl:ensure-generic-function 'Angle_range-val :lambda-list '(m))
(cl:defmethod Angle_range-val ((m <LaserAvoid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:Angle_range-val is deprecated.  Use transbot_msgs-msg:Angle_range instead.")
  (Angle_range m))

(cl:ensure-generic-function 'ResponseDist-val :lambda-list '(m))
(cl:defmethod ResponseDist-val ((m <LaserAvoid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:ResponseDist-val is deprecated.  Use transbot_msgs-msg:ResponseDist instead.")
  (ResponseDist m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <LaserAvoid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:speed-val is deprecated.  Use transbot_msgs-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LaserAvoid>) ostream)
  "Serializes a message object of type '<LaserAvoid>"
  (cl:let* ((signed (cl:slot-value msg 'Angle_range)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ResponseDist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LaserAvoid>) istream)
  "Deserializes a message object of type '<LaserAvoid>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Angle_range) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ResponseDist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LaserAvoid>)))
  "Returns string type for a message object of type '<LaserAvoid>"
  "transbot_msgs/LaserAvoid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LaserAvoid)))
  "Returns string type for a message object of type 'LaserAvoid"
  "transbot_msgs/LaserAvoid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LaserAvoid>)))
  "Returns md5sum for a message object of type '<LaserAvoid>"
  "6872d2ec650238739f9b3c8aab8ed9b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LaserAvoid)))
  "Returns md5sum for a message object of type 'LaserAvoid"
  "6872d2ec650238739f9b3c8aab8ed9b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LaserAvoid>)))
  "Returns full string definition for message of type '<LaserAvoid>"
  (cl:format cl:nil "int32 Angle_range~%float32 ResponseDist~%float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LaserAvoid)))
  "Returns full string definition for message of type 'LaserAvoid"
  (cl:format cl:nil "int32 Angle_range~%float32 ResponseDist~%float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LaserAvoid>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LaserAvoid>))
  "Converts a ROS message object to a list"
  (cl:list 'LaserAvoid
    (cl:cons ':Angle_range (Angle_range msg))
    (cl:cons ':ResponseDist (ResponseDist msg))
    (cl:cons ':speed (speed msg))
))
