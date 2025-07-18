; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-msg)


;//! \htmlinclude Battery.msg.html

(cl:defclass <Battery> (roslisp-msg-protocol:ros-message)
  ((Voltage
    :reader Voltage
    :initarg :Voltage
    :type cl:float
    :initform 0.0))
)

(cl:defclass Battery (<Battery>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Battery>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Battery)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-msg:<Battery> is deprecated: use transbot_msgs-msg:Battery instead.")))

(cl:ensure-generic-function 'Voltage-val :lambda-list '(m))
(cl:defmethod Voltage-val ((m <Battery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:Voltage-val is deprecated.  Use transbot_msgs-msg:Voltage instead.")
  (Voltage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Battery>) ostream)
  "Serializes a message object of type '<Battery>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Battery>) istream)
  "Deserializes a message object of type '<Battery>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Voltage) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Battery>)))
  "Returns string type for a message object of type '<Battery>"
  "transbot_msgs/Battery")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Battery)))
  "Returns string type for a message object of type 'Battery"
  "transbot_msgs/Battery")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Battery>)))
  "Returns md5sum for a message object of type '<Battery>"
  "cb42914b7d362060569576ceed0643ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Battery)))
  "Returns md5sum for a message object of type 'Battery"
  "cb42914b7d362060569576ceed0643ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Battery>)))
  "Returns full string definition for message of type '<Battery>"
  (cl:format cl:nil "float32 Voltage~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Battery)))
  "Returns full string definition for message of type 'Battery"
  (cl:format cl:nil "float32 Voltage~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Battery>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Battery>))
  "Converts a ROS message object to a list"
  (cl:list 'Battery
    (cl:cons ':Voltage (Voltage msg))
))
