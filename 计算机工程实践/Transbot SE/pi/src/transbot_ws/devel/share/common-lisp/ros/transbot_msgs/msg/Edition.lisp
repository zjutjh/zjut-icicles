; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-msg)


;//! \htmlinclude Edition.msg.html

(cl:defclass <Edition> (roslisp-msg-protocol:ros-message)
  ((edition
    :reader edition
    :initarg :edition
    :type cl:float
    :initform 0.0))
)

(cl:defclass Edition (<Edition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Edition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Edition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-msg:<Edition> is deprecated: use transbot_msgs-msg:Edition instead.")))

(cl:ensure-generic-function 'edition-val :lambda-list '(m))
(cl:defmethod edition-val ((m <Edition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:edition-val is deprecated.  Use transbot_msgs-msg:edition instead.")
  (edition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Edition>) ostream)
  "Serializes a message object of type '<Edition>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'edition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Edition>) istream)
  "Deserializes a message object of type '<Edition>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'edition) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Edition>)))
  "Returns string type for a message object of type '<Edition>"
  "transbot_msgs/Edition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Edition)))
  "Returns string type for a message object of type 'Edition"
  "transbot_msgs/Edition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Edition>)))
  "Returns md5sum for a message object of type '<Edition>"
  "373df2b35ba40a1a8b8afa0bf078b756")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Edition)))
  "Returns md5sum for a message object of type 'Edition"
  "373df2b35ba40a1a8b8afa0bf078b756")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Edition>)))
  "Returns full string definition for message of type '<Edition>"
  (cl:format cl:nil "float32 edition~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Edition)))
  "Returns full string definition for message of type 'Edition"
  (cl:format cl:nil "float32 edition~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Edition>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Edition>))
  "Converts a ROS message object to a list"
  (cl:list 'Edition
    (cl:cons ':edition (edition msg))
))
