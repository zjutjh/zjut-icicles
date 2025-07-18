; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-msg)


;//! \htmlinclude General.msg.html

(cl:defclass <General> (roslisp-msg-protocol:ros-message)
  ((Graphics
    :reader Graphics
    :initarg :Graphics
    :type cl:string
    :initform "")
   (TrackState
    :reader TrackState
    :initarg :TrackState
    :type cl:string
    :initform ""))
)

(cl:defclass General (<General>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <General>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'General)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-msg:<General> is deprecated: use transbot_msgs-msg:General instead.")))

(cl:ensure-generic-function 'Graphics-val :lambda-list '(m))
(cl:defmethod Graphics-val ((m <General>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:Graphics-val is deprecated.  Use transbot_msgs-msg:Graphics instead.")
  (Graphics m))

(cl:ensure-generic-function 'TrackState-val :lambda-list '(m))
(cl:defmethod TrackState-val ((m <General>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:TrackState-val is deprecated.  Use transbot_msgs-msg:TrackState instead.")
  (TrackState m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <General>) ostream)
  "Serializes a message object of type '<General>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Graphics))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Graphics))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'TrackState))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'TrackState))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <General>) istream)
  "Deserializes a message object of type '<General>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Graphics) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Graphics) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'TrackState) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'TrackState) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<General>)))
  "Returns string type for a message object of type '<General>"
  "transbot_msgs/General")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'General)))
  "Returns string type for a message object of type 'General"
  "transbot_msgs/General")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<General>)))
  "Returns md5sum for a message object of type '<General>"
  "17ceb36a2d5cf93a882109ffc0506c61")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'General)))
  "Returns md5sum for a message object of type 'General"
  "17ceb36a2d5cf93a882109ffc0506c61")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<General>)))
  "Returns full string definition for message of type '<General>"
  (cl:format cl:nil "string Graphics~%string TrackState~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'General)))
  "Returns full string definition for message of type 'General"
  (cl:format cl:nil "string Graphics~%string TrackState~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <General>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Graphics))
     4 (cl:length (cl:slot-value msg 'TrackState))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <General>))
  "Converts a ROS message object to a list"
  (cl:list 'General
    (cl:cons ':Graphics (Graphics msg))
    (cl:cons ':TrackState (TrackState msg))
))
