; Auto-generated. Do not edit!


(cl:in-package yahboomcar_msgs-msg)


;//! \htmlinclude Image_Msg.msg.html

(cl:defclass <Image_Msg> (roslisp-msg-protocol:ros-message)
  ((height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (channels
    :reader channels
    :initarg :channels
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Image_Msg (<Image_Msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Image_Msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Image_Msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yahboomcar_msgs-msg:<Image_Msg> is deprecated: use yahboomcar_msgs-msg:Image_Msg instead.")))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <Image_Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:height-val is deprecated.  Use yahboomcar_msgs-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <Image_Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:width-val is deprecated.  Use yahboomcar_msgs-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'channels-val :lambda-list '(m))
(cl:defmethod channels-val ((m <Image_Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:channels-val is deprecated.  Use yahboomcar_msgs-msg:channels instead.")
  (channels m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Image_Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:data-val is deprecated.  Use yahboomcar_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Image_Msg>) ostream)
  "Serializes a message object of type '<Image_Msg>"
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'channels)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Image_Msg>) istream)
  "Deserializes a message object of type '<Image_Msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'channels) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Image_Msg>)))
  "Returns string type for a message object of type '<Image_Msg>"
  "yahboomcar_msgs/Image_Msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Image_Msg)))
  "Returns string type for a message object of type 'Image_Msg"
  "yahboomcar_msgs/Image_Msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Image_Msg>)))
  "Returns md5sum for a message object of type '<Image_Msg>"
  "33648d31a5c2999db4993f94515b68ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Image_Msg)))
  "Returns md5sum for a message object of type 'Image_Msg"
  "33648d31a5c2999db4993f94515b68ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Image_Msg>)))
  "Returns full string definition for message of type '<Image_Msg>"
  (cl:format cl:nil "int32 height~%int32 width~%int32 channels~%uint8[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Image_Msg)))
  "Returns full string definition for message of type 'Image_Msg"
  (cl:format cl:nil "int32 height~%int32 width~%int32 channels~%uint8[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Image_Msg>))
  (cl:+ 0
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Image_Msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Image_Msg
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':channels (channels msg))
    (cl:cons ':data (data msg))
))
