; Auto-generated. Do not edit!


(cl:in-package view_detect-msg)


;//! \htmlinclude YoloResult.msg.html

(cl:defclass <YoloResult> (roslisp-msg-protocol:ros-message)
  ((image_id
    :reader image_id
    :initarg :image_id
    :type cl:string
    :initform "")
   (class_names
    :reader class_names
    :initarg :class_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (counts
    :reader counts
    :initarg :counts
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass YoloResult (<YoloResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YoloResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YoloResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name view_detect-msg:<YoloResult> is deprecated: use view_detect-msg:YoloResult instead.")))

(cl:ensure-generic-function 'image_id-val :lambda-list '(m))
(cl:defmethod image_id-val ((m <YoloResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader view_detect-msg:image_id-val is deprecated.  Use view_detect-msg:image_id instead.")
  (image_id m))

(cl:ensure-generic-function 'class_names-val :lambda-list '(m))
(cl:defmethod class_names-val ((m <YoloResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader view_detect-msg:class_names-val is deprecated.  Use view_detect-msg:class_names instead.")
  (class_names m))

(cl:ensure-generic-function 'counts-val :lambda-list '(m))
(cl:defmethod counts-val ((m <YoloResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader view_detect-msg:counts-val is deprecated.  Use view_detect-msg:counts instead.")
  (counts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YoloResult>) ostream)
  "Serializes a message object of type '<YoloResult>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'image_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'image_id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'class_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'class_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'counts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'counts))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YoloResult>) istream)
  "Deserializes a message object of type '<YoloResult>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'image_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'image_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'class_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'class_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'counts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'counts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YoloResult>)))
  "Returns string type for a message object of type '<YoloResult>"
  "view_detect/YoloResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YoloResult)))
  "Returns string type for a message object of type 'YoloResult"
  "view_detect/YoloResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YoloResult>)))
  "Returns md5sum for a message object of type '<YoloResult>"
  "52917fd0d63c30fa5cbba61ad8a19792")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YoloResult)))
  "Returns md5sum for a message object of type 'YoloResult"
  "52917fd0d63c30fa5cbba61ad8a19792")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YoloResult>)))
  "Returns full string definition for message of type '<YoloResult>"
  (cl:format cl:nil "# YOLO检测结果消息~%string image_id        # 图像ID或文件名~%string[] class_names   # 检测到的类别名称~%int32[] counts         # 每个类别的数量 ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YoloResult)))
  "Returns full string definition for message of type 'YoloResult"
  (cl:format cl:nil "# YOLO检测结果消息~%string image_id        # 图像ID或文件名~%string[] class_names   # 检测到的类别名称~%int32[] counts         # 每个类别的数量 ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YoloResult>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'image_id))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'class_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'counts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YoloResult>))
  "Converts a ROS message object to a list"
  (cl:list 'YoloResult
    (cl:cons ':image_id (image_id msg))
    (cl:cons ':class_names (class_names msg))
    (cl:cons ':counts (counts msg))
))
