; Auto-generated. Do not edit!


(cl:in-package yahboomcar_msgs-msg)


;//! \htmlinclude Target.msg.html

(cl:defclass <Target> (roslisp-msg-protocol:ros-message)
  ((frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform "")
   (stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (scores
    :reader scores
    :initarg :scores
    :type cl:float
    :initform 0.0)
   (ptx
    :reader ptx
    :initarg :ptx
    :type cl:float
    :initform 0.0)
   (pty
    :reader pty
    :initarg :pty
    :type cl:float
    :initform 0.0)
   (distw
    :reader distw
    :initarg :distw
    :type cl:float
    :initform 0.0)
   (disth
    :reader disth
    :initarg :disth
    :type cl:float
    :initform 0.0)
   (centerx
    :reader centerx
    :initarg :centerx
    :type cl:float
    :initform 0.0)
   (centery
    :reader centery
    :initarg :centery
    :type cl:float
    :initform 0.0))
)

(cl:defclass Target (<Target>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Target>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Target)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yahboomcar_msgs-msg:<Target> is deprecated: use yahboomcar_msgs-msg:Target instead.")))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:frame_id-val is deprecated.  Use yahboomcar_msgs-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:stamp-val is deprecated.  Use yahboomcar_msgs-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'scores-val :lambda-list '(m))
(cl:defmethod scores-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:scores-val is deprecated.  Use yahboomcar_msgs-msg:scores instead.")
  (scores m))

(cl:ensure-generic-function 'ptx-val :lambda-list '(m))
(cl:defmethod ptx-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:ptx-val is deprecated.  Use yahboomcar_msgs-msg:ptx instead.")
  (ptx m))

(cl:ensure-generic-function 'pty-val :lambda-list '(m))
(cl:defmethod pty-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:pty-val is deprecated.  Use yahboomcar_msgs-msg:pty instead.")
  (pty m))

(cl:ensure-generic-function 'distw-val :lambda-list '(m))
(cl:defmethod distw-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:distw-val is deprecated.  Use yahboomcar_msgs-msg:distw instead.")
  (distw m))

(cl:ensure-generic-function 'disth-val :lambda-list '(m))
(cl:defmethod disth-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:disth-val is deprecated.  Use yahboomcar_msgs-msg:disth instead.")
  (disth m))

(cl:ensure-generic-function 'centerx-val :lambda-list '(m))
(cl:defmethod centerx-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:centerx-val is deprecated.  Use yahboomcar_msgs-msg:centerx instead.")
  (centerx m))

(cl:ensure-generic-function 'centery-val :lambda-list '(m))
(cl:defmethod centery-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:centery-val is deprecated.  Use yahboomcar_msgs-msg:centery instead.")
  (centery m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Target>) ostream)
  "Serializes a message object of type '<Target>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'scores))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ptx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pty))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'disth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'centerx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'centery))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Target>) istream)
  "Deserializes a message object of type '<Target>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scores) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ptx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pty) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'disth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'centerx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'centery) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Target>)))
  "Returns string type for a message object of type '<Target>"
  "yahboomcar_msgs/Target")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Target)))
  "Returns string type for a message object of type 'Target"
  "yahboomcar_msgs/Target")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Target>)))
  "Returns md5sum for a message object of type '<Target>"
  "ed563213df8d0c63d0090b5a0b306f53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Target)))
  "Returns md5sum for a message object of type 'Target"
  "ed563213df8d0c63d0090b5a0b306f53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Target>)))
  "Returns full string definition for message of type '<Target>"
  (cl:format cl:nil "string frame_id~%time stamp~%float32 scores~%float32 ptx~%float32 pty~%float32 distw~%float32 disth~%float32 centerx~%float32 centery~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Target)))
  "Returns full string definition for message of type 'Target"
  (cl:format cl:nil "string frame_id~%time stamp~%float32 scores~%float32 ptx~%float32 pty~%float32 distw~%float32 disth~%float32 centerx~%float32 centery~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Target>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'frame_id))
     8
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Target>))
  "Converts a ROS message object to a list"
  (cl:list 'Target
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':scores (scores msg))
    (cl:cons ':ptx (ptx msg))
    (cl:cons ':pty (pty msg))
    (cl:cons ':distw (distw msg))
    (cl:cons ':disth (disth msg))
    (cl:cons ':centerx (centerx msg))
    (cl:cons ':centery (centery msg))
))
