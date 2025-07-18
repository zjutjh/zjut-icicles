; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude Face.msg.html

(cl:defclass <Face> (roslisp-msg-protocol:ros-message)
  ((face
    :reader face
    :initarg :face
    :type opencv_apps-msg:Rect
    :initform (cl:make-instance 'opencv_apps-msg:Rect))
   (eyes
    :reader eyes
    :initarg :eyes
    :type (cl:vector opencv_apps-msg:Rect)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Rect :initial-element (cl:make-instance 'opencv_apps-msg:Rect)))
   (label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass Face (<Face>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Face>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Face)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<Face> is deprecated: use opencv_apps-msg:Face instead.")))

(cl:ensure-generic-function 'face-val :lambda-list '(m))
(cl:defmethod face-val ((m <Face>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:face-val is deprecated.  Use opencv_apps-msg:face instead.")
  (face m))

(cl:ensure-generic-function 'eyes-val :lambda-list '(m))
(cl:defmethod eyes-val ((m <Face>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:eyes-val is deprecated.  Use opencv_apps-msg:eyes instead.")
  (eyes m))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <Face>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:label-val is deprecated.  Use opencv_apps-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <Face>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:confidence-val is deprecated.  Use opencv_apps-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Face>) ostream)
  "Serializes a message object of type '<Face>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'face) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'eyes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'eyes))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Face>) istream)
  "Deserializes a message object of type '<Face>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'face) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'eyes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'eyes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Rect))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Face>)))
  "Returns string type for a message object of type '<Face>"
  "opencv_apps/Face")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Face)))
  "Returns string type for a message object of type 'Face"
  "opencv_apps/Face")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Face>)))
  "Returns md5sum for a message object of type '<Face>"
  "a1a50e747b0ca7822ce8611c3ffa7a02")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Face)))
  "Returns md5sum for a message object of type 'Face"
  "a1a50e747b0ca7822ce8611c3ffa7a02")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Face>)))
  "Returns full string definition for message of type '<Face>"
  (cl:format cl:nil "Rect face~%Rect[] eyes~%string label~%float64 confidence~%~%================================================================================~%MSG: opencv_apps/Rect~%# opencv Rect data type, x-y is center point~%float64 x~%float64 y~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Face)))
  "Returns full string definition for message of type 'Face"
  (cl:format cl:nil "Rect face~%Rect[] eyes~%string label~%float64 confidence~%~%================================================================================~%MSG: opencv_apps/Rect~%# opencv Rect data type, x-y is center point~%float64 x~%float64 y~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Face>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'face))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'eyes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:length (cl:slot-value msg 'label))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Face>))
  "Converts a ROS message object to a list"
  (cl:list 'Face
    (cl:cons ':face (face msg))
    (cl:cons ':eyes (eyes msg))
    (cl:cons ':label (label msg))
    (cl:cons ':confidence (confidence msg))
))
