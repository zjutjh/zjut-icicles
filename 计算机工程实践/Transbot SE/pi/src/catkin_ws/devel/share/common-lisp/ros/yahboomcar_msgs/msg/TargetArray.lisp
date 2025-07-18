; Auto-generated. Do not edit!


(cl:in-package yahboomcar_msgs-msg)


;//! \htmlinclude TargetArray.msg.html

(cl:defclass <TargetArray> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector yahboomcar_msgs-msg:Target)
   :initform (cl:make-array 0 :element-type 'yahboomcar_msgs-msg:Target :initial-element (cl:make-instance 'yahboomcar_msgs-msg:Target))))
)

(cl:defclass TargetArray (<TargetArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yahboomcar_msgs-msg:<TargetArray> is deprecated: use yahboomcar_msgs-msg:TargetArray instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <TargetArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-msg:data-val is deprecated.  Use yahboomcar_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetArray>) ostream)
  "Serializes a message object of type '<TargetArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetArray>) istream)
  "Deserializes a message object of type '<TargetArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'yahboomcar_msgs-msg:Target))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetArray>)))
  "Returns string type for a message object of type '<TargetArray>"
  "yahboomcar_msgs/TargetArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetArray)))
  "Returns string type for a message object of type 'TargetArray"
  "yahboomcar_msgs/TargetArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetArray>)))
  "Returns md5sum for a message object of type '<TargetArray>"
  "5c74c52b301e4b25679f3941c4f6f23f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetArray)))
  "Returns md5sum for a message object of type 'TargetArray"
  "5c74c52b301e4b25679f3941c4f6f23f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetArray>)))
  "Returns full string definition for message of type '<TargetArray>"
  (cl:format cl:nil "yahboomcar_msgs/Target[] data~%~%================================================================================~%MSG: yahboomcar_msgs/Target~%string frame_id~%time stamp~%float32 scores~%float32 ptx~%float32 pty~%float32 distw~%float32 disth~%float32 centerx~%float32 centery~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetArray)))
  "Returns full string definition for message of type 'TargetArray"
  (cl:format cl:nil "yahboomcar_msgs/Target[] data~%~%================================================================================~%MSG: yahboomcar_msgs/Target~%string frame_id~%time stamp~%float32 scores~%float32 ptx~%float32 pty~%float32 distw~%float32 disth~%float32 centerx~%float32 centery~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetArray>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetArray
    (cl:cons ':data (data msg))
))
