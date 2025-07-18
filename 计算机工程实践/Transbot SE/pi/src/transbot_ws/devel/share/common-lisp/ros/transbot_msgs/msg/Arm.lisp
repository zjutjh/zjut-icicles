; Auto-generated. Do not edit!


(cl:in-package transbot_msgs-msg)


;//! \htmlinclude Arm.msg.html

(cl:defclass <Arm> (roslisp-msg-protocol:ros-message)
  ((joint
    :reader joint
    :initarg :joint
    :type (cl:vector transbot_msgs-msg:Joint)
   :initform (cl:make-array 0 :element-type 'transbot_msgs-msg:Joint :initial-element (cl:make-instance 'transbot_msgs-msg:Joint))))
)

(cl:defclass Arm (<Arm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Arm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Arm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transbot_msgs-msg:<Arm> is deprecated: use transbot_msgs-msg:Arm instead.")))

(cl:ensure-generic-function 'joint-val :lambda-list '(m))
(cl:defmethod joint-val ((m <Arm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transbot_msgs-msg:joint-val is deprecated.  Use transbot_msgs-msg:joint instead.")
  (joint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Arm>) ostream)
  "Serializes a message object of type '<Arm>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'joint))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Arm>) istream)
  "Deserializes a message object of type '<Arm>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'transbot_msgs-msg:Joint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Arm>)))
  "Returns string type for a message object of type '<Arm>"
  "transbot_msgs/Arm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Arm)))
  "Returns string type for a message object of type 'Arm"
  "transbot_msgs/Arm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Arm>)))
  "Returns md5sum for a message object of type '<Arm>"
  "b46d7c4342769b3898ee5c56a7392dd2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Arm)))
  "Returns md5sum for a message object of type 'Arm"
  "b46d7c4342769b3898ee5c56a7392dd2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Arm>)))
  "Returns full string definition for message of type '<Arm>"
  (cl:format cl:nil "Joint[] joint~%~%================================================================================~%MSG: transbot_msgs/Joint~%int32 id~%int32 run_time~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Arm)))
  "Returns full string definition for message of type 'Arm"
  (cl:format cl:nil "Joint[] joint~%~%================================================================================~%MSG: transbot_msgs/Joint~%int32 id~%int32 run_time~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Arm>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Arm>))
  "Converts a ROS message object to a list"
  (cl:list 'Arm
    (cl:cons ':joint (joint msg))
))
