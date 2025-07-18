; Auto-generated. Do not edit!


(cl:in-package robot_localization-srv)


;//! \htmlinclude GetState-request.msg.html

(cl:defclass <GetState-request> (roslisp-msg-protocol:ros-message)
  ((time_stamp
    :reader time_stamp
    :initarg :time_stamp
    :type cl:real
    :initform 0)
   (frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform ""))
)

(cl:defclass GetState-request (<GetState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_localization-srv:<GetState-request> is deprecated: use robot_localization-srv:GetState-request instead.")))

(cl:ensure-generic-function 'time_stamp-val :lambda-list '(m))
(cl:defmethod time_stamp-val ((m <GetState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_localization-srv:time_stamp-val is deprecated.  Use robot_localization-srv:time_stamp instead.")
  (time_stamp m))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <GetState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_localization-srv:frame_id-val is deprecated.  Use robot_localization-srv:frame_id instead.")
  (frame_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetState-request>) ostream)
  "Serializes a message object of type '<GetState-request>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_stamp) (cl:floor (cl:slot-value msg 'time_stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetState-request>) istream)
  "Deserializes a message object of type '<GetState-request>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetState-request>)))
  "Returns string type for a service object of type '<GetState-request>"
  "robot_localization/GetStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetState-request)))
  "Returns string type for a service object of type 'GetState-request"
  "robot_localization/GetStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetState-request>)))
  "Returns md5sum for a message object of type '<GetState-request>"
  "b143410e9c7f7be208eedf8f691d8069")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetState-request)))
  "Returns md5sum for a message object of type 'GetState-request"
  "b143410e9c7f7be208eedf8f691d8069")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetState-request>)))
  "Returns full string definition for message of type '<GetState-request>"
  (cl:format cl:nil "time time_stamp~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetState-request)))
  "Returns full string definition for message of type 'GetState-request"
  (cl:format cl:nil "time time_stamp~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetState-request>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'frame_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetState-request
    (cl:cons ':time_stamp (time_stamp msg))
    (cl:cons ':frame_id (frame_id msg))
))
;//! \htmlinclude GetState-response.msg.html

(cl:defclass <GetState-response> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type (cl:vector cl:float)
   :initform (cl:make-array 15 :element-type 'cl:float :initial-element 0.0))
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 225 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetState-response (<GetState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_localization-srv:<GetState-response> is deprecated: use robot_localization-srv:GetState-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <GetState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_localization-srv:state-val is deprecated.  Use robot_localization-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <GetState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_localization-srv:covariance-val is deprecated.  Use robot_localization-srv:covariance instead.")
  (covariance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetState-response>) ostream)
  "Serializes a message object of type '<GetState-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'state))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'covariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetState-response>) istream)
  "Deserializes a message object of type '<GetState-response>"
  (cl:setf (cl:slot-value msg 'state) (cl:make-array 15))
  (cl:let ((vals (cl:slot-value msg 'state)))
    (cl:dotimes (i 15)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array 225))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i 225)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetState-response>)))
  "Returns string type for a service object of type '<GetState-response>"
  "robot_localization/GetStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetState-response)))
  "Returns string type for a service object of type 'GetState-response"
  "robot_localization/GetStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetState-response>)))
  "Returns md5sum for a message object of type '<GetState-response>"
  "b143410e9c7f7be208eedf8f691d8069")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetState-response)))
  "Returns md5sum for a message object of type 'GetState-response"
  "b143410e9c7f7be208eedf8f691d8069")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetState-response>)))
  "Returns full string definition for message of type '<GetState-response>"
  (cl:format cl:nil "# State vector: x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az~%float64[15] state~%~%# Covariance matrix in row-major order~%float64[225] covariance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetState-response)))
  "Returns full string definition for message of type 'GetState-response"
  (cl:format cl:nil "# State vector: x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az~%float64[15] state~%~%# Covariance matrix in row-major order~%float64[225] covariance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetState-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetState-response
    (cl:cons ':state (state msg))
    (cl:cons ':covariance (covariance msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetState)))
  'GetState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetState)))
  'GetState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetState)))
  "Returns string type for a service object of type '<GetState>"
  "robot_localization/GetState")