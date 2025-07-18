; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude FaceArray.msg.html

(cl:defclass <FaceArray> (roslisp-msg-protocol:ros-message)
  ((faces
    :reader faces
    :initarg :faces
    :type (cl:vector opencv_apps-msg:Face)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Face :initial-element (cl:make-instance 'opencv_apps-msg:Face))))
)

(cl:defclass FaceArray (<FaceArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaceArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaceArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<FaceArray> is deprecated: use opencv_apps-msg:FaceArray instead.")))

(cl:ensure-generic-function 'faces-val :lambda-list '(m))
(cl:defmethod faces-val ((m <FaceArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:faces-val is deprecated.  Use opencv_apps-msg:faces instead.")
  (faces m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaceArray>) ostream)
  "Serializes a message object of type '<FaceArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'faces))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'faces))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaceArray>) istream)
  "Deserializes a message object of type '<FaceArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'faces) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'faces)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Face))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaceArray>)))
  "Returns string type for a message object of type '<FaceArray>"
  "opencv_apps/FaceArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaceArray)))
  "Returns string type for a message object of type 'FaceArray"
  "opencv_apps/FaceArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaceArray>)))
  "Returns md5sum for a message object of type '<FaceArray>"
  "3ae7a36ff47d72f5dd1d764612b2b3c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaceArray)))
  "Returns md5sum for a message object of type 'FaceArray"
  "3ae7a36ff47d72f5dd1d764612b2b3c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaceArray>)))
  "Returns full string definition for message of type '<FaceArray>"
  (cl:format cl:nil "Face[] faces~%~%~%================================================================================~%MSG: opencv_apps/Face~%Rect face~%Rect[] eyes~%string label~%float64 confidence~%~%================================================================================~%MSG: opencv_apps/Rect~%# opencv Rect data type, x-y is center point~%float64 x~%float64 y~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaceArray)))
  "Returns full string definition for message of type 'FaceArray"
  (cl:format cl:nil "Face[] faces~%~%~%================================================================================~%MSG: opencv_apps/Face~%Rect face~%Rect[] eyes~%string label~%float64 confidence~%~%================================================================================~%MSG: opencv_apps/Rect~%# opencv Rect data type, x-y is center point~%float64 x~%float64 y~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaceArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'faces) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaceArray>))
  "Converts a ROS message object to a list"
  (cl:list 'FaceArray
    (cl:cons ':faces (faces msg))
))
