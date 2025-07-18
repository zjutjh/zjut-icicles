; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude RectArray.msg.html

(cl:defclass <RectArray> (roslisp-msg-protocol:ros-message)
  ((rects
    :reader rects
    :initarg :rects
    :type (cl:vector opencv_apps-msg:Rect)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Rect :initial-element (cl:make-instance 'opencv_apps-msg:Rect))))
)

(cl:defclass RectArray (<RectArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RectArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RectArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<RectArray> is deprecated: use opencv_apps-msg:RectArray instead.")))

(cl:ensure-generic-function 'rects-val :lambda-list '(m))
(cl:defmethod rects-val ((m <RectArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:rects-val is deprecated.  Use opencv_apps-msg:rects instead.")
  (rects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RectArray>) ostream)
  "Serializes a message object of type '<RectArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'rects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RectArray>) istream)
  "Deserializes a message object of type '<RectArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Rect))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RectArray>)))
  "Returns string type for a message object of type '<RectArray>"
  "opencv_apps/RectArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RectArray)))
  "Returns string type for a message object of type 'RectArray"
  "opencv_apps/RectArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RectArray>)))
  "Returns md5sum for a message object of type '<RectArray>"
  "d4a6f20c7699fa2791af675958a5f148")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RectArray)))
  "Returns md5sum for a message object of type 'RectArray"
  "d4a6f20c7699fa2791af675958a5f148")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RectArray>)))
  "Returns full string definition for message of type '<RectArray>"
  (cl:format cl:nil "Rect[] rects~%~%================================================================================~%MSG: opencv_apps/Rect~%# opencv Rect data type, x-y is center point~%float64 x~%float64 y~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RectArray)))
  "Returns full string definition for message of type 'RectArray"
  (cl:format cl:nil "Rect[] rects~%~%================================================================================~%MSG: opencv_apps/Rect~%# opencv Rect data type, x-y is center point~%float64 x~%float64 y~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RectArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RectArray>))
  "Converts a ROS message object to a list"
  (cl:list 'RectArray
    (cl:cons ':rects (rects msg))
))
