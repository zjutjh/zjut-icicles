; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude ContourArray.msg.html

(cl:defclass <ContourArray> (roslisp-msg-protocol:ros-message)
  ((contours
    :reader contours
    :initarg :contours
    :type (cl:vector opencv_apps-msg:Contour)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Contour :initial-element (cl:make-instance 'opencv_apps-msg:Contour))))
)

(cl:defclass ContourArray (<ContourArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ContourArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ContourArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<ContourArray> is deprecated: use opencv_apps-msg:ContourArray instead.")))

(cl:ensure-generic-function 'contours-val :lambda-list '(m))
(cl:defmethod contours-val ((m <ContourArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:contours-val is deprecated.  Use opencv_apps-msg:contours instead.")
  (contours m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ContourArray>) ostream)
  "Serializes a message object of type '<ContourArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'contours))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'contours))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ContourArray>) istream)
  "Deserializes a message object of type '<ContourArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'contours) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'contours)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Contour))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ContourArray>)))
  "Returns string type for a message object of type '<ContourArray>"
  "opencv_apps/ContourArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ContourArray)))
  "Returns string type for a message object of type 'ContourArray"
  "opencv_apps/ContourArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ContourArray>)))
  "Returns md5sum for a message object of type '<ContourArray>"
  "fc54374f45559dfed248b316ccf9181d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ContourArray)))
  "Returns md5sum for a message object of type 'ContourArray"
  "fc54374f45559dfed248b316ccf9181d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ContourArray>)))
  "Returns full string definition for message of type '<ContourArray>"
  (cl:format cl:nil "Contour[] contours~%~%================================================================================~%MSG: opencv_apps/Contour~%Point2D[] points~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ContourArray)))
  "Returns full string definition for message of type 'ContourArray"
  (cl:format cl:nil "Contour[] contours~%~%================================================================================~%MSG: opencv_apps/Contour~%Point2D[] points~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ContourArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'contours) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ContourArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ContourArray
    (cl:cons ':contours (contours msg))
))
