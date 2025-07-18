; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude LineArray.msg.html

(cl:defclass <LineArray> (roslisp-msg-protocol:ros-message)
  ((lines
    :reader lines
    :initarg :lines
    :type (cl:vector opencv_apps-msg:Line)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Line :initial-element (cl:make-instance 'opencv_apps-msg:Line))))
)

(cl:defclass LineArray (<LineArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LineArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LineArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<LineArray> is deprecated: use opencv_apps-msg:LineArray instead.")))

(cl:ensure-generic-function 'lines-val :lambda-list '(m))
(cl:defmethod lines-val ((m <LineArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:lines-val is deprecated.  Use opencv_apps-msg:lines instead.")
  (lines m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LineArray>) ostream)
  "Serializes a message object of type '<LineArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'lines))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LineArray>) istream)
  "Deserializes a message object of type '<LineArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lines)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Line))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LineArray>)))
  "Returns string type for a message object of type '<LineArray>"
  "opencv_apps/LineArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LineArray)))
  "Returns string type for a message object of type 'LineArray"
  "opencv_apps/LineArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LineArray>)))
  "Returns md5sum for a message object of type '<LineArray>"
  "2b5441933900cc71528395dda29124da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LineArray)))
  "Returns md5sum for a message object of type 'LineArray"
  "2b5441933900cc71528395dda29124da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LineArray>)))
  "Returns full string definition for message of type '<LineArray>"
  (cl:format cl:nil "Line[] lines~%~%================================================================================~%MSG: opencv_apps/Line~%Point2D pt1~%Point2D pt2~%~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LineArray)))
  "Returns full string definition for message of type 'LineArray"
  (cl:format cl:nil "Line[] lines~%~%================================================================================~%MSG: opencv_apps/Line~%Point2D pt1~%Point2D pt2~%~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LineArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LineArray>))
  "Converts a ROS message object to a list"
  (cl:list 'LineArray
    (cl:cons ':lines (lines msg))
))
