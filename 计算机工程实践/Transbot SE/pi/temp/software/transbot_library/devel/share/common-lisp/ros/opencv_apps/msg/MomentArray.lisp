; Auto-generated. Do not edit!


(cl:in-package opencv_apps-msg)


;//! \htmlinclude MomentArray.msg.html

(cl:defclass <MomentArray> (roslisp-msg-protocol:ros-message)
  ((moments
    :reader moments
    :initarg :moments
    :type (cl:vector opencv_apps-msg:Moment)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Moment :initial-element (cl:make-instance 'opencv_apps-msg:Moment))))
)

(cl:defclass MomentArray (<MomentArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MomentArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MomentArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-msg:<MomentArray> is deprecated: use opencv_apps-msg:MomentArray instead.")))

(cl:ensure-generic-function 'moments-val :lambda-list '(m))
(cl:defmethod moments-val ((m <MomentArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-msg:moments-val is deprecated.  Use opencv_apps-msg:moments instead.")
  (moments m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MomentArray>) ostream)
  "Serializes a message object of type '<MomentArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'moments))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'moments))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MomentArray>) istream)
  "Deserializes a message object of type '<MomentArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'moments) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'moments)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'opencv_apps-msg:Moment))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MomentArray>)))
  "Returns string type for a message object of type '<MomentArray>"
  "opencv_apps/MomentArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MomentArray)))
  "Returns string type for a message object of type 'MomentArray"
  "opencv_apps/MomentArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MomentArray>)))
  "Returns md5sum for a message object of type '<MomentArray>"
  "fb51ddd1dea5da45f56842efe759d448")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MomentArray)))
  "Returns md5sum for a message object of type 'MomentArray"
  "fb51ddd1dea5da45f56842efe759d448")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MomentArray>)))
  "Returns full string definition for message of type '<MomentArray>"
  (cl:format cl:nil "Moment[] moments~%~%================================================================================~%MSG: opencv_apps/Moment~%# spatial moments~%float64 m00~%float64 m10~%float64 m01~%float64 m20~%float64 m11~%float64 m02~%float64 m30~%float64 m21~%float64 m12~%float64 m03~%~%# central moments~%float64 mu20~%float64 mu11~%float64 mu02~%float64 mu30~%float64 mu21~%float64 mu12~%float64 mu03~%~%# central normalized moments~%float64 nu20~%float64 nu11~%float64 nu02~%float64 nu30~%float64 nu21~%float64 nu12~%float64 nu03~%~%# center of mass m10/m00, m01/m00~%Point2D center~%float64 length~%float64 area~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MomentArray)))
  "Returns full string definition for message of type 'MomentArray"
  (cl:format cl:nil "Moment[] moments~%~%================================================================================~%MSG: opencv_apps/Moment~%# spatial moments~%float64 m00~%float64 m10~%float64 m01~%float64 m20~%float64 m11~%float64 m02~%float64 m30~%float64 m21~%float64 m12~%float64 m03~%~%# central moments~%float64 mu20~%float64 mu11~%float64 mu02~%float64 mu30~%float64 mu21~%float64 mu12~%float64 mu03~%~%# central normalized moments~%float64 nu20~%float64 nu11~%float64 nu02~%float64 nu30~%float64 nu21~%float64 nu12~%float64 nu03~%~%# center of mass m10/m00, m01/m00~%Point2D center~%float64 length~%float64 area~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MomentArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'moments) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MomentArray>))
  "Converts a ROS message object to a list"
  (cl:list 'MomentArray
    (cl:cons ':moments (moments msg))
))
