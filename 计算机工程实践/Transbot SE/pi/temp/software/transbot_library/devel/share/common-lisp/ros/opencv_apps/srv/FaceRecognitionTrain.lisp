; Auto-generated. Do not edit!


(cl:in-package opencv_apps-srv)


;//! \htmlinclude FaceRecognitionTrain-request.msg.html

(cl:defclass <FaceRecognitionTrain-request> (roslisp-msg-protocol:ros-message)
  ((images
    :reader images
    :initarg :images
    :type (cl:vector sensor_msgs-msg:Image)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:Image :initial-element (cl:make-instance 'sensor_msgs-msg:Image)))
   (rects
    :reader rects
    :initarg :rects
    :type (cl:vector opencv_apps-msg:Rect)
   :initform (cl:make-array 0 :element-type 'opencv_apps-msg:Rect :initial-element (cl:make-instance 'opencv_apps-msg:Rect)))
   (labels
    :reader labels
    :initarg :labels
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass FaceRecognitionTrain-request (<FaceRecognitionTrain-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaceRecognitionTrain-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaceRecognitionTrain-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-srv:<FaceRecognitionTrain-request> is deprecated: use opencv_apps-srv:FaceRecognitionTrain-request instead.")))

(cl:ensure-generic-function 'images-val :lambda-list '(m))
(cl:defmethod images-val ((m <FaceRecognitionTrain-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-srv:images-val is deprecated.  Use opencv_apps-srv:images instead.")
  (images m))

(cl:ensure-generic-function 'rects-val :lambda-list '(m))
(cl:defmethod rects-val ((m <FaceRecognitionTrain-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-srv:rects-val is deprecated.  Use opencv_apps-srv:rects instead.")
  (rects m))

(cl:ensure-generic-function 'labels-val :lambda-list '(m))
(cl:defmethod labels-val ((m <FaceRecognitionTrain-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-srv:labels-val is deprecated.  Use opencv_apps-srv:labels instead.")
  (labels m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaceRecognitionTrain-request>) ostream)
  "Serializes a message object of type '<FaceRecognitionTrain-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'images))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'images))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'rects))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'labels))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'labels))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaceRecognitionTrain-request>) istream)
  "Deserializes a message object of type '<FaceRecognitionTrain-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'images) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'images)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:Image))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'labels) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'labels)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaceRecognitionTrain-request>)))
  "Returns string type for a service object of type '<FaceRecognitionTrain-request>"
  "opencv_apps/FaceRecognitionTrainRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaceRecognitionTrain-request)))
  "Returns string type for a service object of type 'FaceRecognitionTrain-request"
  "opencv_apps/FaceRecognitionTrainRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaceRecognitionTrain-request>)))
  "Returns md5sum for a message object of type '<FaceRecognitionTrain-request>"
  "c47a3ceb75cbe248d69217439e66a8e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaceRecognitionTrain-request)))
  "Returns md5sum for a message object of type 'FaceRecognitionTrain-request"
  "c47a3ceb75cbe248d69217439e66a8e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaceRecognitionTrain-request>)))
  "Returns full string definition for message of type '<FaceRecognitionTrain-request>"
  (cl:format cl:nil "sensor_msgs/Image[] images~%Rect[] rects~%string[] labels~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Rect~%# opencv Rect data type, x-y is center point~%float64 x~%float64 y~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaceRecognitionTrain-request)))
  "Returns full string definition for message of type 'FaceRecognitionTrain-request"
  (cl:format cl:nil "sensor_msgs/Image[] images~%Rect[] rects~%string[] labels~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: opencv_apps/Rect~%# opencv Rect data type, x-y is center point~%float64 x~%float64 y~%float64 width~%float64 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaceRecognitionTrain-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'images) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'labels) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaceRecognitionTrain-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FaceRecognitionTrain-request
    (cl:cons ':images (images msg))
    (cl:cons ':rects (rects msg))
    (cl:cons ':labels (labels msg))
))
;//! \htmlinclude FaceRecognitionTrain-response.msg.html

(cl:defclass <FaceRecognitionTrain-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil)
   (error
    :reader error
    :initarg :error
    :type cl:string
    :initform ""))
)

(cl:defclass FaceRecognitionTrain-response (<FaceRecognitionTrain-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaceRecognitionTrain-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaceRecognitionTrain-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_apps-srv:<FaceRecognitionTrain-response> is deprecated: use opencv_apps-srv:FaceRecognitionTrain-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <FaceRecognitionTrain-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-srv:ok-val is deprecated.  Use opencv_apps-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <FaceRecognitionTrain-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_apps-srv:error-val is deprecated.  Use opencv_apps-srv:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaceRecognitionTrain-response>) ostream)
  "Serializes a message object of type '<FaceRecognitionTrain-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'error))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaceRecognitionTrain-response>) istream)
  "Deserializes a message object of type '<FaceRecognitionTrain-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'error) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaceRecognitionTrain-response>)))
  "Returns string type for a service object of type '<FaceRecognitionTrain-response>"
  "opencv_apps/FaceRecognitionTrainResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaceRecognitionTrain-response)))
  "Returns string type for a service object of type 'FaceRecognitionTrain-response"
  "opencv_apps/FaceRecognitionTrainResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaceRecognitionTrain-response>)))
  "Returns md5sum for a message object of type '<FaceRecognitionTrain-response>"
  "c47a3ceb75cbe248d69217439e66a8e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaceRecognitionTrain-response)))
  "Returns md5sum for a message object of type 'FaceRecognitionTrain-response"
  "c47a3ceb75cbe248d69217439e66a8e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaceRecognitionTrain-response>)))
  "Returns full string definition for message of type '<FaceRecognitionTrain-response>"
  (cl:format cl:nil "bool ok~%string error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaceRecognitionTrain-response)))
  "Returns full string definition for message of type 'FaceRecognitionTrain-response"
  (cl:format cl:nil "bool ok~%string error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaceRecognitionTrain-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'error))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaceRecognitionTrain-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FaceRecognitionTrain-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':error (error msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FaceRecognitionTrain)))
  'FaceRecognitionTrain-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FaceRecognitionTrain)))
  'FaceRecognitionTrain-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaceRecognitionTrain)))
  "Returns string type for a service object of type '<FaceRecognitionTrain>"
  "opencv_apps/FaceRecognitionTrain")