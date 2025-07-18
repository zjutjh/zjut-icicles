; Auto-generated. Do not edit!


(cl:in-package robot_localization-srv)


;//! \htmlinclude SetDatum-request.msg.html

(cl:defclass <SetDatum-request> (roslisp-msg-protocol:ros-message)
  ((geo_pose
    :reader geo_pose
    :initarg :geo_pose
    :type geographic_msgs-msg:GeoPose
    :initform (cl:make-instance 'geographic_msgs-msg:GeoPose)))
)

(cl:defclass SetDatum-request (<SetDatum-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDatum-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDatum-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_localization-srv:<SetDatum-request> is deprecated: use robot_localization-srv:SetDatum-request instead.")))

(cl:ensure-generic-function 'geo_pose-val :lambda-list '(m))
(cl:defmethod geo_pose-val ((m <SetDatum-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_localization-srv:geo_pose-val is deprecated.  Use robot_localization-srv:geo_pose instead.")
  (geo_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDatum-request>) ostream)
  "Serializes a message object of type '<SetDatum-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'geo_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDatum-request>) istream)
  "Deserializes a message object of type '<SetDatum-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'geo_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDatum-request>)))
  "Returns string type for a service object of type '<SetDatum-request>"
  "robot_localization/SetDatumRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDatum-request)))
  "Returns string type for a service object of type 'SetDatum-request"
  "robot_localization/SetDatumRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDatum-request>)))
  "Returns md5sum for a message object of type '<SetDatum-request>"
  "fe903ca95d0210defda73a1629604439")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDatum-request)))
  "Returns md5sum for a message object of type 'SetDatum-request"
  "fe903ca95d0210defda73a1629604439")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDatum-request>)))
  "Returns full string definition for message of type '<SetDatum-request>"
  (cl:format cl:nil "geographic_msgs/GeoPose geo_pose~%~%================================================================================~%MSG: geographic_msgs/GeoPose~%# Geographic pose, using the WGS 84 reference ellipsoid.~%#~%# Orientation uses the East-North-Up (ENU) frame of reference.~%# (But, what about singularities at the poles?)~%~%GeoPoint position~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: geographic_msgs/GeoPoint~%# Geographic point, using the WGS 84 reference ellipsoid.~%~%# Latitude [degrees]. Positive is north of equator; negative is south~%# (-90 <= latitude <= +90).~%float64 latitude~%~%# Longitude [degrees]. Positive is east of prime meridian; negative is~%# west (-180 <= longitude <= +180). At the poles, latitude is -90 or~%# +90, and longitude is irrelevant, but must be in range.~%float64 longitude~%~%# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).~%float64 altitude~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDatum-request)))
  "Returns full string definition for message of type 'SetDatum-request"
  (cl:format cl:nil "geographic_msgs/GeoPose geo_pose~%~%================================================================================~%MSG: geographic_msgs/GeoPose~%# Geographic pose, using the WGS 84 reference ellipsoid.~%#~%# Orientation uses the East-North-Up (ENU) frame of reference.~%# (But, what about singularities at the poles?)~%~%GeoPoint position~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: geographic_msgs/GeoPoint~%# Geographic point, using the WGS 84 reference ellipsoid.~%~%# Latitude [degrees]. Positive is north of equator; negative is south~%# (-90 <= latitude <= +90).~%float64 latitude~%~%# Longitude [degrees]. Positive is east of prime meridian; negative is~%# west (-180 <= longitude <= +180). At the poles, latitude is -90 or~%# +90, and longitude is irrelevant, but must be in range.~%float64 longitude~%~%# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).~%float64 altitude~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDatum-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'geo_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDatum-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDatum-request
    (cl:cons ':geo_pose (geo_pose msg))
))
;//! \htmlinclude SetDatum-response.msg.html

(cl:defclass <SetDatum-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetDatum-response (<SetDatum-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDatum-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDatum-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_localization-srv:<SetDatum-response> is deprecated: use robot_localization-srv:SetDatum-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDatum-response>) ostream)
  "Serializes a message object of type '<SetDatum-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDatum-response>) istream)
  "Deserializes a message object of type '<SetDatum-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDatum-response>)))
  "Returns string type for a service object of type '<SetDatum-response>"
  "robot_localization/SetDatumResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDatum-response)))
  "Returns string type for a service object of type 'SetDatum-response"
  "robot_localization/SetDatumResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDatum-response>)))
  "Returns md5sum for a message object of type '<SetDatum-response>"
  "fe903ca95d0210defda73a1629604439")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDatum-response)))
  "Returns md5sum for a message object of type 'SetDatum-response"
  "fe903ca95d0210defda73a1629604439")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDatum-response>)))
  "Returns full string definition for message of type '<SetDatum-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDatum-response)))
  "Returns full string definition for message of type 'SetDatum-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDatum-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDatum-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDatum-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDatum)))
  'SetDatum-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDatum)))
  'SetDatum-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDatum)))
  "Returns string type for a service object of type '<SetDatum>"
  "robot_localization/SetDatum")