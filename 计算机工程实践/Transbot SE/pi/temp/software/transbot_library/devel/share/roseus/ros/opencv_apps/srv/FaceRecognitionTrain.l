;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::FaceRecognitionTrain)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'FaceRecognitionTrain (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::FACERECOGNITIONTRAIN")
  (make-package "OPENCV_APPS::FACERECOGNITIONTRAIN"))
(unless (find-package "OPENCV_APPS::FACERECOGNITIONTRAINREQUEST")
  (make-package "OPENCV_APPS::FACERECOGNITIONTRAINREQUEST"))
(unless (find-package "OPENCV_APPS::FACERECOGNITIONTRAINRESPONSE")
  (make-package "OPENCV_APPS::FACERECOGNITIONTRAINRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass opencv_apps::FaceRecognitionTrainRequest
  :super ros::object
  :slots (_images _rects _labels ))

(defmethod opencv_apps::FaceRecognitionTrainRequest
  (:init
   (&key
    ((:images __images) (let (r) (dotimes (i 0) (push (instance sensor_msgs::Image :init) r)) r))
    ((:rects __rects) (let (r) (dotimes (i 0) (push (instance opencv_apps::Rect :init) r)) r))
    ((:labels __labels) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _images __images)
   (setq _rects __rects)
   (setq _labels __labels)
   self)
  (:images
   (&rest __images)
   (if (keywordp (car __images))
       (send* _images __images)
     (progn
       (if __images (setq _images (car __images)))
       _images)))
  (:rects
   (&rest __rects)
   (if (keywordp (car __rects))
       (send* _rects __rects)
     (progn
       (if __rects (setq _rects (car __rects)))
       _rects)))
  (:labels
   (&optional __labels)
   (if __labels (setq _labels __labels)) _labels)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image[] _images
    (apply #'+ (send-all _images :serialization-length)) 4
    ;; opencv_apps/Rect[] _rects
    (apply #'+ (send-all _rects :serialization-length)) 4
    ;; string[] _labels
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _labels)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image[] _images
     (write-long (length _images) s)
     (dolist (elem _images)
       (send elem :serialize s)
       )
     ;; opencv_apps/Rect[] _rects
     (write-long (length _rects) s)
     (dolist (elem _rects)
       (send elem :serialize s)
       )
     ;; string[] _labels
     (write-long (length _labels) s)
     (dolist (elem _labels)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image[] _images
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _images (let (r) (dotimes (i n) (push (instance sensor_msgs::Image :init) r)) r))
     (dolist (elem- _images)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; opencv_apps/Rect[] _rects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rects (let (r) (dotimes (i n) (push (instance opencv_apps::Rect :init) r)) r))
     (dolist (elem- _rects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; string[] _labels
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _labels (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _labels i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass opencv_apps::FaceRecognitionTrainResponse
  :super ros::object
  :slots (_ok _error ))

(defmethod opencv_apps::FaceRecognitionTrainResponse
  (:init
   (&key
    ((:ok __ok) nil)
    ((:error __error) "")
    )
   (send-super :init)
   (setq _ok __ok)
   (setq _error (string __error))
   self)
  (:ok
   (&optional __ok)
   (if __ok (setq _ok __ok)) _ok)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:serialization-length
   ()
   (+
    ;; bool _ok
    1
    ;; string _error
    4 (length _error)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;; string _error
       (write-long (length _error) s) (princ _error s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _error
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _error (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass opencv_apps::FaceRecognitionTrain
  :super ros::object
  :slots ())

(setf (get opencv_apps::FaceRecognitionTrain :md5sum-) "c47a3ceb75cbe248d69217439e66a8e2")
(setf (get opencv_apps::FaceRecognitionTrain :datatype-) "opencv_apps/FaceRecognitionTrain")
(setf (get opencv_apps::FaceRecognitionTrain :request) opencv_apps::FaceRecognitionTrainRequest)
(setf (get opencv_apps::FaceRecognitionTrain :response) opencv_apps::FaceRecognitionTrainResponse)

(defmethod opencv_apps::FaceRecognitionTrainRequest
  (:response () (instance opencv_apps::FaceRecognitionTrainResponse :init)))

(setf (get opencv_apps::FaceRecognitionTrainRequest :md5sum-) "c47a3ceb75cbe248d69217439e66a8e2")
(setf (get opencv_apps::FaceRecognitionTrainRequest :datatype-) "opencv_apps/FaceRecognitionTrainRequest")
(setf (get opencv_apps::FaceRecognitionTrainRequest :definition-)
      "sensor_msgs/Image[] images
Rect[] rects
string[] labels

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: opencv_apps/Rect
# opencv Rect data type, x-y is center point
float64 x
float64 y
float64 width
float64 height

---
bool ok
string error

")

(setf (get opencv_apps::FaceRecognitionTrainResponse :md5sum-) "c47a3ceb75cbe248d69217439e66a8e2")
(setf (get opencv_apps::FaceRecognitionTrainResponse :datatype-) "opencv_apps/FaceRecognitionTrainResponse")
(setf (get opencv_apps::FaceRecognitionTrainResponse :definition-)
      "sensor_msgs/Image[] images
Rect[] rects
string[] labels

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: opencv_apps/Rect
# opencv Rect data type, x-y is center point
float64 x
float64 y
float64 width
float64 height

---
bool ok
string error

")



(provide :opencv_apps/FaceRecognitionTrain "c47a3ceb75cbe248d69217439e66a8e2")


