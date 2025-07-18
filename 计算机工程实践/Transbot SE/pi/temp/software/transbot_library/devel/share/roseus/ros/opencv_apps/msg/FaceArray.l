;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::FaceArray)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'FaceArray (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::FACEARRAY")
  (make-package "OPENCV_APPS::FACEARRAY"))

(in-package "ROS")
;;//! \htmlinclude FaceArray.msg.html


(defclass opencv_apps::FaceArray
  :super ros::object
  :slots (_faces ))

(defmethod opencv_apps::FaceArray
  (:init
   (&key
    ((:faces __faces) (let (r) (dotimes (i 0) (push (instance opencv_apps::Face :init) r)) r))
    )
   (send-super :init)
   (setq _faces __faces)
   self)
  (:faces
   (&rest __faces)
   (if (keywordp (car __faces))
       (send* _faces __faces)
     (progn
       (if __faces (setq _faces (car __faces)))
       _faces)))
  (:serialization-length
   ()
   (+
    ;; opencv_apps/Face[] _faces
    (apply #'+ (send-all _faces :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Face[] _faces
     (write-long (length _faces) s)
     (dolist (elem _faces)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Face[] _faces
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _faces (let (r) (dotimes (i n) (push (instance opencv_apps::Face :init) r)) r))
     (dolist (elem- _faces)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::FaceArray :md5sum-) "3ae7a36ff47d72f5dd1d764612b2b3c8")
(setf (get opencv_apps::FaceArray :datatype-) "opencv_apps/FaceArray")
(setf (get opencv_apps::FaceArray :definition-)
      "Face[] faces


================================================================================
MSG: opencv_apps/Face
Rect face
Rect[] eyes
string label
float64 confidence

================================================================================
MSG: opencv_apps/Rect
# opencv Rect data type, x-y is center point
float64 x
float64 y
float64 width
float64 height


")



(provide :opencv_apps/FaceArray "3ae7a36ff47d72f5dd1d764612b2b3c8")


