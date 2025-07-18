;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::MomentArray)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'MomentArray (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::MOMENTARRAY")
  (make-package "OPENCV_APPS::MOMENTARRAY"))

(in-package "ROS")
;;//! \htmlinclude MomentArray.msg.html


(defclass opencv_apps::MomentArray
  :super ros::object
  :slots (_moments ))

(defmethod opencv_apps::MomentArray
  (:init
   (&key
    ((:moments __moments) (let (r) (dotimes (i 0) (push (instance opencv_apps::Moment :init) r)) r))
    )
   (send-super :init)
   (setq _moments __moments)
   self)
  (:moments
   (&rest __moments)
   (if (keywordp (car __moments))
       (send* _moments __moments)
     (progn
       (if __moments (setq _moments (car __moments)))
       _moments)))
  (:serialization-length
   ()
   (+
    ;; opencv_apps/Moment[] _moments
    (apply #'+ (send-all _moments :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Moment[] _moments
     (write-long (length _moments) s)
     (dolist (elem _moments)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Moment[] _moments
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _moments (let (r) (dotimes (i n) (push (instance opencv_apps::Moment :init) r)) r))
     (dolist (elem- _moments)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::MomentArray :md5sum-) "fb51ddd1dea5da45f56842efe759d448")
(setf (get opencv_apps::MomentArray :datatype-) "opencv_apps/MomentArray")
(setf (get opencv_apps::MomentArray :definition-)
      "Moment[] moments

================================================================================
MSG: opencv_apps/Moment
# spatial moments
float64 m00
float64 m10
float64 m01
float64 m20
float64 m11
float64 m02
float64 m30
float64 m21
float64 m12
float64 m03

# central moments
float64 mu20
float64 mu11
float64 mu02
float64 mu30
float64 mu21
float64 mu12
float64 mu03

# central normalized moments
float64 nu20
float64 nu11
float64 nu02
float64 nu30
float64 nu21
float64 nu12
float64 nu03

# center of mass m10/m00, m01/m00
Point2D center
float64 length
float64 area

================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/MomentArray "fb51ddd1dea5da45f56842efe759d448")


