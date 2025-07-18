;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::Flow)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'Flow (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::FLOW")
  (make-package "OPENCV_APPS::FLOW"))

(in-package "ROS")
;;//! \htmlinclude Flow.msg.html


(defclass opencv_apps::Flow
  :super ros::object
  :slots (_point _velocity ))

(defmethod opencv_apps::Flow
  (:init
   (&key
    ((:point __point) (instance opencv_apps::Point2D :init))
    ((:velocity __velocity) (instance opencv_apps::Point2D :init))
    )
   (send-super :init)
   (setq _point __point)
   (setq _velocity __velocity)
   self)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:serialization-length
   ()
   (+
    ;; opencv_apps/Point2D _point
    (send _point :serialization-length)
    ;; opencv_apps/Point2D _velocity
    (send _velocity :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Point2D _point
       (send _point :serialize s)
     ;; opencv_apps/Point2D _velocity
       (send _velocity :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Point2D _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; opencv_apps/Point2D _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;;
   self)
  )

(setf (get opencv_apps::Flow :md5sum-) "dd9a9efd88ba39035e78af697593d751")
(setf (get opencv_apps::Flow :datatype-) "opencv_apps/Flow")
(setf (get opencv_apps::Flow :definition-)
      "Point2D point
Point2D velocity

================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/Flow "dd9a9efd88ba39035e78af697593d751")


