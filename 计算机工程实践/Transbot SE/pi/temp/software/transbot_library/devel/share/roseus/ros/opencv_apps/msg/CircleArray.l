;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::CircleArray)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'CircleArray (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::CIRCLEARRAY")
  (make-package "OPENCV_APPS::CIRCLEARRAY"))

(in-package "ROS")
;;//! \htmlinclude CircleArray.msg.html


(defclass opencv_apps::CircleArray
  :super ros::object
  :slots (_circles ))

(defmethod opencv_apps::CircleArray
  (:init
   (&key
    ((:circles __circles) (let (r) (dotimes (i 0) (push (instance opencv_apps::Circle :init) r)) r))
    )
   (send-super :init)
   (setq _circles __circles)
   self)
  (:circles
   (&rest __circles)
   (if (keywordp (car __circles))
       (send* _circles __circles)
     (progn
       (if __circles (setq _circles (car __circles)))
       _circles)))
  (:serialization-length
   ()
   (+
    ;; opencv_apps/Circle[] _circles
    (apply #'+ (send-all _circles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Circle[] _circles
     (write-long (length _circles) s)
     (dolist (elem _circles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Circle[] _circles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _circles (let (r) (dotimes (i n) (push (instance opencv_apps::Circle :init) r)) r))
     (dolist (elem- _circles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::CircleArray :md5sum-) "1970b146e338dd024c765e522039a727")
(setf (get opencv_apps::CircleArray :datatype-) "opencv_apps/CircleArray")
(setf (get opencv_apps::CircleArray :definition-)
      "Circle[] circles


================================================================================
MSG: opencv_apps/Circle
Point2D center
float64 radius


================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/CircleArray "1970b146e338dd024c765e522039a727")


