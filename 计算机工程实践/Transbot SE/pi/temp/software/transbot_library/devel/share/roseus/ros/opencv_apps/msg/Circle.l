;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::Circle)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'Circle (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::CIRCLE")
  (make-package "OPENCV_APPS::CIRCLE"))

(in-package "ROS")
;;//! \htmlinclude Circle.msg.html


(defclass opencv_apps::Circle
  :super ros::object
  :slots (_center _radius ))

(defmethod opencv_apps::Circle
  (:init
   (&key
    ((:center __center) (instance opencv_apps::Point2D :init))
    ((:radius __radius) 0.0)
    )
   (send-super :init)
   (setq _center __center)
   (setq _radius (float __radius))
   self)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:serialization-length
   ()
   (+
    ;; opencv_apps/Point2D _center
    (send _center :serialization-length)
    ;; float64 _radius
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Point2D _center
       (send _center :serialize s)
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Point2D _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get opencv_apps::Circle :md5sum-) "4f6847051b4fe493b5af8caad66201d5")
(setf (get opencv_apps::Circle :datatype-) "opencv_apps/Circle")
(setf (get opencv_apps::Circle :definition-)
      "Point2D center
float64 radius


================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/Circle "4f6847051b4fe493b5af8caad66201d5")


