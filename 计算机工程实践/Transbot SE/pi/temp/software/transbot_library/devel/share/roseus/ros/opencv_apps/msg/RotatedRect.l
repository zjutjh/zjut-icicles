;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::RotatedRect)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'RotatedRect (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::ROTATEDRECT")
  (make-package "OPENCV_APPS::ROTATEDRECT"))

(in-package "ROS")
;;//! \htmlinclude RotatedRect.msg.html


(defclass opencv_apps::RotatedRect
  :super ros::object
  :slots (_angle _center _size ))

(defmethod opencv_apps::RotatedRect
  (:init
   (&key
    ((:angle __angle) 0.0)
    ((:center __center) (instance opencv_apps::Point2D :init))
    ((:size __size) (instance opencv_apps::Size :init))
    )
   (send-super :init)
   (setq _angle (float __angle))
   (setq _center __center)
   (setq _size __size)
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:size
   (&rest __size)
   (if (keywordp (car __size))
       (send* _size __size)
     (progn
       (if __size (setq _size (car __size)))
       _size)))
  (:serialization-length
   ()
   (+
    ;; float64 _angle
    8
    ;; opencv_apps/Point2D _center
    (send _center :serialization-length)
    ;; opencv_apps/Size _size
    (send _size :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; opencv_apps/Point2D _center
       (send _center :serialize s)
     ;; opencv_apps/Size _size
       (send _size :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; opencv_apps/Point2D _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; opencv_apps/Size _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;;
   self)
  )

(setf (get opencv_apps::RotatedRect :md5sum-) "0ae60505c52f020176686d0689b8d390")
(setf (get opencv_apps::RotatedRect :datatype-) "opencv_apps/RotatedRect")
(setf (get opencv_apps::RotatedRect :definition-)
      "float64 angle
Point2D center
Size size

================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


================================================================================
MSG: opencv_apps/Size
float64 width
float64 height


")



(provide :opencv_apps/RotatedRect "0ae60505c52f020176686d0689b8d390")


