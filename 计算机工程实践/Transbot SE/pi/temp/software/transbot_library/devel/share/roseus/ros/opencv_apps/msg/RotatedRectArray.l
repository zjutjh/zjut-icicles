;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::RotatedRectArray)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'RotatedRectArray (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::ROTATEDRECTARRAY")
  (make-package "OPENCV_APPS::ROTATEDRECTARRAY"))

(in-package "ROS")
;;//! \htmlinclude RotatedRectArray.msg.html


(defclass opencv_apps::RotatedRectArray
  :super ros::object
  :slots (_rects ))

(defmethod opencv_apps::RotatedRectArray
  (:init
   (&key
    ((:rects __rects) (let (r) (dotimes (i 0) (push (instance opencv_apps::RotatedRect :init) r)) r))
    )
   (send-super :init)
   (setq _rects __rects)
   self)
  (:rects
   (&rest __rects)
   (if (keywordp (car __rects))
       (send* _rects __rects)
     (progn
       (if __rects (setq _rects (car __rects)))
       _rects)))
  (:serialization-length
   ()
   (+
    ;; opencv_apps/RotatedRect[] _rects
    (apply #'+ (send-all _rects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/RotatedRect[] _rects
     (write-long (length _rects) s)
     (dolist (elem _rects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/RotatedRect[] _rects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rects (let (r) (dotimes (i n) (push (instance opencv_apps::RotatedRect :init) r)) r))
     (dolist (elem- _rects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::RotatedRectArray :md5sum-) "a27e397ed2b5b1a633561d324f64d2a6")
(setf (get opencv_apps::RotatedRectArray :datatype-) "opencv_apps/RotatedRectArray")
(setf (get opencv_apps::RotatedRectArray :definition-)
      "RotatedRect[] rects

================================================================================
MSG: opencv_apps/RotatedRect
float64 angle
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



(provide :opencv_apps/RotatedRectArray "a27e397ed2b5b1a633561d324f64d2a6")


