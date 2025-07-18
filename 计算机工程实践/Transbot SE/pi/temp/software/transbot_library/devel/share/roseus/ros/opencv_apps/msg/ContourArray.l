;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::ContourArray)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'ContourArray (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::CONTOURARRAY")
  (make-package "OPENCV_APPS::CONTOURARRAY"))

(in-package "ROS")
;;//! \htmlinclude ContourArray.msg.html


(defclass opencv_apps::ContourArray
  :super ros::object
  :slots (_contours ))

(defmethod opencv_apps::ContourArray
  (:init
   (&key
    ((:contours __contours) (let (r) (dotimes (i 0) (push (instance opencv_apps::Contour :init) r)) r))
    )
   (send-super :init)
   (setq _contours __contours)
   self)
  (:contours
   (&rest __contours)
   (if (keywordp (car __contours))
       (send* _contours __contours)
     (progn
       (if __contours (setq _contours (car __contours)))
       _contours)))
  (:serialization-length
   ()
   (+
    ;; opencv_apps/Contour[] _contours
    (apply #'+ (send-all _contours :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Contour[] _contours
     (write-long (length _contours) s)
     (dolist (elem _contours)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Contour[] _contours
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _contours (let (r) (dotimes (i n) (push (instance opencv_apps::Contour :init) r)) r))
     (dolist (elem- _contours)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::ContourArray :md5sum-) "fc54374f45559dfed248b316ccf9181d")
(setf (get opencv_apps::ContourArray :datatype-) "opencv_apps/ContourArray")
(setf (get opencv_apps::ContourArray :definition-)
      "Contour[] contours

================================================================================
MSG: opencv_apps/Contour
Point2D[] points

================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/ContourArray "fc54374f45559dfed248b316ccf9181d")


