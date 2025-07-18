;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::RectArray)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'RectArray (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::RECTARRAY")
  (make-package "OPENCV_APPS::RECTARRAY"))

(in-package "ROS")
;;//! \htmlinclude RectArray.msg.html


(defclass opencv_apps::RectArray
  :super ros::object
  :slots (_rects ))

(defmethod opencv_apps::RectArray
  (:init
   (&key
    ((:rects __rects) (let (r) (dotimes (i 0) (push (instance opencv_apps::Rect :init) r)) r))
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
    ;; opencv_apps/Rect[] _rects
    (apply #'+ (send-all _rects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Rect[] _rects
     (write-long (length _rects) s)
     (dolist (elem _rects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Rect[] _rects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rects (let (r) (dotimes (i n) (push (instance opencv_apps::Rect :init) r)) r))
     (dolist (elem- _rects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::RectArray :md5sum-) "d4a6f20c7699fa2791af675958a5f148")
(setf (get opencv_apps::RectArray :datatype-) "opencv_apps/RectArray")
(setf (get opencv_apps::RectArray :definition-)
      "Rect[] rects

================================================================================
MSG: opencv_apps/Rect
# opencv Rect data type, x-y is center point
float64 x
float64 y
float64 width
float64 height


")



(provide :opencv_apps/RectArray "d4a6f20c7699fa2791af675958a5f148")


