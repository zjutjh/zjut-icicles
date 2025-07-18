;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::Rect)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'Rect (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::RECT")
  (make-package "OPENCV_APPS::RECT"))

(in-package "ROS")
;;//! \htmlinclude Rect.msg.html


(defclass opencv_apps::Rect
  :super ros::object
  :slots (_x _y _width _height ))

(defmethod opencv_apps::Rect
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:width __width) 0.0)
    ((:height __height) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _width (float __width))
   (setq _height (float __height))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _width
    8
    ;; float64 _height
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _width
       (sys::poke _width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _width
     (setq _width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get opencv_apps::Rect :md5sum-) "7048f28f1f0ef51e102638c86d9a7728")
(setf (get opencv_apps::Rect :datatype-) "opencv_apps/Rect")
(setf (get opencv_apps::Rect :definition-)
      "# opencv Rect data type, x-y is center point
float64 x
float64 y
float64 width
float64 height


")



(provide :opencv_apps/Rect "7048f28f1f0ef51e102638c86d9a7728")


