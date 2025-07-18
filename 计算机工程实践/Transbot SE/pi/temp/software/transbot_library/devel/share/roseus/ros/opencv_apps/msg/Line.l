;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::Line)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'Line (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::LINE")
  (make-package "OPENCV_APPS::LINE"))

(in-package "ROS")
;;//! \htmlinclude Line.msg.html


(defclass opencv_apps::Line
  :super ros::object
  :slots (_pt1 _pt2 ))

(defmethod opencv_apps::Line
  (:init
   (&key
    ((:pt1 __pt1) (instance opencv_apps::Point2D :init))
    ((:pt2 __pt2) (instance opencv_apps::Point2D :init))
    )
   (send-super :init)
   (setq _pt1 __pt1)
   (setq _pt2 __pt2)
   self)
  (:pt1
   (&rest __pt1)
   (if (keywordp (car __pt1))
       (send* _pt1 __pt1)
     (progn
       (if __pt1 (setq _pt1 (car __pt1)))
       _pt1)))
  (:pt2
   (&rest __pt2)
   (if (keywordp (car __pt2))
       (send* _pt2 __pt2)
     (progn
       (if __pt2 (setq _pt2 (car __pt2)))
       _pt2)))
  (:serialization-length
   ()
   (+
    ;; opencv_apps/Point2D _pt1
    (send _pt1 :serialization-length)
    ;; opencv_apps/Point2D _pt2
    (send _pt2 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Point2D _pt1
       (send _pt1 :serialize s)
     ;; opencv_apps/Point2D _pt2
       (send _pt2 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Point2D _pt1
     (send _pt1 :deserialize buf ptr-) (incf ptr- (send _pt1 :serialization-length))
   ;; opencv_apps/Point2D _pt2
     (send _pt2 :deserialize buf ptr-) (incf ptr- (send _pt2 :serialization-length))
   ;;
   self)
  )

(setf (get opencv_apps::Line :md5sum-) "a1419010b3fc4549e3f450018363d000")
(setf (get opencv_apps::Line :datatype-) "opencv_apps/Line")
(setf (get opencv_apps::Line :definition-)
      "Point2D pt1
Point2D pt2


================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/Line "a1419010b3fc4549e3f450018363d000")


