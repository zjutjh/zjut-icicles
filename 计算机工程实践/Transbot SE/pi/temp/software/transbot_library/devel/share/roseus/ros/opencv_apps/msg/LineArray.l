;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::LineArray)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'LineArray (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::LINEARRAY")
  (make-package "OPENCV_APPS::LINEARRAY"))

(in-package "ROS")
;;//! \htmlinclude LineArray.msg.html


(defclass opencv_apps::LineArray
  :super ros::object
  :slots (_lines ))

(defmethod opencv_apps::LineArray
  (:init
   (&key
    ((:lines __lines) (let (r) (dotimes (i 0) (push (instance opencv_apps::Line :init) r)) r))
    )
   (send-super :init)
   (setq _lines __lines)
   self)
  (:lines
   (&rest __lines)
   (if (keywordp (car __lines))
       (send* _lines __lines)
     (progn
       (if __lines (setq _lines (car __lines)))
       _lines)))
  (:serialization-length
   ()
   (+
    ;; opencv_apps/Line[] _lines
    (apply #'+ (send-all _lines :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Line[] _lines
     (write-long (length _lines) s)
     (dolist (elem _lines)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Line[] _lines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lines (let (r) (dotimes (i n) (push (instance opencv_apps::Line :init) r)) r))
     (dolist (elem- _lines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::LineArray :md5sum-) "2b5441933900cc71528395dda29124da")
(setf (get opencv_apps::LineArray :datatype-) "opencv_apps/LineArray")
(setf (get opencv_apps::LineArray :definition-)
      "Line[] lines

================================================================================
MSG: opencv_apps/Line
Point2D pt1
Point2D pt2


================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/LineArray "2b5441933900cc71528395dda29124da")


