;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::FlowArray)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'FlowArray (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::FLOWARRAY")
  (make-package "OPENCV_APPS::FLOWARRAY"))

(in-package "ROS")
;;//! \htmlinclude FlowArray.msg.html


(defclass opencv_apps::FlowArray
  :super ros::object
  :slots (_flow ))

(defmethod opencv_apps::FlowArray
  (:init
   (&key
    ((:flow __flow) (let (r) (dotimes (i 0) (push (instance opencv_apps::Flow :init) r)) r))
    )
   (send-super :init)
   (setq _flow __flow)
   self)
  (:flow
   (&rest __flow)
   (if (keywordp (car __flow))
       (send* _flow __flow)
     (progn
       (if __flow (setq _flow (car __flow)))
       _flow)))
  (:serialization-length
   ()
   (+
    ;; opencv_apps/Flow[] _flow
    (apply #'+ (send-all _flow :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Flow[] _flow
     (write-long (length _flow) s)
     (dolist (elem _flow)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Flow[] _flow
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _flow (let (r) (dotimes (i n) (push (instance opencv_apps::Flow :init) r)) r))
     (dolist (elem- _flow)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get opencv_apps::FlowArray :md5sum-) "fe292dca56eb3673cd698ea9ef841962")
(setf (get opencv_apps::FlowArray :datatype-) "opencv_apps/FlowArray")
(setf (get opencv_apps::FlowArray :definition-)
      "Flow[] flow

================================================================================
MSG: opencv_apps/Flow
Point2D point
Point2D velocity

================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/FlowArray "fe292dca56eb3673cd698ea9ef841962")


