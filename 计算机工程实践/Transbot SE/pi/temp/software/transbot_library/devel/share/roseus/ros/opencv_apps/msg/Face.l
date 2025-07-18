;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::Face)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'Face (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::FACE")
  (make-package "OPENCV_APPS::FACE"))

(in-package "ROS")
;;//! \htmlinclude Face.msg.html


(defclass opencv_apps::Face
  :super ros::object
  :slots (_face _eyes _label _confidence ))

(defmethod opencv_apps::Face
  (:init
   (&key
    ((:face __face) (instance opencv_apps::Rect :init))
    ((:eyes __eyes) (let (r) (dotimes (i 0) (push (instance opencv_apps::Rect :init) r)) r))
    ((:label __label) "")
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _face __face)
   (setq _eyes __eyes)
   (setq _label (string __label))
   (setq _confidence (float __confidence))
   self)
  (:face
   (&rest __face)
   (if (keywordp (car __face))
       (send* _face __face)
     (progn
       (if __face (setq _face (car __face)))
       _face)))
  (:eyes
   (&rest __eyes)
   (if (keywordp (car __eyes))
       (send* _eyes __eyes)
     (progn
       (if __eyes (setq _eyes (car __eyes)))
       _eyes)))
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; opencv_apps/Rect _face
    (send _face :serialization-length)
    ;; opencv_apps/Rect[] _eyes
    (apply #'+ (send-all _eyes :serialization-length)) 4
    ;; string _label
    4 (length _label)
    ;; float64 _confidence
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencv_apps/Rect _face
       (send _face :serialize s)
     ;; opencv_apps/Rect[] _eyes
     (write-long (length _eyes) s)
     (dolist (elem _eyes)
       (send elem :serialize s)
       )
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float64 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencv_apps/Rect _face
     (send _face :deserialize buf ptr-) (incf ptr- (send _face :serialization-length))
   ;; opencv_apps/Rect[] _eyes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _eyes (let (r) (dotimes (i n) (push (instance opencv_apps::Rect :init) r)) r))
     (dolist (elem- _eyes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _confidence
     (setq _confidence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get opencv_apps::Face :md5sum-) "a1a50e747b0ca7822ce8611c3ffa7a02")
(setf (get opencv_apps::Face :datatype-) "opencv_apps/Face")
(setf (get opencv_apps::Face :definition-)
      "Rect face
Rect[] eyes
string label
float64 confidence

================================================================================
MSG: opencv_apps/Rect
# opencv Rect data type, x-y is center point
float64 x
float64 y
float64 width
float64 height


")



(provide :opencv_apps/Face "a1a50e747b0ca7822ce8611c3ffa7a02")


