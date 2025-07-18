;; Auto-generated. Do not edit!


(when (boundp 'opencv_apps::Moment)
  (if (not (find-package "OPENCV_APPS"))
    (make-package "OPENCV_APPS"))
  (shadow 'Moment (find-package "OPENCV_APPS")))
(unless (find-package "OPENCV_APPS::MOMENT")
  (make-package "OPENCV_APPS::MOMENT"))

(in-package "ROS")
;;//! \htmlinclude Moment.msg.html


(defclass opencv_apps::Moment
  :super ros::object
  :slots (_m00 _m10 _m01 _m20 _m11 _m02 _m30 _m21 _m12 _m03 _mu20 _mu11 _mu02 _mu30 _mu21 _mu12 _mu03 _nu20 _nu11 _nu02 _nu30 _nu21 _nu12 _nu03 _center _length _area ))

(defmethod opencv_apps::Moment
  (:init
   (&key
    ((:m00 __m00) 0.0)
    ((:m10 __m10) 0.0)
    ((:m01 __m01) 0.0)
    ((:m20 __m20) 0.0)
    ((:m11 __m11) 0.0)
    ((:m02 __m02) 0.0)
    ((:m30 __m30) 0.0)
    ((:m21 __m21) 0.0)
    ((:m12 __m12) 0.0)
    ((:m03 __m03) 0.0)
    ((:mu20 __mu20) 0.0)
    ((:mu11 __mu11) 0.0)
    ((:mu02 __mu02) 0.0)
    ((:mu30 __mu30) 0.0)
    ((:mu21 __mu21) 0.0)
    ((:mu12 __mu12) 0.0)
    ((:mu03 __mu03) 0.0)
    ((:nu20 __nu20) 0.0)
    ((:nu11 __nu11) 0.0)
    ((:nu02 __nu02) 0.0)
    ((:nu30 __nu30) 0.0)
    ((:nu21 __nu21) 0.0)
    ((:nu12 __nu12) 0.0)
    ((:nu03 __nu03) 0.0)
    ((:center __center) (instance opencv_apps::Point2D :init))
    ((:length __length) 0.0)
    ((:area __area) 0.0)
    )
   (send-super :init)
   (setq _m00 (float __m00))
   (setq _m10 (float __m10))
   (setq _m01 (float __m01))
   (setq _m20 (float __m20))
   (setq _m11 (float __m11))
   (setq _m02 (float __m02))
   (setq _m30 (float __m30))
   (setq _m21 (float __m21))
   (setq _m12 (float __m12))
   (setq _m03 (float __m03))
   (setq _mu20 (float __mu20))
   (setq _mu11 (float __mu11))
   (setq _mu02 (float __mu02))
   (setq _mu30 (float __mu30))
   (setq _mu21 (float __mu21))
   (setq _mu12 (float __mu12))
   (setq _mu03 (float __mu03))
   (setq _nu20 (float __nu20))
   (setq _nu11 (float __nu11))
   (setq _nu02 (float __nu02))
   (setq _nu30 (float __nu30))
   (setq _nu21 (float __nu21))
   (setq _nu12 (float __nu12))
   (setq _nu03 (float __nu03))
   (setq _center __center)
   (setq _length (float __length))
   (setq _area (float __area))
   self)
  (:m00
   (&optional __m00)
   (if __m00 (setq _m00 __m00)) _m00)
  (:m10
   (&optional __m10)
   (if __m10 (setq _m10 __m10)) _m10)
  (:m01
   (&optional __m01)
   (if __m01 (setq _m01 __m01)) _m01)
  (:m20
   (&optional __m20)
   (if __m20 (setq _m20 __m20)) _m20)
  (:m11
   (&optional __m11)
   (if __m11 (setq _m11 __m11)) _m11)
  (:m02
   (&optional __m02)
   (if __m02 (setq _m02 __m02)) _m02)
  (:m30
   (&optional __m30)
   (if __m30 (setq _m30 __m30)) _m30)
  (:m21
   (&optional __m21)
   (if __m21 (setq _m21 __m21)) _m21)
  (:m12
   (&optional __m12)
   (if __m12 (setq _m12 __m12)) _m12)
  (:m03
   (&optional __m03)
   (if __m03 (setq _m03 __m03)) _m03)
  (:mu20
   (&optional __mu20)
   (if __mu20 (setq _mu20 __mu20)) _mu20)
  (:mu11
   (&optional __mu11)
   (if __mu11 (setq _mu11 __mu11)) _mu11)
  (:mu02
   (&optional __mu02)
   (if __mu02 (setq _mu02 __mu02)) _mu02)
  (:mu30
   (&optional __mu30)
   (if __mu30 (setq _mu30 __mu30)) _mu30)
  (:mu21
   (&optional __mu21)
   (if __mu21 (setq _mu21 __mu21)) _mu21)
  (:mu12
   (&optional __mu12)
   (if __mu12 (setq _mu12 __mu12)) _mu12)
  (:mu03
   (&optional __mu03)
   (if __mu03 (setq _mu03 __mu03)) _mu03)
  (:nu20
   (&optional __nu20)
   (if __nu20 (setq _nu20 __nu20)) _nu20)
  (:nu11
   (&optional __nu11)
   (if __nu11 (setq _nu11 __nu11)) _nu11)
  (:nu02
   (&optional __nu02)
   (if __nu02 (setq _nu02 __nu02)) _nu02)
  (:nu30
   (&optional __nu30)
   (if __nu30 (setq _nu30 __nu30)) _nu30)
  (:nu21
   (&optional __nu21)
   (if __nu21 (setq _nu21 __nu21)) _nu21)
  (:nu12
   (&optional __nu12)
   (if __nu12 (setq _nu12 __nu12)) _nu12)
  (:nu03
   (&optional __nu03)
   (if __nu03 (setq _nu03 __nu03)) _nu03)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:area
   (&optional __area)
   (if __area (setq _area __area)) _area)
  (:serialization-length
   ()
   (+
    ;; float64 _m00
    8
    ;; float64 _m10
    8
    ;; float64 _m01
    8
    ;; float64 _m20
    8
    ;; float64 _m11
    8
    ;; float64 _m02
    8
    ;; float64 _m30
    8
    ;; float64 _m21
    8
    ;; float64 _m12
    8
    ;; float64 _m03
    8
    ;; float64 _mu20
    8
    ;; float64 _mu11
    8
    ;; float64 _mu02
    8
    ;; float64 _mu30
    8
    ;; float64 _mu21
    8
    ;; float64 _mu12
    8
    ;; float64 _mu03
    8
    ;; float64 _nu20
    8
    ;; float64 _nu11
    8
    ;; float64 _nu02
    8
    ;; float64 _nu30
    8
    ;; float64 _nu21
    8
    ;; float64 _nu12
    8
    ;; float64 _nu03
    8
    ;; opencv_apps/Point2D _center
    (send _center :serialization-length)
    ;; float64 _length
    8
    ;; float64 _area
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _m00
       (sys::poke _m00 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m10
       (sys::poke _m10 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m01
       (sys::poke _m01 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m20
       (sys::poke _m20 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m11
       (sys::poke _m11 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m02
       (sys::poke _m02 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m30
       (sys::poke _m30 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m21
       (sys::poke _m21 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m12
       (sys::poke _m12 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m03
       (sys::poke _m03 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mu20
       (sys::poke _mu20 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mu11
       (sys::poke _mu11 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mu02
       (sys::poke _mu02 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mu30
       (sys::poke _mu30 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mu21
       (sys::poke _mu21 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mu12
       (sys::poke _mu12 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mu03
       (sys::poke _mu03 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _nu20
       (sys::poke _nu20 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _nu11
       (sys::poke _nu11 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _nu02
       (sys::poke _nu02 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _nu30
       (sys::poke _nu30 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _nu21
       (sys::poke _nu21 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _nu12
       (sys::poke _nu12 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _nu03
       (sys::poke _nu03 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; opencv_apps/Point2D _center
       (send _center :serialize s)
     ;; float64 _length
       (sys::poke _length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _area
       (sys::poke _area (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _m00
     (setq _m00 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m10
     (setq _m10 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m01
     (setq _m01 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m20
     (setq _m20 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m11
     (setq _m11 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m02
     (setq _m02 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m30
     (setq _m30 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m21
     (setq _m21 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m12
     (setq _m12 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m03
     (setq _m03 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mu20
     (setq _mu20 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mu11
     (setq _mu11 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mu02
     (setq _mu02 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mu30
     (setq _mu30 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mu21
     (setq _mu21 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mu12
     (setq _mu12 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mu03
     (setq _mu03 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _nu20
     (setq _nu20 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _nu11
     (setq _nu11 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _nu02
     (setq _nu02 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _nu30
     (setq _nu30 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _nu21
     (setq _nu21 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _nu12
     (setq _nu12 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _nu03
     (setq _nu03 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; opencv_apps/Point2D _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; float64 _length
     (setq _length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _area
     (setq _area (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get opencv_apps::Moment :md5sum-) "560ee3fabfffb4ed4155742d6db8a03c")
(setf (get opencv_apps::Moment :datatype-) "opencv_apps/Moment")
(setf (get opencv_apps::Moment :definition-)
      "# spatial moments
float64 m00
float64 m10
float64 m01
float64 m20
float64 m11
float64 m02
float64 m30
float64 m21
float64 m12
float64 m03

# central moments
float64 mu20
float64 mu11
float64 mu02
float64 mu30
float64 mu21
float64 mu12
float64 mu03

# central normalized moments
float64 nu20
float64 nu11
float64 nu02
float64 nu30
float64 nu21
float64 nu12
float64 nu03

# center of mass m10/m00, m01/m00
Point2D center
float64 length
float64 area

================================================================================
MSG: opencv_apps/Point2D
float64 x
float64 y


")



(provide :opencv_apps/Moment "560ee3fabfffb4ed4155742d6db8a03c")


