;; Auto-generated. Do not edit!


(when (boundp 'yahboomcar_msgs::Image_Msg)
  (if (not (find-package "YAHBOOMCAR_MSGS"))
    (make-package "YAHBOOMCAR_MSGS"))
  (shadow 'Image_Msg (find-package "YAHBOOMCAR_MSGS")))
(unless (find-package "YAHBOOMCAR_MSGS::IMAGE_MSG")
  (make-package "YAHBOOMCAR_MSGS::IMAGE_MSG"))

(in-package "ROS")
;;//! \htmlinclude Image_Msg.msg.html


(defclass yahboomcar_msgs::Image_Msg
  :super ros::object
  :slots (_height _width _channels _data ))

(defmethod yahboomcar_msgs::Image_Msg
  (:init
   (&key
    ((:height __height) 0)
    ((:width __width) 0)
    ((:channels __channels) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _height (round __height))
   (setq _width (round __width))
   (setq _channels (round __channels))
   (setq _data __data)
   self)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:channels
   (&optional __channels)
   (if __channels (setq _channels __channels)) _channels)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32 _height
    4
    ;; int32 _width
    4
    ;; int32 _channels
    4
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _height
       (write-long _height s)
     ;; int32 _width
       (write-long _width s)
     ;; int32 _channels
       (write-long _channels s)
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _channels
     (setq _channels (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get yahboomcar_msgs::Image_Msg :md5sum-) "33648d31a5c2999db4993f94515b68ee")
(setf (get yahboomcar_msgs::Image_Msg :datatype-) "yahboomcar_msgs/Image_Msg")
(setf (get yahboomcar_msgs::Image_Msg :definition-)
      "int32 height
int32 width
int32 channels
uint8[] data
")



(provide :yahboomcar_msgs/Image_Msg "33648d31a5c2999db4993f94515b68ee")


