;; Auto-generated. Do not edit!


(when (boundp 'view_detect::YoloResult)
  (if (not (find-package "VIEW_DETECT"))
    (make-package "VIEW_DETECT"))
  (shadow 'YoloResult (find-package "VIEW_DETECT")))
(unless (find-package "VIEW_DETECT::YOLORESULT")
  (make-package "VIEW_DETECT::YOLORESULT"))

(in-package "ROS")
;;//! \htmlinclude YoloResult.msg.html


(defclass view_detect::YoloResult
  :super ros::object
  :slots (_image_id _class_names _counts ))

(defmethod view_detect::YoloResult
  (:init
   (&key
    ((:image_id __image_id) "")
    ((:class_names __class_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:counts __counts) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _image_id (string __image_id))
   (setq _class_names __class_names)
   (setq _counts __counts)
   self)
  (:image_id
   (&optional __image_id)
   (if __image_id (setq _image_id __image_id)) _image_id)
  (:class_names
   (&optional __class_names)
   (if __class_names (setq _class_names __class_names)) _class_names)
  (:counts
   (&optional __counts)
   (if __counts (setq _counts __counts)) _counts)
  (:serialization-length
   ()
   (+
    ;; string _image_id
    4 (length _image_id)
    ;; string[] _class_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _class_names)) 4
    ;; int32[] _counts
    (* 4    (length _counts)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _image_id
       (write-long (length _image_id) s) (princ _image_id s)
     ;; string[] _class_names
     (write-long (length _class_names) s)
     (dolist (elem _class_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; int32[] _counts
     (write-long (length _counts) s)
     (dotimes (i (length _counts))
       (write-long (elt _counts i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _image_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _image_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _class_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _class_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _class_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; int32[] _counts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _counts (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _counts i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get view_detect::YoloResult :md5sum-) "52917fd0d63c30fa5cbba61ad8a19792")
(setf (get view_detect::YoloResult :datatype-) "view_detect/YoloResult")
(setf (get view_detect::YoloResult :definition-)
      "# YOLO检测结果消息
string image_id        # 图像ID或文件名
string[] class_names   # 检测到的类别名称
int32[] counts         # 每个类别的数量 
")



(provide :view_detect/YoloResult "52917fd0d63c30fa5cbba61ad8a19792")


