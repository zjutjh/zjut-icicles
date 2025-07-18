;; Auto-generated. Do not edit!


(when (boundp 'learning_topic::Information)
  (if (not (find-package "LEARNING_TOPIC"))
    (make-package "LEARNING_TOPIC"))
  (shadow 'Information (find-package "LEARNING_TOPIC")))
(unless (find-package "LEARNING_TOPIC::INFORMATION")
  (make-package "LEARNING_TOPIC::INFORMATION"))

(in-package "ROS")
;;//! \htmlinclude Information.msg.html


(defclass learning_topic::Information
  :super ros::object
  :slots (_company _city ))

(defmethod learning_topic::Information
  (:init
   (&key
    ((:company __company) "")
    ((:city __city) "")
    )
   (send-super :init)
   (setq _company (string __company))
   (setq _city (string __city))
   self)
  (:company
   (&optional __company)
   (if __company (setq _company __company)) _company)
  (:city
   (&optional __city)
   (if __city (setq _city __city)) _city)
  (:serialization-length
   ()
   (+
    ;; string _company
    4 (length _company)
    ;; string _city
    4 (length _city)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _company
       (write-long (length _company) s) (princ _company s)
     ;; string _city
       (write-long (length _city) s) (princ _city s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _company
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _company (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _city
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _city (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get learning_topic::Information :md5sum-) "6f9332e7eae53dbcb74dc13ad7572af4")
(setf (get learning_topic::Information :datatype-) "learning_topic/Information")
(setf (get learning_topic::Information :definition-)
      "string company
string city  

")



(provide :learning_topic/Information "6f9332e7eae53dbcb74dc13ad7572af4")


