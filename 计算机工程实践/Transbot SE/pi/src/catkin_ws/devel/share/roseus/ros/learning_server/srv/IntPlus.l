;; Auto-generated. Do not edit!


(when (boundp 'learning_server::IntPlus)
  (if (not (find-package "LEARNING_SERVER"))
    (make-package "LEARNING_SERVER"))
  (shadow 'IntPlus (find-package "LEARNING_SERVER")))
(unless (find-package "LEARNING_SERVER::INTPLUS")
  (make-package "LEARNING_SERVER::INTPLUS"))
(unless (find-package "LEARNING_SERVER::INTPLUSREQUEST")
  (make-package "LEARNING_SERVER::INTPLUSREQUEST"))
(unless (find-package "LEARNING_SERVER::INTPLUSRESPONSE")
  (make-package "LEARNING_SERVER::INTPLUSRESPONSE"))

(in-package "ROS")





(defclass learning_server::IntPlusRequest
  :super ros::object
  :slots (_a _b ))

(defmethod learning_server::IntPlusRequest
  (:init
   (&key
    ((:a __a) 0)
    ((:b __b) 0)
    )
   (send-super :init)
   (setq _a (round __a))
   (setq _b (round __b))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; uint8 _a
    1
    ;; uint8 _b
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _a
       (write-byte _a s)
     ;; uint8 _b
       (write-byte _b s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _a
     (setq _a (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _b
     (setq _b (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass learning_server::IntPlusResponse
  :super ros::object
  :slots (_result ))

(defmethod learning_server::IntPlusResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; uint8 _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _result
       (write-byte _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _result
     (setq _result (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass learning_server::IntPlus
  :super ros::object
  :slots ())

(setf (get learning_server::IntPlus :md5sum-) "32f3e69fccd784eebfbfc5b37be0e2f0")
(setf (get learning_server::IntPlus :datatype-) "learning_server/IntPlus")
(setf (get learning_server::IntPlus :request) learning_server::IntPlusRequest)
(setf (get learning_server::IntPlus :response) learning_server::IntPlusResponse)

(defmethod learning_server::IntPlusRequest
  (:response () (instance learning_server::IntPlusResponse :init)))

(setf (get learning_server::IntPlusRequest :md5sum-) "32f3e69fccd784eebfbfc5b37be0e2f0")
(setf (get learning_server::IntPlusRequest :datatype-) "learning_server/IntPlusRequest")
(setf (get learning_server::IntPlusRequest :definition-)
      "uint8  a
uint8  b

---
uint8 result

")

(setf (get learning_server::IntPlusResponse :md5sum-) "32f3e69fccd784eebfbfc5b37be0e2f0")
(setf (get learning_server::IntPlusResponse :datatype-) "learning_server/IntPlusResponse")
(setf (get learning_server::IntPlusResponse :definition-)
      "uint8  a
uint8  b

---
uint8 result

")



(provide :learning_server/IntPlus "32f3e69fccd784eebfbfc5b37be0e2f0")


