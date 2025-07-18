;; Auto-generated. Do not edit!


(when (boundp 'yahboomcar_msgs::Target)
  (if (not (find-package "YAHBOOMCAR_MSGS"))
    (make-package "YAHBOOMCAR_MSGS"))
  (shadow 'Target (find-package "YAHBOOMCAR_MSGS")))
(unless (find-package "YAHBOOMCAR_MSGS::TARGET")
  (make-package "YAHBOOMCAR_MSGS::TARGET"))

(in-package "ROS")
;;//! \htmlinclude Target.msg.html


(defclass yahboomcar_msgs::Target
  :super ros::object
  :slots (_frame_id _stamp _scores _ptx _pty _distw _disth _centerx _centery ))

(defmethod yahboomcar_msgs::Target
  (:init
   (&key
    ((:frame_id __frame_id) "")
    ((:stamp __stamp) (instance ros::time :init))
    ((:scores __scores) 0.0)
    ((:ptx __ptx) 0.0)
    ((:pty __pty) 0.0)
    ((:distw __distw) 0.0)
    ((:disth __disth) 0.0)
    ((:centerx __centerx) 0.0)
    ((:centery __centery) 0.0)
    )
   (send-super :init)
   (setq _frame_id (string __frame_id))
   (setq _stamp __stamp)
   (setq _scores (float __scores))
   (setq _ptx (float __ptx))
   (setq _pty (float __pty))
   (setq _distw (float __distw))
   (setq _disth (float __disth))
   (setq _centerx (float __centerx))
   (setq _centery (float __centery))
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:scores
   (&optional __scores)
   (if __scores (setq _scores __scores)) _scores)
  (:ptx
   (&optional __ptx)
   (if __ptx (setq _ptx __ptx)) _ptx)
  (:pty
   (&optional __pty)
   (if __pty (setq _pty __pty)) _pty)
  (:distw
   (&optional __distw)
   (if __distw (setq _distw __distw)) _distw)
  (:disth
   (&optional __disth)
   (if __disth (setq _disth __disth)) _disth)
  (:centerx
   (&optional __centerx)
   (if __centerx (setq _centerx __centerx)) _centerx)
  (:centery
   (&optional __centery)
   (if __centery (setq _centery __centery)) _centery)
  (:serialization-length
   ()
   (+
    ;; string _frame_id
    4 (length _frame_id)
    ;; time _stamp
    8
    ;; float32 _scores
    4
    ;; float32 _ptx
    4
    ;; float32 _pty
    4
    ;; float32 _distw
    4
    ;; float32 _disth
    4
    ;; float32 _centerx
    4
    ;; float32 _centery
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32 _scores
       (sys::poke _scores (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ptx
       (sys::poke _ptx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pty
       (sys::poke _pty (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distw
       (sys::poke _distw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _disth
       (sys::poke _disth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _centerx
       (sys::poke _centerx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _centery
       (sys::poke _centery (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _scores
     (setq _scores (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ptx
     (setq _ptx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pty
     (setq _pty (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distw
     (setq _distw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _disth
     (setq _disth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _centerx
     (setq _centerx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _centery
     (setq _centery (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yahboomcar_msgs::Target :md5sum-) "ed563213df8d0c63d0090b5a0b306f53")
(setf (get yahboomcar_msgs::Target :datatype-) "yahboomcar_msgs/Target")
(setf (get yahboomcar_msgs::Target :definition-)
      "string frame_id
time stamp
float32 scores
float32 ptx
float32 pty
float32 distw
float32 disth
float32 centerx
float32 centery

")



(provide :yahboomcar_msgs/Target "ed563213df8d0c63d0090b5a0b306f53")


