;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::General)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'General (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::GENERAL")
  (make-package "TRANSBOT_MSGS::GENERAL"))

(in-package "ROS")
;;//! \htmlinclude General.msg.html


(defclass transbot_msgs::General
  :super ros::object
  :slots (_Graphics _TrackState ))

(defmethod transbot_msgs::General
  (:init
   (&key
    ((:Graphics __Graphics) "")
    ((:TrackState __TrackState) "")
    )
   (send-super :init)
   (setq _Graphics (string __Graphics))
   (setq _TrackState (string __TrackState))
   self)
  (:Graphics
   (&optional __Graphics)
   (if __Graphics (setq _Graphics __Graphics)) _Graphics)
  (:TrackState
   (&optional __TrackState)
   (if __TrackState (setq _TrackState __TrackState)) _TrackState)
  (:serialization-length
   ()
   (+
    ;; string _Graphics
    4 (length _Graphics)
    ;; string _TrackState
    4 (length _TrackState)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Graphics
       (write-long (length _Graphics) s) (princ _Graphics s)
     ;; string _TrackState
       (write-long (length _TrackState) s) (princ _TrackState s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Graphics
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Graphics (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _TrackState
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _TrackState (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get transbot_msgs::General :md5sum-) "17ceb36a2d5cf93a882109ffc0506c61")
(setf (get transbot_msgs::General :datatype-) "transbot_msgs/General")
(setf (get transbot_msgs::General :definition-)
      "string Graphics
string TrackState
")



(provide :transbot_msgs/General "17ceb36a2d5cf93a882109ffc0506c61")


