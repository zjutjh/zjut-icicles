;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::Joint)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'Joint (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::JOINT")
  (make-package "TRANSBOT_MSGS::JOINT"))

(in-package "ROS")
;;//! \htmlinclude Joint.msg.html


(defclass transbot_msgs::Joint
  :super ros::object
  :slots (_id _run_time _angle ))

(defmethod transbot_msgs::Joint
  (:init
   (&key
    ((:id __id) 0)
    ((:run_time __run_time) 0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _run_time (round __run_time))
   (setq _angle (float __angle))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:run_time
   (&optional __run_time)
   (if __run_time (setq _run_time __run_time)) _run_time)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; int32 _run_time
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; int32 _run_time
       (write-long _run_time s)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _run_time
     (setq _run_time (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get transbot_msgs::Joint :md5sum-) "e4bdd2abb31a24fd4ffe1e13a32f2cb6")
(setf (get transbot_msgs::Joint :datatype-) "transbot_msgs/Joint")
(setf (get transbot_msgs::Joint :definition-)
      "int32 id
int32 run_time
float32 angle

")



(provide :transbot_msgs/Joint "e4bdd2abb31a24fd4ffe1e13a32f2cb6")


