;; Auto-generated. Do not edit!


(when (boundp 'transbot_msgs::SensorState)
  (if (not (find-package "TRANSBOT_MSGS"))
    (make-package "TRANSBOT_MSGS"))
  (shadow 'SensorState (find-package "TRANSBOT_MSGS")))
(unless (find-package "TRANSBOT_MSGS::SENSORSTATE")
  (make-package "TRANSBOT_MSGS::SENSORSTATE"))

(in-package "ROS")
;;//! \htmlinclude SensorState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*BUMPER_FORWARD*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*BUMPER_FORWARD* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*BUMPER_FORWARD* 1)
(intern "*BUMPER_BACKWARD*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*BUMPER_BACKWARD* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*BUMPER_BACKWARD* 2)
(intern "*CLIFF*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*CLIFF* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*CLIFF* 1)
(intern "*SONAR*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*SONAR* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*SONAR* 1)
(intern "*ILLUMINATION*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*ILLUMINATION* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*ILLUMINATION* 1)
(intern "*BUTTON0*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*BUTTON0* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*BUTTON0* 1)
(intern "*BUTTON1*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*BUTTON1* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*BUTTON1* 2)
(intern "*ERROR_LEFT_MOTOR*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*ERROR_LEFT_MOTOR* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*ERROR_LEFT_MOTOR* 1)
(intern "*ERROR_RIGHT_MOTOR*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*ERROR_RIGHT_MOTOR* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*ERROR_RIGHT_MOTOR* 2)
(intern "*TORQUE_ON*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*TORQUE_ON* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*TORQUE_ON* 1)
(intern "*TORQUE_OFF*" (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(shadow '*TORQUE_OFF* (find-package "TRANSBOT_MSGS::SENSORSTATE"))
(defconstant transbot_msgs::SensorState::*TORQUE_OFF* 2)
(defclass transbot_msgs::SensorState
  :super ros::object
  :slots (_header _bumper _cliff _sonar _illumination _led _button _torque _left_encoder _right_encoder _battery ))

(defmethod transbot_msgs::SensorState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bumper __bumper) 0)
    ((:cliff __cliff) 0.0)
    ((:sonar __sonar) 0.0)
    ((:illumination __illumination) 0.0)
    ((:led __led) 0)
    ((:button __button) 0)
    ((:torque __torque) nil)
    ((:left_encoder __left_encoder) 0)
    ((:right_encoder __right_encoder) 0)
    ((:battery __battery) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _bumper (round __bumper))
   (setq _cliff (float __cliff))
   (setq _sonar (float __sonar))
   (setq _illumination (float __illumination))
   (setq _led (round __led))
   (setq _button (round __button))
   (setq _torque __torque)
   (setq _left_encoder (round __left_encoder))
   (setq _right_encoder (round __right_encoder))
   (setq _battery (float __battery))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bumper
   (&optional __bumper)
   (if __bumper (setq _bumper __bumper)) _bumper)
  (:cliff
   (&optional __cliff)
   (if __cliff (setq _cliff __cliff)) _cliff)
  (:sonar
   (&optional __sonar)
   (if __sonar (setq _sonar __sonar)) _sonar)
  (:illumination
   (&optional __illumination)
   (if __illumination (setq _illumination __illumination)) _illumination)
  (:led
   (&optional __led)
   (if __led (setq _led __led)) _led)
  (:button
   (&optional __button)
   (if __button (setq _button __button)) _button)
  (:torque
   (&optional __torque)
   (if __torque (setq _torque __torque)) _torque)
  (:left_encoder
   (&optional __left_encoder)
   (if __left_encoder (setq _left_encoder __left_encoder)) _left_encoder)
  (:right_encoder
   (&optional __right_encoder)
   (if __right_encoder (setq _right_encoder __right_encoder)) _right_encoder)
  (:battery
   (&optional __battery)
   (if __battery (setq _battery __battery)) _battery)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _bumper
    1
    ;; float32 _cliff
    4
    ;; float32 _sonar
    4
    ;; float32 _illumination
    4
    ;; uint8 _led
    1
    ;; uint8 _button
    1
    ;; bool _torque
    1
    ;; int32 _left_encoder
    4
    ;; int32 _right_encoder
    4
    ;; float32 _battery
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _bumper
       (write-byte _bumper s)
     ;; float32 _cliff
       (sys::poke _cliff (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sonar
       (sys::poke _sonar (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _illumination
       (sys::poke _illumination (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _led
       (write-byte _led s)
     ;; uint8 _button
       (write-byte _button s)
     ;; bool _torque
       (if _torque (write-byte -1 s) (write-byte 0 s))
     ;; int32 _left_encoder
       (write-long _left_encoder s)
     ;; int32 _right_encoder
       (write-long _right_encoder s)
     ;; float32 _battery
       (sys::poke _battery (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _bumper
     (setq _bumper (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _cliff
     (setq _cliff (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sonar
     (setq _sonar (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _illumination
     (setq _illumination (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _led
     (setq _led (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _button
     (setq _button (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _torque
     (setq _torque (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _left_encoder
     (setq _left_encoder (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _right_encoder
     (setq _right_encoder (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _battery
     (setq _battery (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get transbot_msgs::SensorState :md5sum-) "7250c1dc0b61c4190e78f528f599285f")
(setf (get transbot_msgs::SensorState :datatype-) "transbot_msgs/SensorState")
(setf (get transbot_msgs::SensorState :definition-)
      "########################################
# CONSTANTS
########################################
# Bumper states (states are combined, when multiple bumpers are pressed)
uint8 BUMPER_FORWARD  = 1
uint8 BUMPER_BACKWARD = 2

# Cliff sensor states (states are combined, when multiple cliff sensors are triggered)
uint8 CLIFF = 1

# Sonar sensor states (states are combined, when multiple sonar sensors are triggered)
uint8 SONAR = 1

# Illumination sensor (states are combined, when multiple illumination sensors are triggered) 
uint8 ILLUMINATION = 1

# Button states (states are combined, when multiple buttons are pressed)
uint8 BUTTON0 = 1
uint8 BUTTON1 = 2

# Motor errors
uint8 ERROR_LEFT_MOTOR  = 1
uint8 ERROR_RIGHT_MOTOR = 2

# Motor torque
uint8 TORQUE_ON  = 1
uint8 TORQUE_OFF = 2

########################################
# Messages
########################################
Header  header
uint8   bumper
float32 cliff
float32 sonar
float32 illumination
uint8   led
uint8  button
bool   torque
int32  left_encoder    # (-2,147,483,648 ~ 2,147,483,647)
int32  right_encoder   # (-2,147,483,648 ~ 2,147,483,647)
float32  battery

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :transbot_msgs/SensorState "7250c1dc0b61c4190e78f528f599285f")


