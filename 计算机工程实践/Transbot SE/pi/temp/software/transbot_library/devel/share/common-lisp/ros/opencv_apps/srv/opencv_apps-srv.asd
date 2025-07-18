
(cl:in-package :asdf)

(defsystem "opencv_apps-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :opencv_apps-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "FaceRecognitionTrain" :depends-on ("_package_FaceRecognitionTrain"))
    (:file "_package_FaceRecognitionTrain" :depends-on ("_package"))
  ))