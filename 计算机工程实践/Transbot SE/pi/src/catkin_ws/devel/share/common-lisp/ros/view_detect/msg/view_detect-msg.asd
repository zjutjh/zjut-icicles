
(cl:in-package :asdf)

(defsystem "view_detect-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "YoloResult" :depends-on ("_package_YoloResult"))
    (:file "_package_YoloResult" :depends-on ("_package"))
  ))