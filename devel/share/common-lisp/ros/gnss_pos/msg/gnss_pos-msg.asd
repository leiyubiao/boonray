
(cl:in-package :asdf)

(defsystem "gnss_pos-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "pos_xy" :depends-on ("_package_pos_xy"))
    (:file "_package_pos_xy" :depends-on ("_package"))
  ))