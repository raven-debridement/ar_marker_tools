
(cl:in-package :asdf)

(defsystem "marker_detect-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MarkerInfos" :depends-on ("_package_MarkerInfos"))
    (:file "_package_MarkerInfos" :depends-on ("_package"))
    (:file "MarkerXYs" :depends-on ("_package_MarkerXYs"))
    (:file "_package_MarkerXYs" :depends-on ("_package"))
  ))