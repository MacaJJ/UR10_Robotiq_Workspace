
(cl:in-package :asdf)

(defsystem "detection_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoundingBox" :depends-on ("_package_BoundingBox"))
    (:file "_package_BoundingBox" :depends-on ("_package"))
    (:file "BoundingBox2D" :depends-on ("_package_BoundingBox2D"))
    (:file "_package_BoundingBox2D" :depends-on ("_package"))
    (:file "BoundingBoxes" :depends-on ("_package_BoundingBoxes"))
    (:file "_package_BoundingBoxes" :depends-on ("_package"))
    (:file "Detection2D" :depends-on ("_package_Detection2D"))
    (:file "_package_Detection2D" :depends-on ("_package"))
    (:file "Detection2DArray" :depends-on ("_package_Detection2DArray"))
    (:file "_package_Detection2DArray" :depends-on ("_package"))
    (:file "ObjectHypothesisWithPose" :depends-on ("_package_ObjectHypothesisWithPose"))
    (:file "_package_ObjectHypothesisWithPose" :depends-on ("_package"))
  ))