
(cl:in-package :asdf)

(defsystem "deeporange14_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AuStatusMsg" :depends-on ("_package_AuStatusMsg"))
    (:file "_package_AuStatusMsg" :depends-on ("_package"))
    (:file "MissionStatus" :depends-on ("_package_MissionStatus"))
    (:file "_package_MissionStatus" :depends-on ("_package"))
    (:file "MobilityMsg" :depends-on ("_package_MobilityMsg"))
    (:file "_package_MobilityMsg" :depends-on ("_package"))
    (:file "PIDComponentsMsg" :depends-on ("_package_PIDComponentsMsg"))
    (:file "_package_PIDComponentsMsg" :depends-on ("_package"))
    (:file "RaptorStateMsg" :depends-on ("_package_RaptorStateMsg"))
    (:file "_package_RaptorStateMsg" :depends-on ("_package"))
    (:file "TorqueCmdStamped" :depends-on ("_package_TorqueCmdStamped"))
    (:file "_package_TorqueCmdStamped" :depends-on ("_package"))
  ))