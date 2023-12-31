;; Auto-generated. Do not edit!


(when (boundp 'raptor_dbw_msgs::SonarArcNum)
  (if (not (find-package "RAPTOR_DBW_MSGS"))
    (make-package "RAPTOR_DBW_MSGS"))
  (shadow 'SonarArcNum (find-package "RAPTOR_DBW_MSGS")))
(unless (find-package "RAPTOR_DBW_MSGS::SONARARCNUM")
  (make-package "RAPTOR_DBW_MSGS::SONARARCNUM"))

(in-package "ROS")
;;//! \htmlinclude SonarArcNum.msg.html


(intern "*NO_DISTANCE*" (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(shadow '*NO_DISTANCE* (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(defconstant raptor_dbw_msgs::SonarArcNum::*NO_DISTANCE* 0)
(intern "*ARC1*" (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(shadow '*ARC1* (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(defconstant raptor_dbw_msgs::SonarArcNum::*ARC1* 1)
(intern "*ARC2*" (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(shadow '*ARC2* (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(defconstant raptor_dbw_msgs::SonarArcNum::*ARC2* 2)
(intern "*ARC3*" (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(shadow '*ARC3* (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(defconstant raptor_dbw_msgs::SonarArcNum::*ARC3* 3)
(intern "*ARC4*" (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(shadow '*ARC4* (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(defconstant raptor_dbw_msgs::SonarArcNum::*ARC4* 4)
(intern "*ARC5*" (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(shadow '*ARC5* (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(defconstant raptor_dbw_msgs::SonarArcNum::*ARC5* 5)
(intern "*ARC6*" (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(shadow '*ARC6* (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(defconstant raptor_dbw_msgs::SonarArcNum::*ARC6* 6)
(intern "*ARC7*" (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(shadow '*ARC7* (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(defconstant raptor_dbw_msgs::SonarArcNum::*ARC7* 7)
(intern "*ARC8*" (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(shadow '*ARC8* (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(defconstant raptor_dbw_msgs::SonarArcNum::*ARC8* 8)
(intern "*NO_ARC*" (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(shadow '*NO_ARC* (find-package "RAPTOR_DBW_MSGS::SONARARCNUM"))
(defconstant raptor_dbw_msgs::SonarArcNum::*NO_ARC* 15)

(defun raptor_dbw_msgs::SonarArcNum-to-symbol (const)
  (cond
        ((= const 0) 'raptor_dbw_msgs::SonarArcNum::*NO_DISTANCE*)
        ((= const 1) 'raptor_dbw_msgs::SonarArcNum::*ARC1*)
        ((= const 2) 'raptor_dbw_msgs::SonarArcNum::*ARC2*)
        ((= const 3) 'raptor_dbw_msgs::SonarArcNum::*ARC3*)
        ((= const 4) 'raptor_dbw_msgs::SonarArcNum::*ARC4*)
        ((= const 5) 'raptor_dbw_msgs::SonarArcNum::*ARC5*)
        ((= const 6) 'raptor_dbw_msgs::SonarArcNum::*ARC6*)
        ((= const 7) 'raptor_dbw_msgs::SonarArcNum::*ARC7*)
        ((= const 8) 'raptor_dbw_msgs::SonarArcNum::*ARC8*)
        ((= const 15) 'raptor_dbw_msgs::SonarArcNum::*NO_ARC*)
        (t nil)))

(defclass raptor_dbw_msgs::SonarArcNum
  :super ros::object
  :slots (_status ))

(defmethod raptor_dbw_msgs::SonarArcNum
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; uint8 _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get raptor_dbw_msgs::SonarArcNum :md5sum-) "06cc406551462e830a15f0d81395d2f4")
(setf (get raptor_dbw_msgs::SonarArcNum :datatype-) "raptor_dbw_msgs/SonarArcNum")
(setf (get raptor_dbw_msgs::SonarArcNum :definition-)
      "uint8 status

# Unitless - based on bars on display
uint8 NO_DISTANCE = 0
uint8 ARC1 = 1
uint8 ARC2 = 2
uint8 ARC3 = 3
uint8 ARC4 = 4
uint8 ARC5 = 5
uint8 ARC6 = 6
uint8 ARC7 = 7
uint8 ARC8 = 8
uint8 NO_ARC = 15

")



(provide :raptor_dbw_msgs/SonarArcNum "06cc406551462e830a15f0d81395d2f4")


