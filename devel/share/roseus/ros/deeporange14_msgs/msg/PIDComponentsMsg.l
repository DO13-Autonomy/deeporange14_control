;; Auto-generated. Do not edit!


(when (boundp 'deeporange14_msgs::PIDComponentsMsg)
  (if (not (find-package "DEEPORANGE14_MSGS"))
    (make-package "DEEPORANGE14_MSGS"))
  (shadow 'PIDComponentsMsg (find-package "DEEPORANGE14_MSGS")))
(unless (find-package "DEEPORANGE14_MSGS::PIDCOMPONENTSMSG")
  (make-package "DEEPORANGE14_MSGS::PIDCOMPONENTSMSG"))

(in-package "ROS")
;;//! \htmlinclude PIDComponentsMsg.msg.html


(defclass deeporange14_msgs::PIDComponentsMsg
  :super ros::object
  :slots (_P_Vx _I_Vx _D_Vx _P_Wz _I_Wz _D_Wz ))

(defmethod deeporange14_msgs::PIDComponentsMsg
  (:init
   (&key
    ((:P_Vx __P_Vx) 0.0)
    ((:I_Vx __I_Vx) 0.0)
    ((:D_Vx __D_Vx) 0.0)
    ((:P_Wz __P_Wz) 0.0)
    ((:I_Wz __I_Wz) 0.0)
    ((:D_Wz __D_Wz) 0.0)
    )
   (send-super :init)
   (setq _P_Vx (float __P_Vx))
   (setq _I_Vx (float __I_Vx))
   (setq _D_Vx (float __D_Vx))
   (setq _P_Wz (float __P_Wz))
   (setq _I_Wz (float __I_Wz))
   (setq _D_Wz (float __D_Wz))
   self)
  (:P_Vx
   (&optional __P_Vx)
   (if __P_Vx (setq _P_Vx __P_Vx)) _P_Vx)
  (:I_Vx
   (&optional __I_Vx)
   (if __I_Vx (setq _I_Vx __I_Vx)) _I_Vx)
  (:D_Vx
   (&optional __D_Vx)
   (if __D_Vx (setq _D_Vx __D_Vx)) _D_Vx)
  (:P_Wz
   (&optional __P_Wz)
   (if __P_Wz (setq _P_Wz __P_Wz)) _P_Wz)
  (:I_Wz
   (&optional __I_Wz)
   (if __I_Wz (setq _I_Wz __I_Wz)) _I_Wz)
  (:D_Wz
   (&optional __D_Wz)
   (if __D_Wz (setq _D_Wz __D_Wz)) _D_Wz)
  (:serialization-length
   ()
   (+
    ;; float64 _P_Vx
    8
    ;; float64 _I_Vx
    8
    ;; float64 _D_Vx
    8
    ;; float64 _P_Wz
    8
    ;; float64 _I_Wz
    8
    ;; float64 _D_Wz
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _P_Vx
       (sys::poke _P_Vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _I_Vx
       (sys::poke _I_Vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _D_Vx
       (sys::poke _D_Vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _P_Wz
       (sys::poke _P_Wz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _I_Wz
       (sys::poke _I_Wz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _D_Wz
       (sys::poke _D_Wz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _P_Vx
     (setq _P_Vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _I_Vx
     (setq _I_Vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _D_Vx
     (setq _D_Vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _P_Wz
     (setq _P_Wz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _I_Wz
     (setq _I_Wz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _D_Wz
     (setq _D_Wz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get deeporange14_msgs::PIDComponentsMsg :md5sum-) "cc85b3278733449a3886e64952b6b2bc")
(setf (get deeporange14_msgs::PIDComponentsMsg :datatype-) "deeporange14_msgs/PIDComponentsMsg")
(setf (get deeporange14_msgs::PIDComponentsMsg :definition-)
      "float64 P_Vx
float64 I_Vx
float64 D_Vx
float64 P_Wz
float64 I_Wz
float64 D_Wz
")



(provide :deeporange14_msgs/PIDComponentsMsg "cc85b3278733449a3886e64952b6b2bc")


