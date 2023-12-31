;; Auto-generated. Do not edit!


(when (boundp 'raptor_dbw_msgs::AcceleratorPedalCmd)
  (if (not (find-package "RAPTOR_DBW_MSGS"))
    (make-package "RAPTOR_DBW_MSGS"))
  (shadow 'AcceleratorPedalCmd (find-package "RAPTOR_DBW_MSGS")))
(unless (find-package "RAPTOR_DBW_MSGS::ACCELERATORPEDALCMD")
  (make-package "RAPTOR_DBW_MSGS::ACCELERATORPEDALCMD"))

(in-package "ROS")
;;//! \htmlinclude AcceleratorPedalCmd.msg.html


(defclass raptor_dbw_msgs::AcceleratorPedalCmd
  :super ros::object
  :slots (_pedal_cmd _torque_cmd _speed_cmd _enable _ignore _rolling_counter _road_slope _control_type _accel_limit _accel_positive_jerk_limit ))

(defmethod raptor_dbw_msgs::AcceleratorPedalCmd
  (:init
   (&key
    ((:pedal_cmd __pedal_cmd) 0.0)
    ((:torque_cmd __torque_cmd) 0.0)
    ((:speed_cmd __speed_cmd) 0.0)
    ((:enable __enable) nil)
    ((:ignore __ignore) nil)
    ((:rolling_counter __rolling_counter) 0)
    ((:road_slope __road_slope) 0.0)
    ((:control_type __control_type) (instance raptor_dbw_msgs::ActuatorControlMode :init))
    ((:accel_limit __accel_limit) 0.0)
    ((:accel_positive_jerk_limit __accel_positive_jerk_limit) 0.0)
    )
   (send-super :init)
   (setq _pedal_cmd (float __pedal_cmd))
   (setq _torque_cmd (float __torque_cmd))
   (setq _speed_cmd (float __speed_cmd))
   (setq _enable __enable)
   (setq _ignore __ignore)
   (setq _rolling_counter (round __rolling_counter))
   (setq _road_slope (float __road_slope))
   (setq _control_type __control_type)
   (setq _accel_limit (float __accel_limit))
   (setq _accel_positive_jerk_limit (float __accel_positive_jerk_limit))
   self)
  (:pedal_cmd
   (&optional __pedal_cmd)
   (if __pedal_cmd (setq _pedal_cmd __pedal_cmd)) _pedal_cmd)
  (:torque_cmd
   (&optional __torque_cmd)
   (if __torque_cmd (setq _torque_cmd __torque_cmd)) _torque_cmd)
  (:speed_cmd
   (&optional __speed_cmd)
   (if __speed_cmd (setq _speed_cmd __speed_cmd)) _speed_cmd)
  (:enable
   (&optional (__enable :null))
   (if (not (eq __enable :null)) (setq _enable __enable)) _enable)
  (:ignore
   (&optional (__ignore :null))
   (if (not (eq __ignore :null)) (setq _ignore __ignore)) _ignore)
  (:rolling_counter
   (&optional __rolling_counter)
   (if __rolling_counter (setq _rolling_counter __rolling_counter)) _rolling_counter)
  (:road_slope
   (&optional __road_slope)
   (if __road_slope (setq _road_slope __road_slope)) _road_slope)
  (:control_type
   (&rest __control_type)
   (if (keywordp (car __control_type))
       (send* _control_type __control_type)
     (progn
       (if __control_type (setq _control_type (car __control_type)))
       _control_type)))
  (:accel_limit
   (&optional __accel_limit)
   (if __accel_limit (setq _accel_limit __accel_limit)) _accel_limit)
  (:accel_positive_jerk_limit
   (&optional __accel_positive_jerk_limit)
   (if __accel_positive_jerk_limit (setq _accel_positive_jerk_limit __accel_positive_jerk_limit)) _accel_positive_jerk_limit)
  (:serialization-length
   ()
   (+
    ;; float32 _pedal_cmd
    4
    ;; float32 _torque_cmd
    4
    ;; float32 _speed_cmd
    4
    ;; bool _enable
    1
    ;; bool _ignore
    1
    ;; uint8 _rolling_counter
    1
    ;; float32 _road_slope
    4
    ;; raptor_dbw_msgs/ActuatorControlMode _control_type
    (send _control_type :serialization-length)
    ;; float32 _accel_limit
    4
    ;; float32 _accel_positive_jerk_limit
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pedal_cmd
       (sys::poke _pedal_cmd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _torque_cmd
       (sys::poke _torque_cmd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed_cmd
       (sys::poke _speed_cmd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;; bool _ignore
       (if _ignore (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _rolling_counter
       (write-byte _rolling_counter s)
     ;; float32 _road_slope
       (sys::poke _road_slope (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; raptor_dbw_msgs/ActuatorControlMode _control_type
       (send _control_type :serialize s)
     ;; float32 _accel_limit
       (sys::poke _accel_limit (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _accel_positive_jerk_limit
       (sys::poke _accel_positive_jerk_limit (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pedal_cmd
     (setq _pedal_cmd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _torque_cmd
     (setq _torque_cmd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed_cmd
     (setq _speed_cmd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ignore
     (setq _ignore (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _rolling_counter
     (setq _rolling_counter (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _road_slope
     (setq _road_slope (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; raptor_dbw_msgs/ActuatorControlMode _control_type
     (send _control_type :deserialize buf ptr-) (incf ptr- (send _control_type :serialization-length))
   ;; float32 _accel_limit
     (setq _accel_limit (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _accel_positive_jerk_limit
     (setq _accel_positive_jerk_limit (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get raptor_dbw_msgs::AcceleratorPedalCmd :md5sum-) "b4fa43636bd38fecc84a7531ea6865c4")
(setf (get raptor_dbw_msgs::AcceleratorPedalCmd :datatype-) "raptor_dbw_msgs/AcceleratorPedalCmd")
(setf (get raptor_dbw_msgs::AcceleratorPedalCmd :definition-)
      "# Accelerator pedal (%)
float32 pedal_cmd

float32 torque_cmd # %-torque

float32 speed_cmd # m/s

# Enable
bool enable

# Ignore driver overrides
bool ignore

# Watchdog counter (optional)
uint8 rolling_counter

float32 road_slope # degrees - used with speed mode

ActuatorControlMode control_type

float32 accel_limit # m/s^2

float32 accel_positive_jerk_limit # m/s^3
================================================================================
MSG: raptor_dbw_msgs/ActuatorControlMode
uint8 value

uint8 open_loop = 0
uint8 closed_loop_actuator = 1
uint8 closed_loop_vehicle = 2
uint8 none = 255
")



(provide :raptor_dbw_msgs/AcceleratorPedalCmd "b4fa43636bd38fecc84a7531ea6865c4")


