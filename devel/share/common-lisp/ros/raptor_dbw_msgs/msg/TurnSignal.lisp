; Auto-generated. Do not edit!


(cl:in-package raptor_dbw_msgs-msg)


;//! \htmlinclude TurnSignal.msg.html

(cl:defclass <TurnSignal> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TurnSignal (<TurnSignal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TurnSignal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TurnSignal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor_dbw_msgs-msg:<TurnSignal> is deprecated: use raptor_dbw_msgs-msg:TurnSignal instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <TurnSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor_dbw_msgs-msg:value-val is deprecated.  Use raptor_dbw_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TurnSignal>)))
    "Constants for message type '<TurnSignal>"
  '((:NONE . 0)
    (:LEFT . 1)
    (:RIGHT . 2)
    (:HAZARDS . 3)
    (:SNA . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TurnSignal)))
    "Constants for message type 'TurnSignal"
  '((:NONE . 0)
    (:LEFT . 1)
    (:RIGHT . 2)
    (:HAZARDS . 3)
    (:SNA . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TurnSignal>) ostream)
  "Serializes a message object of type '<TurnSignal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TurnSignal>) istream)
  "Deserializes a message object of type '<TurnSignal>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TurnSignal>)))
  "Returns string type for a message object of type '<TurnSignal>"
  "raptor_dbw_msgs/TurnSignal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TurnSignal)))
  "Returns string type for a message object of type 'TurnSignal"
  "raptor_dbw_msgs/TurnSignal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TurnSignal>)))
  "Returns md5sum for a message object of type '<TurnSignal>"
  "933b11bd0b0ae8b539bf6f942bfb3693")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TurnSignal)))
  "Returns md5sum for a message object of type 'TurnSignal"
  "933b11bd0b0ae8b539bf6f942bfb3693")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TurnSignal>)))
  "Returns full string definition for message of type '<TurnSignal>"
  (cl:format cl:nil "uint8 value~%~%uint8 NONE = 0~%uint8 LEFT = 1~%uint8 RIGHT = 2~%uint8 HAZARDS = 3~%uint8 SNA = 7~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TurnSignal)))
  "Returns full string definition for message of type 'TurnSignal"
  (cl:format cl:nil "uint8 value~%~%uint8 NONE = 0~%uint8 LEFT = 1~%uint8 RIGHT = 2~%uint8 HAZARDS = 3~%uint8 SNA = 7~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TurnSignal>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TurnSignal>))
  "Converts a ROS message object to a list"
  (cl:list 'TurnSignal
    (cl:cons ':value (value msg))
))
