; Auto-generated. Do not edit!


(cl:in-package deeporange14_msgs-msg)


;//! \htmlinclude PIDComponentsMsg.msg.html

(cl:defclass <PIDComponentsMsg> (roslisp-msg-protocol:ros-message)
  ((P_Vx
    :reader P_Vx
    :initarg :P_Vx
    :type cl:float
    :initform 0.0)
   (I_Vx
    :reader I_Vx
    :initarg :I_Vx
    :type cl:float
    :initform 0.0)
   (D_Vx
    :reader D_Vx
    :initarg :D_Vx
    :type cl:float
    :initform 0.0)
   (P_Wz
    :reader P_Wz
    :initarg :P_Wz
    :type cl:float
    :initform 0.0)
   (I_Wz
    :reader I_Wz
    :initarg :I_Wz
    :type cl:float
    :initform 0.0)
   (D_Wz
    :reader D_Wz
    :initarg :D_Wz
    :type cl:float
    :initform 0.0))
)

(cl:defclass PIDComponentsMsg (<PIDComponentsMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PIDComponentsMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PIDComponentsMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deeporange14_msgs-msg:<PIDComponentsMsg> is deprecated: use deeporange14_msgs-msg:PIDComponentsMsg instead.")))

(cl:ensure-generic-function 'P_Vx-val :lambda-list '(m))
(cl:defmethod P_Vx-val ((m <PIDComponentsMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deeporange14_msgs-msg:P_Vx-val is deprecated.  Use deeporange14_msgs-msg:P_Vx instead.")
  (P_Vx m))

(cl:ensure-generic-function 'I_Vx-val :lambda-list '(m))
(cl:defmethod I_Vx-val ((m <PIDComponentsMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deeporange14_msgs-msg:I_Vx-val is deprecated.  Use deeporange14_msgs-msg:I_Vx instead.")
  (I_Vx m))

(cl:ensure-generic-function 'D_Vx-val :lambda-list '(m))
(cl:defmethod D_Vx-val ((m <PIDComponentsMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deeporange14_msgs-msg:D_Vx-val is deprecated.  Use deeporange14_msgs-msg:D_Vx instead.")
  (D_Vx m))

(cl:ensure-generic-function 'P_Wz-val :lambda-list '(m))
(cl:defmethod P_Wz-val ((m <PIDComponentsMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deeporange14_msgs-msg:P_Wz-val is deprecated.  Use deeporange14_msgs-msg:P_Wz instead.")
  (P_Wz m))

(cl:ensure-generic-function 'I_Wz-val :lambda-list '(m))
(cl:defmethod I_Wz-val ((m <PIDComponentsMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deeporange14_msgs-msg:I_Wz-val is deprecated.  Use deeporange14_msgs-msg:I_Wz instead.")
  (I_Wz m))

(cl:ensure-generic-function 'D_Wz-val :lambda-list '(m))
(cl:defmethod D_Wz-val ((m <PIDComponentsMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deeporange14_msgs-msg:D_Wz-val is deprecated.  Use deeporange14_msgs-msg:D_Wz instead.")
  (D_Wz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PIDComponentsMsg>) ostream)
  "Serializes a message object of type '<PIDComponentsMsg>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'P_Vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'I_Vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'D_Vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'P_Wz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'I_Wz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'D_Wz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PIDComponentsMsg>) istream)
  "Deserializes a message object of type '<PIDComponentsMsg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P_Vx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'I_Vx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'D_Vx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P_Wz) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'I_Wz) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'D_Wz) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PIDComponentsMsg>)))
  "Returns string type for a message object of type '<PIDComponentsMsg>"
  "deeporange14_msgs/PIDComponentsMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PIDComponentsMsg)))
  "Returns string type for a message object of type 'PIDComponentsMsg"
  "deeporange14_msgs/PIDComponentsMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PIDComponentsMsg>)))
  "Returns md5sum for a message object of type '<PIDComponentsMsg>"
  "cc85b3278733449a3886e64952b6b2bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PIDComponentsMsg)))
  "Returns md5sum for a message object of type 'PIDComponentsMsg"
  "cc85b3278733449a3886e64952b6b2bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PIDComponentsMsg>)))
  "Returns full string definition for message of type '<PIDComponentsMsg>"
  (cl:format cl:nil "float64 P_Vx~%float64 I_Vx~%float64 D_Vx~%float64 P_Wz~%float64 I_Wz~%float64 D_Wz~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PIDComponentsMsg)))
  "Returns full string definition for message of type 'PIDComponentsMsg"
  (cl:format cl:nil "float64 P_Vx~%float64 I_Vx~%float64 D_Vx~%float64 P_Wz~%float64 I_Wz~%float64 D_Wz~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PIDComponentsMsg>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PIDComponentsMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PIDComponentsMsg
    (cl:cons ':P_Vx (P_Vx msg))
    (cl:cons ':I_Vx (I_Vx msg))
    (cl:cons ':D_Vx (D_Vx msg))
    (cl:cons ':P_Wz (P_Wz msg))
    (cl:cons ':I_Wz (I_Wz msg))
    (cl:cons ':D_Wz (D_Wz msg))
))
