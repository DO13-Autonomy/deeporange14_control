# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from raptor_dbw_msgs/SteeringCmd.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import raptor_dbw_msgs.msg

class SteeringCmd(genpy.Message):
  _md5sum = "604a9bcc92c8bb45c3c9ce85f6bc45e1"
  _type = "raptor_dbw_msgs/SteeringCmd"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# Steering Wheel
float32 angle_cmd        # degrees, range -500 to 500
float32 angle_velocity   # degrees/s, range 0 to 498, 0 = maximum

# Enable
bool enable

# Ignore driver overrides
bool ignore

# Watchdog counter (optional)
uint8 rolling_counter

float32 torque_cmd # %-torque

float32 vehicle_curvature_cmd # 1/m

ActuatorControlMode control_type
================================================================================
MSG: raptor_dbw_msgs/ActuatorControlMode
uint8 value

uint8 open_loop = 0
uint8 closed_loop_actuator = 1
uint8 closed_loop_vehicle = 2
uint8 none = 255"""
  __slots__ = ['angle_cmd','angle_velocity','enable','ignore','rolling_counter','torque_cmd','vehicle_curvature_cmd','control_type']
  _slot_types = ['float32','float32','bool','bool','uint8','float32','float32','raptor_dbw_msgs/ActuatorControlMode']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       angle_cmd,angle_velocity,enable,ignore,rolling_counter,torque_cmd,vehicle_curvature_cmd,control_type

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SteeringCmd, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.angle_cmd is None:
        self.angle_cmd = 0.
      if self.angle_velocity is None:
        self.angle_velocity = 0.
      if self.enable is None:
        self.enable = False
      if self.ignore is None:
        self.ignore = False
      if self.rolling_counter is None:
        self.rolling_counter = 0
      if self.torque_cmd is None:
        self.torque_cmd = 0.
      if self.vehicle_curvature_cmd is None:
        self.vehicle_curvature_cmd = 0.
      if self.control_type is None:
        self.control_type = raptor_dbw_msgs.msg.ActuatorControlMode()
    else:
      self.angle_cmd = 0.
      self.angle_velocity = 0.
      self.enable = False
      self.ignore = False
      self.rolling_counter = 0
      self.torque_cmd = 0.
      self.vehicle_curvature_cmd = 0.
      self.control_type = raptor_dbw_msgs.msg.ActuatorControlMode()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2f3B2fB().pack(_x.angle_cmd, _x.angle_velocity, _x.enable, _x.ignore, _x.rolling_counter, _x.torque_cmd, _x.vehicle_curvature_cmd, _x.control_type.value))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.control_type is None:
        self.control_type = raptor_dbw_msgs.msg.ActuatorControlMode()
      end = 0
      _x = self
      start = end
      end += 20
      (_x.angle_cmd, _x.angle_velocity, _x.enable, _x.ignore, _x.rolling_counter, _x.torque_cmd, _x.vehicle_curvature_cmd, _x.control_type.value,) = _get_struct_2f3B2fB().unpack(str[start:end])
      self.enable = bool(self.enable)
      self.ignore = bool(self.ignore)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2f3B2fB().pack(_x.angle_cmd, _x.angle_velocity, _x.enable, _x.ignore, _x.rolling_counter, _x.torque_cmd, _x.vehicle_curvature_cmd, _x.control_type.value))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.control_type is None:
        self.control_type = raptor_dbw_msgs.msg.ActuatorControlMode()
      end = 0
      _x = self
      start = end
      end += 20
      (_x.angle_cmd, _x.angle_velocity, _x.enable, _x.ignore, _x.rolling_counter, _x.torque_cmd, _x.vehicle_curvature_cmd, _x.control_type.value,) = _get_struct_2f3B2fB().unpack(str[start:end])
      self.enable = bool(self.enable)
      self.ignore = bool(self.ignore)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2f3B2fB = None
def _get_struct_2f3B2fB():
    global _struct_2f3B2fB
    if _struct_2f3B2fB is None:
        _struct_2f3B2fB = struct.Struct("<2f3B2fB")
    return _struct_2f3B2fB
