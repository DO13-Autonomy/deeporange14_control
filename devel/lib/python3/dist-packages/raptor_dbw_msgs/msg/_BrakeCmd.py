# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from raptor_dbw_msgs/BrakeCmd.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import raptor_dbw_msgs.msg

class BrakeCmd(genpy.Message):
  _md5sum = "4d6dead8aa3923674dc63b7884f45810"
  _type = "raptor_dbw_msgs/BrakeCmd"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# Brake pedal (%)
float32 pedal_cmd

# Enable
bool enable

# Watchdog counter (optional)
uint8 rolling_counter

float32 torque_cmd # %-torque 
float32 decel_limit # m/s^2

ActuatorControlMode control_type

float32 decel_negative_jerk_limit # m/s^3
================================================================================
MSG: raptor_dbw_msgs/ActuatorControlMode
uint8 value

uint8 open_loop = 0
uint8 closed_loop_actuator = 1
uint8 closed_loop_vehicle = 2
uint8 none = 255"""
  __slots__ = ['pedal_cmd','enable','rolling_counter','torque_cmd','decel_limit','control_type','decel_negative_jerk_limit']
  _slot_types = ['float32','bool','uint8','float32','float32','raptor_dbw_msgs/ActuatorControlMode','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pedal_cmd,enable,rolling_counter,torque_cmd,decel_limit,control_type,decel_negative_jerk_limit

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(BrakeCmd, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.pedal_cmd is None:
        self.pedal_cmd = 0.
      if self.enable is None:
        self.enable = False
      if self.rolling_counter is None:
        self.rolling_counter = 0
      if self.torque_cmd is None:
        self.torque_cmd = 0.
      if self.decel_limit is None:
        self.decel_limit = 0.
      if self.control_type is None:
        self.control_type = raptor_dbw_msgs.msg.ActuatorControlMode()
      if self.decel_negative_jerk_limit is None:
        self.decel_negative_jerk_limit = 0.
    else:
      self.pedal_cmd = 0.
      self.enable = False
      self.rolling_counter = 0
      self.torque_cmd = 0.
      self.decel_limit = 0.
      self.control_type = raptor_dbw_msgs.msg.ActuatorControlMode()
      self.decel_negative_jerk_limit = 0.

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
      buff.write(_get_struct_f2B2fBf().pack(_x.pedal_cmd, _x.enable, _x.rolling_counter, _x.torque_cmd, _x.decel_limit, _x.control_type.value, _x.decel_negative_jerk_limit))
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
      end += 19
      (_x.pedal_cmd, _x.enable, _x.rolling_counter, _x.torque_cmd, _x.decel_limit, _x.control_type.value, _x.decel_negative_jerk_limit,) = _get_struct_f2B2fBf().unpack(str[start:end])
      self.enable = bool(self.enable)
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
      buff.write(_get_struct_f2B2fBf().pack(_x.pedal_cmd, _x.enable, _x.rolling_counter, _x.torque_cmd, _x.decel_limit, _x.control_type.value, _x.decel_negative_jerk_limit))
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
      end += 19
      (_x.pedal_cmd, _x.enable, _x.rolling_counter, _x.torque_cmd, _x.decel_limit, _x.control_type.value, _x.decel_negative_jerk_limit,) = _get_struct_f2B2fBf().unpack(str[start:end])
      self.enable = bool(self.enable)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_f2B2fBf = None
def _get_struct_f2B2fBf():
    global _struct_f2B2fBf
    if _struct_f2B2fBf is None:
        _struct_f2B2fBf = struct.Struct("<f2B2fBf")
    return _struct_f2B2fBf