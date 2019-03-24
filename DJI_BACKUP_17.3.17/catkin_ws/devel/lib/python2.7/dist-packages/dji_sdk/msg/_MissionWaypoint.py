# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dji_sdk/MissionWaypoint.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import dji_sdk.msg

class MissionWaypoint(genpy.Message):
  _md5sum = "d321a17884980812391aa8e2850409e4"
  _type = "dji_sdk/MissionWaypoint"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 latitude
float64 longitude
float32 altitude
float32 damping_distance
int16 target_yaw
int16 target_gimbal_pitch
uint8 turn_mode
uint8 has_action
uint16 action_time_limit
MissionWaypointAction waypoint_action

================================================================================
MSG: dji_sdk/MissionWaypointAction
uint8 action_repeat
uint8[15] command_list
int16[15] command_parameter

"""
  __slots__ = ['latitude','longitude','altitude','damping_distance','target_yaw','target_gimbal_pitch','turn_mode','has_action','action_time_limit','waypoint_action']
  _slot_types = ['float64','float64','float32','float32','int16','int16','uint8','uint8','uint16','dji_sdk/MissionWaypointAction']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       latitude,longitude,altitude,damping_distance,target_yaw,target_gimbal_pitch,turn_mode,has_action,action_time_limit,waypoint_action

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MissionWaypoint, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.latitude is None:
        self.latitude = 0.
      if self.longitude is None:
        self.longitude = 0.
      if self.altitude is None:
        self.altitude = 0.
      if self.damping_distance is None:
        self.damping_distance = 0.
      if self.target_yaw is None:
        self.target_yaw = 0
      if self.target_gimbal_pitch is None:
        self.target_gimbal_pitch = 0
      if self.turn_mode is None:
        self.turn_mode = 0
      if self.has_action is None:
        self.has_action = 0
      if self.action_time_limit is None:
        self.action_time_limit = 0
      if self.waypoint_action is None:
        self.waypoint_action = dji_sdk.msg.MissionWaypointAction()
    else:
      self.latitude = 0.
      self.longitude = 0.
      self.altitude = 0.
      self.damping_distance = 0.
      self.target_yaw = 0
      self.target_gimbal_pitch = 0
      self.turn_mode = 0
      self.has_action = 0
      self.action_time_limit = 0
      self.waypoint_action = dji_sdk.msg.MissionWaypointAction()

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
      buff.write(_struct_2d2f2h2BHB.pack(_x.latitude, _x.longitude, _x.altitude, _x.damping_distance, _x.target_yaw, _x.target_gimbal_pitch, _x.turn_mode, _x.has_action, _x.action_time_limit, _x.waypoint_action.action_repeat))
      _x = self.waypoint_action.command_list
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_15B.pack(*_x))
      else:
        buff.write(_struct_15s.pack(_x))
      buff.write(_struct_15h.pack(*self.waypoint_action.command_parameter))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.waypoint_action is None:
        self.waypoint_action = dji_sdk.msg.MissionWaypointAction()
      end = 0
      _x = self
      start = end
      end += 33
      (_x.latitude, _x.longitude, _x.altitude, _x.damping_distance, _x.target_yaw, _x.target_gimbal_pitch, _x.turn_mode, _x.has_action, _x.action_time_limit, _x.waypoint_action.action_repeat,) = _struct_2d2f2h2BHB.unpack(str[start:end])
      start = end
      end += 15
      self.waypoint_action.command_list = str[start:end]
      start = end
      end += 30
      self.waypoint_action.command_parameter = _struct_15h.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_2d2f2h2BHB.pack(_x.latitude, _x.longitude, _x.altitude, _x.damping_distance, _x.target_yaw, _x.target_gimbal_pitch, _x.turn_mode, _x.has_action, _x.action_time_limit, _x.waypoint_action.action_repeat))
      _x = self.waypoint_action.command_list
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_15B.pack(*_x))
      else:
        buff.write(_struct_15s.pack(_x))
      buff.write(self.waypoint_action.command_parameter.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.waypoint_action is None:
        self.waypoint_action = dji_sdk.msg.MissionWaypointAction()
      end = 0
      _x = self
      start = end
      end += 33
      (_x.latitude, _x.longitude, _x.altitude, _x.damping_distance, _x.target_yaw, _x.target_gimbal_pitch, _x.turn_mode, _x.has_action, _x.action_time_limit, _x.waypoint_action.action_repeat,) = _struct_2d2f2h2BHB.unpack(str[start:end])
      start = end
      end += 15
      self.waypoint_action.command_list = str[start:end]
      start = end
      end += 30
      self.waypoint_action.command_parameter = numpy.frombuffer(str[start:end], dtype=numpy.int16, count=15)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2d2f2h2BHB = struct.Struct("<2d2f2h2BHB")
_struct_15h = struct.Struct("<15h")
_struct_15s = struct.Struct("<15s")
_struct_15B = struct.Struct("<15B")
