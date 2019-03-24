# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dji_sdk/MissionHpUploadRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import dji_sdk.msg

class MissionHpUploadRequest(genpy.Message):
  _md5sum = "e36e66ca170c4d03ee023ad56c6bb5a0"
  _type = "dji_sdk/MissionHpUploadRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """MissionHotpointTask hotpoint_task

================================================================================
MSG: dji_sdk/MissionHotpointTask
float64 latitude
float64 longitude
float64 altitude
float64 radius
float32 angular_speed
uint8 is_clockwise
uint8 start_point
uint8 yaw_mode

"""
  __slots__ = ['hotpoint_task']
  _slot_types = ['dji_sdk/MissionHotpointTask']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       hotpoint_task

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MissionHpUploadRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.hotpoint_task is None:
        self.hotpoint_task = dji_sdk.msg.MissionHotpointTask()
    else:
      self.hotpoint_task = dji_sdk.msg.MissionHotpointTask()

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
      buff.write(_struct_4df3B.pack(_x.hotpoint_task.latitude, _x.hotpoint_task.longitude, _x.hotpoint_task.altitude, _x.hotpoint_task.radius, _x.hotpoint_task.angular_speed, _x.hotpoint_task.is_clockwise, _x.hotpoint_task.start_point, _x.hotpoint_task.yaw_mode))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.hotpoint_task is None:
        self.hotpoint_task = dji_sdk.msg.MissionHotpointTask()
      end = 0
      _x = self
      start = end
      end += 39
      (_x.hotpoint_task.latitude, _x.hotpoint_task.longitude, _x.hotpoint_task.altitude, _x.hotpoint_task.radius, _x.hotpoint_task.angular_speed, _x.hotpoint_task.is_clockwise, _x.hotpoint_task.start_point, _x.hotpoint_task.yaw_mode,) = _struct_4df3B.unpack(str[start:end])
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
      buff.write(_struct_4df3B.pack(_x.hotpoint_task.latitude, _x.hotpoint_task.longitude, _x.hotpoint_task.altitude, _x.hotpoint_task.radius, _x.hotpoint_task.angular_speed, _x.hotpoint_task.is_clockwise, _x.hotpoint_task.start_point, _x.hotpoint_task.yaw_mode))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.hotpoint_task is None:
        self.hotpoint_task = dji_sdk.msg.MissionHotpointTask()
      end = 0
      _x = self
      start = end
      end += 39
      (_x.hotpoint_task.latitude, _x.hotpoint_task.longitude, _x.hotpoint_task.altitude, _x.hotpoint_task.radius, _x.hotpoint_task.angular_speed, _x.hotpoint_task.is_clockwise, _x.hotpoint_task.start_point, _x.hotpoint_task.yaw_mode,) = _struct_4df3B.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4df3B = struct.Struct("<4df3B")
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dji_sdk/MissionHpUploadResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class MissionHpUploadResponse(genpy.Message):
  _md5sum = "eb13ac1f1354ccecb7941ee8fa2192e8"
  _type = "dji_sdk/MissionHpUploadResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool result


"""
  __slots__ = ['result']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       result

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MissionHpUploadResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = False
    else:
      self.result = False

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
      buff.write(_struct_B.pack(self.result))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _struct_B.unpack(str[start:end])
      self.result = bool(self.result)
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
      buff.write(_struct_B.pack(self.result))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _struct_B.unpack(str[start:end])
      self.result = bool(self.result)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
class MissionHpUpload(object):
  _type          = 'dji_sdk/MissionHpUpload'
  _md5sum = 'd057d61e861a72611b7918cfd8a98e4a'
  _request_class  = MissionHpUploadRequest
  _response_class = MissionHpUploadResponse
