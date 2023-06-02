# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from robot_msg/target_pose_array.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import robot_msg.msg

class target_pose_array(genpy.Message):
  _md5sum = "7368196c844e512e8b512a71481181c8"
  _type = "robot_msg/target_pose_array"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """robot_msg/target_pose[] target_pose_array

================================================================================
MSG: robot_msg/target_pose
int8 ID
geometry_msgs/Point position

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['target_pose_array']
  _slot_types = ['robot_msg/target_pose[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       target_pose_array

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(target_pose_array, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.target_pose_array is None:
        self.target_pose_array = []
    else:
      self.target_pose_array = []

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
      length = len(self.target_pose_array)
      buff.write(_struct_I.pack(length))
      for val1 in self.target_pose_array:
        _x = val1.ID
        buff.write(_get_struct_b().pack(_x))
        _v1 = val1.position
        _x = _v1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
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
      if self.target_pose_array is None:
        self.target_pose_array = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.target_pose_array = []
      for i in range(0, length):
        val1 = robot_msg.msg.target_pose()
        start = end
        end += 1
        (val1.ID,) = _get_struct_b().unpack(str[start:end])
        _v2 = val1.position
        _x = _v2
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.target_pose_array.append(val1)
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
      length = len(self.target_pose_array)
      buff.write(_struct_I.pack(length))
      for val1 in self.target_pose_array:
        _x = val1.ID
        buff.write(_get_struct_b().pack(_x))
        _v3 = val1.position
        _x = _v3
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
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
      if self.target_pose_array is None:
        self.target_pose_array = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.target_pose_array = []
      for i in range(0, length):
        val1 = robot_msg.msg.target_pose()
        start = end
        end += 1
        (val1.ID,) = _get_struct_b().unpack(str[start:end])
        _v4 = val1.position
        _x = _v4
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.target_pose_array.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_b = None
def _get_struct_b():
    global _struct_b
    if _struct_b is None:
        _struct_b = struct.Struct("<b")
    return _struct_b
