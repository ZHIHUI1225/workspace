// Auto-generated. Do not edit!

// (in-package robot_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let target_pose = require('./target_pose.js');

//-----------------------------------------------------------

class target_pose_array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_pose_array = null;
    }
    else {
      if (initObj.hasOwnProperty('target_pose_array')) {
        this.target_pose_array = initObj.target_pose_array
      }
      else {
        this.target_pose_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type target_pose_array
    // Serialize message field [target_pose_array]
    // Serialize the length for message field [target_pose_array]
    bufferOffset = _serializer.uint32(obj.target_pose_array.length, buffer, bufferOffset);
    obj.target_pose_array.forEach((val) => {
      bufferOffset = target_pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type target_pose_array
    let len;
    let data = new target_pose_array(null);
    // Deserialize message field [target_pose_array]
    // Deserialize array length for message field [target_pose_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.target_pose_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.target_pose_array[i] = target_pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 25 * object.target_pose_array.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_msg/target_pose_array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7368196c844e512e8b512a71481181c8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    robot_msg/target_pose[] target_pose_array
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new target_pose_array(null);
    if (msg.target_pose_array !== undefined) {
      resolved.target_pose_array = new Array(msg.target_pose_array.length);
      for (let i = 0; i < resolved.target_pose_array.length; ++i) {
        resolved.target_pose_array[i] = target_pose.Resolve(msg.target_pose_array[i]);
      }
    }
    else {
      resolved.target_pose_array = []
    }

    return resolved;
    }
};

module.exports = target_pose_array;
