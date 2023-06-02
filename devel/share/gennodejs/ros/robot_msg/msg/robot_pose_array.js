// Auto-generated. Do not edit!

// (in-package robot_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let robot_pose = require('./robot_pose.js');

//-----------------------------------------------------------

class robot_pose_array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_pose_array = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_pose_array')) {
        this.robot_pose_array = initObj.robot_pose_array
      }
      else {
        this.robot_pose_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type robot_pose_array
    // Serialize message field [robot_pose_array]
    // Serialize the length for message field [robot_pose_array]
    bufferOffset = _serializer.uint32(obj.robot_pose_array.length, buffer, bufferOffset);
    obj.robot_pose_array.forEach((val) => {
      bufferOffset = robot_pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robot_pose_array
    let len;
    let data = new robot_pose_array(null);
    // Deserialize message field [robot_pose_array]
    // Deserialize array length for message field [robot_pose_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.robot_pose_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.robot_pose_array[i] = robot_pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 29 * object.robot_pose_array.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_msg/robot_pose_array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b5025b9975c3a323219809cc70bf1c63';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    robot_msg/robot_pose[] robot_pose_array
    
    ================================================================================
    MSG: robot_msg/robot_pose
    std_msgs/Int8 ID
    geometry_msgs/Point position
    float32 yaw
    
    ================================================================================
    MSG: std_msgs/Int8
    int8 data
    
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
    const resolved = new robot_pose_array(null);
    if (msg.robot_pose_array !== undefined) {
      resolved.robot_pose_array = new Array(msg.robot_pose_array.length);
      for (let i = 0; i < resolved.robot_pose_array.length; ++i) {
        resolved.robot_pose_array[i] = robot_pose.Resolve(msg.robot_pose_array[i]);
      }
    }
    else {
      resolved.robot_pose_array = []
    }

    return resolved;
    }
};

module.exports = robot_pose_array;
