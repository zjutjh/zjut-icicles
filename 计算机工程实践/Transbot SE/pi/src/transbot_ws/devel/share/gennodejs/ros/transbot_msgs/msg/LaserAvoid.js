// Auto-generated. Do not edit!

// (in-package transbot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class LaserAvoid {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Angle_range = null;
      this.ResponseDist = null;
      this.speed = null;
    }
    else {
      if (initObj.hasOwnProperty('Angle_range')) {
        this.Angle_range = initObj.Angle_range
      }
      else {
        this.Angle_range = 0;
      }
      if (initObj.hasOwnProperty('ResponseDist')) {
        this.ResponseDist = initObj.ResponseDist
      }
      else {
        this.ResponseDist = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LaserAvoid
    // Serialize message field [Angle_range]
    bufferOffset = _serializer.int32(obj.Angle_range, buffer, bufferOffset);
    // Serialize message field [ResponseDist]
    bufferOffset = _serializer.float32(obj.ResponseDist, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LaserAvoid
    let len;
    let data = new LaserAvoid(null);
    // Deserialize message field [Angle_range]
    data.Angle_range = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ResponseDist]
    data.ResponseDist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'transbot_msgs/LaserAvoid';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6872d2ec650238739f9b3c8aab8ed9b1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 Angle_range
    float32 ResponseDist
    float32 speed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LaserAvoid(null);
    if (msg.Angle_range !== undefined) {
      resolved.Angle_range = msg.Angle_range;
    }
    else {
      resolved.Angle_range = 0
    }

    if (msg.ResponseDist !== undefined) {
      resolved.ResponseDist = msg.ResponseDist;
    }
    else {
      resolved.ResponseDist = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    return resolved;
    }
};

module.exports = LaserAvoid;
