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

class PatrolWarning {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed = null;
      this.Function = null;
      this.ResponseDist = null;
      this.LaserAngle = null;
    }
    else {
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('Function')) {
        this.Function = initObj.Function
      }
      else {
        this.Function = '';
      }
      if (initObj.hasOwnProperty('ResponseDist')) {
        this.ResponseDist = initObj.ResponseDist
      }
      else {
        this.ResponseDist = 0.0;
      }
      if (initObj.hasOwnProperty('LaserAngle')) {
        this.LaserAngle = initObj.LaserAngle
      }
      else {
        this.LaserAngle = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PatrolWarning
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [Function]
    bufferOffset = _serializer.string(obj.Function, buffer, bufferOffset);
    // Serialize message field [ResponseDist]
    bufferOffset = _serializer.float32(obj.ResponseDist, buffer, bufferOffset);
    // Serialize message field [LaserAngle]
    bufferOffset = _serializer.int32(obj.LaserAngle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PatrolWarning
    let len;
    let data = new PatrolWarning(null);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Function]
    data.Function = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ResponseDist]
    data.ResponseDist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [LaserAngle]
    data.LaserAngle = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.Function.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'transbot_msgs/PatrolWarning';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3c2aedf4e9d9a1d5ce206d948829c9bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 speed
    string Function
    float32 ResponseDist
    int32   LaserAngle
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PatrolWarning(null);
    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.Function !== undefined) {
      resolved.Function = msg.Function;
    }
    else {
      resolved.Function = ''
    }

    if (msg.ResponseDist !== undefined) {
      resolved.ResponseDist = msg.ResponseDist;
    }
    else {
      resolved.ResponseDist = 0.0
    }

    if (msg.LaserAngle !== undefined) {
      resolved.LaserAngle = msg.LaserAngle;
    }
    else {
      resolved.LaserAngle = 0
    }

    return resolved;
    }
};

module.exports = PatrolWarning;
