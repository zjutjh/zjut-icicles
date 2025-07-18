// Auto-generated. Do not edit!

// (in-package yahboomcar_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Position {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angleX = null;
      this.angleY = null;
      this.distance = null;
    }
    else {
      if (initObj.hasOwnProperty('angleX')) {
        this.angleX = initObj.angleX
      }
      else {
        this.angleX = 0.0;
      }
      if (initObj.hasOwnProperty('angleY')) {
        this.angleY = initObj.angleY
      }
      else {
        this.angleY = 0.0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Position
    // Serialize message field [angleX]
    bufferOffset = _serializer.float32(obj.angleX, buffer, bufferOffset);
    // Serialize message field [angleY]
    bufferOffset = _serializer.float32(obj.angleY, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Position
    let len;
    let data = new Position(null);
    // Deserialize message field [angleX]
    data.angleX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angleY]
    data.angleY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yahboomcar_msgs/Position';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e4df5e09e92d9d2b4758c9aab7a9ebeb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 angleX
    float32 angleY
    float32 distance
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Position(null);
    if (msg.angleX !== undefined) {
      resolved.angleX = msg.angleX;
    }
    else {
      resolved.angleX = 0.0
    }

    if (msg.angleY !== undefined) {
      resolved.angleY = msg.angleY;
    }
    else {
      resolved.angleY = 0.0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    return resolved;
    }
};

module.exports = Position;
