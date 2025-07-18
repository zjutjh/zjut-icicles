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

class Battery {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Voltage = null;
    }
    else {
      if (initObj.hasOwnProperty('Voltage')) {
        this.Voltage = initObj.Voltage
      }
      else {
        this.Voltage = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Battery
    // Serialize message field [Voltage]
    bufferOffset = _serializer.float32(obj.Voltage, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Battery
    let len;
    let data = new Battery(null);
    // Deserialize message field [Voltage]
    data.Voltage = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'transbot_msgs/Battery';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cb42914b7d362060569576ceed0643ce';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 Voltage
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Battery(null);
    if (msg.Voltage !== undefined) {
      resolved.Voltage = msg.Voltage;
    }
    else {
      resolved.Voltage = 0.0
    }

    return resolved;
    }
};

module.exports = Battery;
