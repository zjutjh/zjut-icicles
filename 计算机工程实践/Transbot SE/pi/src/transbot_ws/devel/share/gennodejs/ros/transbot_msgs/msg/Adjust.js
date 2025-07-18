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

class Adjust {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.adjust = null;
    }
    else {
      if (initObj.hasOwnProperty('adjust')) {
        this.adjust = initObj.adjust
      }
      else {
        this.adjust = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Adjust
    // Serialize message field [adjust]
    bufferOffset = _serializer.bool(obj.adjust, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Adjust
    let len;
    let data = new Adjust(null);
    // Deserialize message field [adjust]
    data.adjust = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'transbot_msgs/Adjust';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '686be2de32be2d650746cf5e906439fb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool adjust
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Adjust(null);
    if (msg.adjust !== undefined) {
      resolved.adjust = msg.adjust;
    }
    else {
      resolved.adjust = false
    }

    return resolved;
    }
};

module.exports = Adjust;
