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

class Edition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.edition = null;
    }
    else {
      if (initObj.hasOwnProperty('edition')) {
        this.edition = initObj.edition
      }
      else {
        this.edition = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Edition
    // Serialize message field [edition]
    bufferOffset = _serializer.float32(obj.edition, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Edition
    let len;
    let data = new Edition(null);
    // Deserialize message field [edition]
    data.edition = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'transbot_msgs/Edition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '373df2b35ba40a1a8b8afa0bf078b756';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 edition
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Edition(null);
    if (msg.edition !== undefined) {
      resolved.edition = msg.edition;
    }
    else {
      resolved.edition = 0.0
    }

    return resolved;
    }
};

module.exports = Edition;
