// Auto-generated. Do not edit!

// (in-package transbot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Joint = require('./Joint.js');

//-----------------------------------------------------------

class Arm {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint = null;
    }
    else {
      if (initObj.hasOwnProperty('joint')) {
        this.joint = initObj.joint
      }
      else {
        this.joint = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Arm
    // Serialize message field [joint]
    // Serialize the length for message field [joint]
    bufferOffset = _serializer.uint32(obj.joint.length, buffer, bufferOffset);
    obj.joint.forEach((val) => {
      bufferOffset = Joint.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Arm
    let len;
    let data = new Arm(null);
    // Deserialize message field [joint]
    // Deserialize array length for message field [joint]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.joint = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.joint[i] = Joint.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.joint.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'transbot_msgs/Arm';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b46d7c4342769b3898ee5c56a7392dd2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Joint[] joint
    
    ================================================================================
    MSG: transbot_msgs/Joint
    int32 id
    int32 run_time
    float32 angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Arm(null);
    if (msg.joint !== undefined) {
      resolved.joint = new Array(msg.joint.length);
      for (let i = 0; i < resolved.joint.length; ++i) {
        resolved.joint[i] = Joint.Resolve(msg.joint[i]);
      }
    }
    else {
      resolved.joint = []
    }

    return resolved;
    }
};

module.exports = Arm;
