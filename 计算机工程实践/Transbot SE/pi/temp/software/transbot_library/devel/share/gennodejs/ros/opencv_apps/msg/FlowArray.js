// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Flow = require('./Flow.js');

//-----------------------------------------------------------

class FlowArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.flow = null;
    }
    else {
      if (initObj.hasOwnProperty('flow')) {
        this.flow = initObj.flow
      }
      else {
        this.flow = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlowArray
    // Serialize message field [flow]
    // Serialize the length for message field [flow]
    bufferOffset = _serializer.uint32(obj.flow.length, buffer, bufferOffset);
    obj.flow.forEach((val) => {
      bufferOffset = Flow.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlowArray
    let len;
    let data = new FlowArray(null);
    // Deserialize message field [flow]
    // Deserialize array length for message field [flow]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.flow = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.flow[i] = Flow.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 32 * object.flow.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/FlowArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fe292dca56eb3673cd698ea9ef841962';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Flow[] flow
    
    ================================================================================
    MSG: opencv_apps/Flow
    Point2D point
    Point2D velocity
    
    ================================================================================
    MSG: opencv_apps/Point2D
    float64 x
    float64 y
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FlowArray(null);
    if (msg.flow !== undefined) {
      resolved.flow = new Array(msg.flow.length);
      for (let i = 0; i < resolved.flow.length; ++i) {
        resolved.flow[i] = Flow.Resolve(msg.flow[i]);
      }
    }
    else {
      resolved.flow = []
    }

    return resolved;
    }
};

module.exports = FlowArray;
