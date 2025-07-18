// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Line = require('./Line.js');

//-----------------------------------------------------------

class LineArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lines = null;
    }
    else {
      if (initObj.hasOwnProperty('lines')) {
        this.lines = initObj.lines
      }
      else {
        this.lines = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LineArray
    // Serialize message field [lines]
    // Serialize the length for message field [lines]
    bufferOffset = _serializer.uint32(obj.lines.length, buffer, bufferOffset);
    obj.lines.forEach((val) => {
      bufferOffset = Line.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LineArray
    let len;
    let data = new LineArray(null);
    // Deserialize message field [lines]
    // Deserialize array length for message field [lines]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.lines = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.lines[i] = Line.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 32 * object.lines.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/LineArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2b5441933900cc71528395dda29124da';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Line[] lines
    
    ================================================================================
    MSG: opencv_apps/Line
    Point2D pt1
    Point2D pt2
    
    
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
    const resolved = new LineArray(null);
    if (msg.lines !== undefined) {
      resolved.lines = new Array(msg.lines.length);
      for (let i = 0; i < resolved.lines.length; ++i) {
        resolved.lines[i] = Line.Resolve(msg.lines[i]);
      }
    }
    else {
      resolved.lines = []
    }

    return resolved;
    }
};

module.exports = LineArray;
