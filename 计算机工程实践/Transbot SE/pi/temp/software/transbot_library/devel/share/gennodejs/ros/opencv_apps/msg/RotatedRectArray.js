// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RotatedRect = require('./RotatedRect.js');

//-----------------------------------------------------------

class RotatedRectArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rects = null;
    }
    else {
      if (initObj.hasOwnProperty('rects')) {
        this.rects = initObj.rects
      }
      else {
        this.rects = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RotatedRectArray
    // Serialize message field [rects]
    // Serialize the length for message field [rects]
    bufferOffset = _serializer.uint32(obj.rects.length, buffer, bufferOffset);
    obj.rects.forEach((val) => {
      bufferOffset = RotatedRect.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RotatedRectArray
    let len;
    let data = new RotatedRectArray(null);
    // Deserialize message field [rects]
    // Deserialize array length for message field [rects]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.rects = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.rects[i] = RotatedRect.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 40 * object.rects.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/RotatedRectArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a27e397ed2b5b1a633561d324f64d2a6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    RotatedRect[] rects
    
    ================================================================================
    MSG: opencv_apps/RotatedRect
    float64 angle
    Point2D center
    Size size
    
    ================================================================================
    MSG: opencv_apps/Point2D
    float64 x
    float64 y
    
    
    ================================================================================
    MSG: opencv_apps/Size
    float64 width
    float64 height
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RotatedRectArray(null);
    if (msg.rects !== undefined) {
      resolved.rects = new Array(msg.rects.length);
      for (let i = 0; i < resolved.rects.length; ++i) {
        resolved.rects[i] = RotatedRect.Resolve(msg.rects[i]);
      }
    }
    else {
      resolved.rects = []
    }

    return resolved;
    }
};

module.exports = RotatedRectArray;
