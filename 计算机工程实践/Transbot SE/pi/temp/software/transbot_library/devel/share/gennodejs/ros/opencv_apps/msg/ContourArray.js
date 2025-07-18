// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Contour = require('./Contour.js');

//-----------------------------------------------------------

class ContourArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.contours = null;
    }
    else {
      if (initObj.hasOwnProperty('contours')) {
        this.contours = initObj.contours
      }
      else {
        this.contours = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ContourArray
    // Serialize message field [contours]
    // Serialize the length for message field [contours]
    bufferOffset = _serializer.uint32(obj.contours.length, buffer, bufferOffset);
    obj.contours.forEach((val) => {
      bufferOffset = Contour.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ContourArray
    let len;
    let data = new ContourArray(null);
    // Deserialize message field [contours]
    // Deserialize array length for message field [contours]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.contours = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.contours[i] = Contour.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.contours.forEach((val) => {
      length += Contour.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/ContourArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc54374f45559dfed248b316ccf9181d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Contour[] contours
    
    ================================================================================
    MSG: opencv_apps/Contour
    Point2D[] points
    
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
    const resolved = new ContourArray(null);
    if (msg.contours !== undefined) {
      resolved.contours = new Array(msg.contours.length);
      for (let i = 0; i < resolved.contours.length; ++i) {
        resolved.contours[i] = Contour.Resolve(msg.contours[i]);
      }
    }
    else {
      resolved.contours = []
    }

    return resolved;
    }
};

module.exports = ContourArray;
