// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Point2D = require('./Point2D.js');

//-----------------------------------------------------------

class Line {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pt1 = null;
      this.pt2 = null;
    }
    else {
      if (initObj.hasOwnProperty('pt1')) {
        this.pt1 = initObj.pt1
      }
      else {
        this.pt1 = new Point2D();
      }
      if (initObj.hasOwnProperty('pt2')) {
        this.pt2 = initObj.pt2
      }
      else {
        this.pt2 = new Point2D();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Line
    // Serialize message field [pt1]
    bufferOffset = Point2D.serialize(obj.pt1, buffer, bufferOffset);
    // Serialize message field [pt2]
    bufferOffset = Point2D.serialize(obj.pt2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Line
    let len;
    let data = new Line(null);
    // Deserialize message field [pt1]
    data.pt1 = Point2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [pt2]
    data.pt2 = Point2D.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/Line';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a1419010b3fc4549e3f450018363d000';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Line(null);
    if (msg.pt1 !== undefined) {
      resolved.pt1 = Point2D.Resolve(msg.pt1)
    }
    else {
      resolved.pt1 = new Point2D()
    }

    if (msg.pt2 !== undefined) {
      resolved.pt2 = Point2D.Resolve(msg.pt2)
    }
    else {
      resolved.pt2 = new Point2D()
    }

    return resolved;
    }
};

module.exports = Line;
