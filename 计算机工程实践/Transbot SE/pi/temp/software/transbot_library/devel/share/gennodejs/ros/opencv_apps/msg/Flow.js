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

class Flow {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point = null;
      this.velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new Point2D();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new Point2D();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Flow
    // Serialize message field [point]
    bufferOffset = Point2D.serialize(obj.point, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = Point2D.serialize(obj.velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Flow
    let len;
    let data = new Flow(null);
    // Deserialize message field [point]
    data.point = Point2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = Point2D.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/Flow';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd9a9efd88ba39035e78af697593d751';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Flow(null);
    if (msg.point !== undefined) {
      resolved.point = Point2D.Resolve(msg.point)
    }
    else {
      resolved.point = new Point2D()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = Point2D.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new Point2D()
    }

    return resolved;
    }
};

module.exports = Flow;
