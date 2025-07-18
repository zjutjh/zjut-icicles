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
let Size = require('./Size.js');

//-----------------------------------------------------------

class RotatedRect {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle = null;
      this.center = null;
      this.size = null;
    }
    else {
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new Point2D();
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = new Size();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RotatedRect
    // Serialize message field [angle]
    bufferOffset = _serializer.float64(obj.angle, buffer, bufferOffset);
    // Serialize message field [center]
    bufferOffset = Point2D.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = Size.serialize(obj.size, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RotatedRect
    let len;
    let data = new RotatedRect(null);
    // Deserialize message field [angle]
    data.angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [center]
    data.center = Point2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = Size.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/RotatedRect';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0ae60505c52f020176686d0689b8d390';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new RotatedRect(null);
    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

    if (msg.center !== undefined) {
      resolved.center = Point2D.Resolve(msg.center)
    }
    else {
      resolved.center = new Point2D()
    }

    if (msg.size !== undefined) {
      resolved.size = Size.Resolve(msg.size)
    }
    else {
      resolved.size = new Size()
    }

    return resolved;
    }
};

module.exports = RotatedRect;
