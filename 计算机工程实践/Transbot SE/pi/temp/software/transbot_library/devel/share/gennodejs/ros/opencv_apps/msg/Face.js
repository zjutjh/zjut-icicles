// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Rect = require('./Rect.js');

//-----------------------------------------------------------

class Face {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.face = null;
      this.eyes = null;
      this.label = null;
      this.confidence = null;
    }
    else {
      if (initObj.hasOwnProperty('face')) {
        this.face = initObj.face
      }
      else {
        this.face = new Rect();
      }
      if (initObj.hasOwnProperty('eyes')) {
        this.eyes = initObj.eyes
      }
      else {
        this.eyes = [];
      }
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Face
    // Serialize message field [face]
    bufferOffset = Rect.serialize(obj.face, buffer, bufferOffset);
    // Serialize message field [eyes]
    // Serialize the length for message field [eyes]
    bufferOffset = _serializer.uint32(obj.eyes.length, buffer, bufferOffset);
    obj.eyes.forEach((val) => {
      bufferOffset = Rect.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float64(obj.confidence, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Face
    let len;
    let data = new Face(null);
    // Deserialize message field [face]
    data.face = Rect.deserialize(buffer, bufferOffset);
    // Deserialize message field [eyes]
    // Deserialize array length for message field [eyes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.eyes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.eyes[i] = Rect.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 32 * object.eyes.length;
    length += object.label.length;
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/Face';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a1a50e747b0ca7822ce8611c3ffa7a02';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Rect face
    Rect[] eyes
    string label
    float64 confidence
    
    ================================================================================
    MSG: opencv_apps/Rect
    # opencv Rect data type, x-y is center point
    float64 x
    float64 y
    float64 width
    float64 height
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Face(null);
    if (msg.face !== undefined) {
      resolved.face = Rect.Resolve(msg.face)
    }
    else {
      resolved.face = new Rect()
    }

    if (msg.eyes !== undefined) {
      resolved.eyes = new Array(msg.eyes.length);
      for (let i = 0; i < resolved.eyes.length; ++i) {
        resolved.eyes[i] = Rect.Resolve(msg.eyes[i]);
      }
    }
    else {
      resolved.eyes = []
    }

    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    return resolved;
    }
};

module.exports = Face;
