// Auto-generated. Do not edit!

// (in-package transbot_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class PatrolRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Commond = null;
      this.LineScaling = null;
      this.RotationScaling = null;
    }
    else {
      if (initObj.hasOwnProperty('Commond')) {
        this.Commond = initObj.Commond
      }
      else {
        this.Commond = '';
      }
      if (initObj.hasOwnProperty('LineScaling')) {
        this.LineScaling = initObj.LineScaling
      }
      else {
        this.LineScaling = 0.0;
      }
      if (initObj.hasOwnProperty('RotationScaling')) {
        this.RotationScaling = initObj.RotationScaling
      }
      else {
        this.RotationScaling = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PatrolRequest
    // Serialize message field [Commond]
    bufferOffset = _serializer.string(obj.Commond, buffer, bufferOffset);
    // Serialize message field [LineScaling]
    bufferOffset = _serializer.float32(obj.LineScaling, buffer, bufferOffset);
    // Serialize message field [RotationScaling]
    bufferOffset = _serializer.float32(obj.RotationScaling, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PatrolRequest
    let len;
    let data = new PatrolRequest(null);
    // Deserialize message field [Commond]
    data.Commond = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [LineScaling]
    data.LineScaling = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [RotationScaling]
    data.RotationScaling = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.Commond.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transbot_msgs/PatrolRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '54a9530f2d4ac2afe60a14f4cfd8803a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #request
    string  Commond
    float32 LineScaling
    float32 RotationScaling
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PatrolRequest(null);
    if (msg.Commond !== undefined) {
      resolved.Commond = msg.Commond;
    }
    else {
      resolved.Commond = ''
    }

    if (msg.LineScaling !== undefined) {
      resolved.LineScaling = msg.LineScaling;
    }
    else {
      resolved.LineScaling = 0.0
    }

    if (msg.RotationScaling !== undefined) {
      resolved.RotationScaling = msg.RotationScaling;
    }
    else {
      resolved.RotationScaling = 0.0
    }

    return resolved;
    }
};

class PatrolResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PatrolResponse
    // Serialize message field [result]
    bufferOffset = _serializer.bool(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PatrolResponse
    let len;
    let data = new PatrolResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transbot_msgs/PatrolResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eb13ac1f1354ccecb7941ee8fa2192e8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #response
    bool result
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PatrolResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = false
    }

    return resolved;
    }
};

module.exports = {
  Request: PatrolRequest,
  Response: PatrolResponse,
  md5sum() { return '725a414bc8766f0cf2b2c0b5f17047e6'; },
  datatype() { return 'transbot_msgs/Patrol'; }
};
