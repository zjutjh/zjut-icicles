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

class BuzzerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.buzzer = null;
    }
    else {
      if (initObj.hasOwnProperty('buzzer')) {
        this.buzzer = initObj.buzzer
      }
      else {
        this.buzzer = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BuzzerRequest
    // Serialize message field [buzzer]
    bufferOffset = _serializer.int32(obj.buzzer, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BuzzerRequest
    let len;
    let data = new BuzzerRequest(null);
    // Deserialize message field [buzzer]
    data.buzzer = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transbot_msgs/BuzzerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '130d26476a79b1e0eb57138986019cf0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #request
    int32 buzzer
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BuzzerRequest(null);
    if (msg.buzzer !== undefined) {
      resolved.buzzer = msg.buzzer;
    }
    else {
      resolved.buzzer = 0
    }

    return resolved;
    }
};

class BuzzerResponse {
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
    // Serializes a message object of type BuzzerResponse
    // Serialize message field [result]
    bufferOffset = _serializer.bool(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BuzzerResponse
    let len;
    let data = new BuzzerResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transbot_msgs/BuzzerResponse';
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
    const resolved = new BuzzerResponse(null);
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
  Request: BuzzerRequest,
  Response: BuzzerResponse,
  md5sum() { return '32ecc8168750cdefd185aff218d2ce5e'; },
  datatype() { return 'transbot_msgs/Buzzer'; }
};
