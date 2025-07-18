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

class HeadlightRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Headlight = null;
    }
    else {
      if (initObj.hasOwnProperty('Headlight')) {
        this.Headlight = initObj.Headlight
      }
      else {
        this.Headlight = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HeadlightRequest
    // Serialize message field [Headlight]
    bufferOffset = _serializer.int32(obj.Headlight, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HeadlightRequest
    let len;
    let data = new HeadlightRequest(null);
    // Deserialize message field [Headlight]
    data.Headlight = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transbot_msgs/HeadlightRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd3f47b9f75d32f762f5837d105474513';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #request
    int32 Headlight
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HeadlightRequest(null);
    if (msg.Headlight !== undefined) {
      resolved.Headlight = msg.Headlight;
    }
    else {
      resolved.Headlight = 0
    }

    return resolved;
    }
};

class HeadlightResponse {
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
    // Serializes a message object of type HeadlightResponse
    // Serialize message field [result]
    bufferOffset = _serializer.bool(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HeadlightResponse
    let len;
    let data = new HeadlightResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transbot_msgs/HeadlightResponse';
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
    const resolved = new HeadlightResponse(null);
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
  Request: HeadlightRequest,
  Response: HeadlightResponse,
  md5sum() { return '39cb7e9dbd56dfa74d38f52f3463c89d'; },
  datatype() { return 'transbot_msgs/Headlight'; }
};
