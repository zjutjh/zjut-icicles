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

class CamDeviceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.GetGev = null;
    }
    else {
      if (initObj.hasOwnProperty('GetGev')) {
        this.GetGev = initObj.GetGev
      }
      else {
        this.GetGev = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CamDeviceRequest
    // Serialize message field [GetGev]
    bufferOffset = _serializer.string(obj.GetGev, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CamDeviceRequest
    let len;
    let data = new CamDeviceRequest(null);
    // Deserialize message field [GetGev]
    data.GetGev = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.GetGev.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transbot_msgs/CamDeviceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '007978485738a378d13c2abf3d31a3ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #request
    string GetGev
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CamDeviceRequest(null);
    if (msg.GetGev !== undefined) {
      resolved.GetGev = msg.GetGev;
    }
    else {
      resolved.GetGev = ''
    }

    return resolved;
    }
};

class CamDeviceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.camDevice = null;
    }
    else {
      if (initObj.hasOwnProperty('camDevice')) {
        this.camDevice = initObj.camDevice
      }
      else {
        this.camDevice = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CamDeviceResponse
    // Serialize message field [camDevice]
    bufferOffset = _serializer.string(obj.camDevice, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CamDeviceResponse
    let len;
    let data = new CamDeviceResponse(null);
    // Deserialize message field [camDevice]
    data.camDevice = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.camDevice.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transbot_msgs/CamDeviceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '77bcfcb9f0516ec55f1e2884e0762f17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #response
    string camDevice
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CamDeviceResponse(null);
    if (msg.camDevice !== undefined) {
      resolved.camDevice = msg.camDevice;
    }
    else {
      resolved.camDevice = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: CamDeviceRequest,
  Response: CamDeviceResponse,
  md5sum() { return '8be1511d89aeca50b4c34cbc069c61f5'; },
  datatype() { return 'transbot_msgs/CamDevice'; }
};
