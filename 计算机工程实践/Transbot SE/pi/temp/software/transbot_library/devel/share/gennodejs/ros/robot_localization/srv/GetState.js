// Auto-generated. Do not edit!

// (in-package robot_localization.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time_stamp = null;
      this.frame_id = null;
    }
    else {
      if (initObj.hasOwnProperty('time_stamp')) {
        this.time_stamp = initObj.time_stamp
      }
      else {
        this.time_stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('frame_id')) {
        this.frame_id = initObj.frame_id
      }
      else {
        this.frame_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetStateRequest
    // Serialize message field [time_stamp]
    bufferOffset = _serializer.time(obj.time_stamp, buffer, bufferOffset);
    // Serialize message field [frame_id]
    bufferOffset = _serializer.string(obj.frame_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetStateRequest
    let len;
    let data = new GetStateRequest(null);
    // Deserialize message field [time_stamp]
    data.time_stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [frame_id]
    data.frame_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.frame_id.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_localization/GetStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '853815113280ed7c4ea64ad795f27171';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time time_stamp
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetStateRequest(null);
    if (msg.time_stamp !== undefined) {
      resolved.time_stamp = msg.time_stamp;
    }
    else {
      resolved.time_stamp = {secs: 0, nsecs: 0}
    }

    if (msg.frame_id !== undefined) {
      resolved.frame_id = msg.frame_id;
    }
    else {
      resolved.frame_id = ''
    }

    return resolved;
    }
};

class GetStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
      this.covariance = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = new Array(15).fill(0);
      }
      if (initObj.hasOwnProperty('covariance')) {
        this.covariance = initObj.covariance
      }
      else {
        this.covariance = new Array(225).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetStateResponse
    // Check that the constant length array field [state] has the right length
    if (obj.state.length !== 15) {
      throw new Error('Unable to serialize array field state - length must be 15')
    }
    // Serialize message field [state]
    bufferOffset = _arraySerializer.float64(obj.state, buffer, bufferOffset, 15);
    // Check that the constant length array field [covariance] has the right length
    if (obj.covariance.length !== 225) {
      throw new Error('Unable to serialize array field covariance - length must be 225')
    }
    // Serialize message field [covariance]
    bufferOffset = _arraySerializer.float64(obj.covariance, buffer, bufferOffset, 225);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetStateResponse
    let len;
    let data = new GetStateResponse(null);
    // Deserialize message field [state]
    data.state = _arrayDeserializer.float64(buffer, bufferOffset, 15)
    // Deserialize message field [covariance]
    data.covariance = _arrayDeserializer.float64(buffer, bufferOffset, 225)
    return data;
  }

  static getMessageSize(object) {
    return 1920;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_localization/GetStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8d49e6249cf8371736e3286b16a7ce83';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # State vector: x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az
    float64[15] state
    
    # Covariance matrix in row-major order
    float64[225] covariance
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetStateResponse(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = new Array(15).fill(0)
    }

    if (msg.covariance !== undefined) {
      resolved.covariance = msg.covariance;
    }
    else {
      resolved.covariance = new Array(225).fill(0)
    }

    return resolved;
    }
};

module.exports = {
  Request: GetStateRequest,
  Response: GetStateResponse,
  md5sum() { return 'b143410e9c7f7be208eedf8f691d8069'; },
  datatype() { return 'robot_localization/GetState'; }
};
