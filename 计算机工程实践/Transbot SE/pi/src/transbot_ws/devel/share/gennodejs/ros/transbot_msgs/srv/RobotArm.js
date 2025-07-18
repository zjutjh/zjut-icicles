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

let Arm = require('../msg/Arm.js');

//-----------------------------------------------------------

class RobotArmRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.apply = null;
    }
    else {
      if (initObj.hasOwnProperty('apply')) {
        this.apply = initObj.apply
      }
      else {
        this.apply = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotArmRequest
    // Serialize message field [apply]
    bufferOffset = _serializer.string(obj.apply, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotArmRequest
    let len;
    let data = new RobotArmRequest(null);
    // Deserialize message field [apply]
    data.apply = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.apply.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transbot_msgs/RobotArmRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8e8b3a3b74f38e6cd4cf9904695ae0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #request
    string apply
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotArmRequest(null);
    if (msg.apply !== undefined) {
      resolved.apply = msg.apply;
    }
    else {
      resolved.apply = ''
    }

    return resolved;
    }
};

class RobotArmResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.RobotArm = null;
    }
    else {
      if (initObj.hasOwnProperty('RobotArm')) {
        this.RobotArm = initObj.RobotArm
      }
      else {
        this.RobotArm = new Arm();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotArmResponse
    // Serialize message field [RobotArm]
    bufferOffset = Arm.serialize(obj.RobotArm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotArmResponse
    let len;
    let data = new RobotArmResponse(null);
    // Deserialize message field [RobotArm]
    data.RobotArm = Arm.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Arm.getMessageSize(object.RobotArm);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transbot_msgs/RobotArmResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ce432449e1f32c24abf23350648dc66d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #response
    Arm RobotArm
    
    
    ================================================================================
    MSG: transbot_msgs/Arm
    Joint[] joint
    
    ================================================================================
    MSG: transbot_msgs/Joint
    int32 id
    int32 run_time
    float32 angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotArmResponse(null);
    if (msg.RobotArm !== undefined) {
      resolved.RobotArm = Arm.Resolve(msg.RobotArm)
    }
    else {
      resolved.RobotArm = new Arm()
    }

    return resolved;
    }
};

module.exports = {
  Request: RobotArmRequest,
  Response: RobotArmResponse,
  md5sum() { return '02b16e175f9698037e15289959eba75b'; },
  datatype() { return 'transbot_msgs/RobotArm'; }
};
