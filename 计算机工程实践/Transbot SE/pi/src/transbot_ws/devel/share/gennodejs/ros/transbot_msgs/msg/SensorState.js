// Auto-generated. Do not edit!

// (in-package transbot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SensorState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.bumper = null;
      this.cliff = null;
      this.sonar = null;
      this.illumination = null;
      this.led = null;
      this.button = null;
      this.torque = null;
      this.left_encoder = null;
      this.right_encoder = null;
      this.battery = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('bumper')) {
        this.bumper = initObj.bumper
      }
      else {
        this.bumper = 0;
      }
      if (initObj.hasOwnProperty('cliff')) {
        this.cliff = initObj.cliff
      }
      else {
        this.cliff = 0.0;
      }
      if (initObj.hasOwnProperty('sonar')) {
        this.sonar = initObj.sonar
      }
      else {
        this.sonar = 0.0;
      }
      if (initObj.hasOwnProperty('illumination')) {
        this.illumination = initObj.illumination
      }
      else {
        this.illumination = 0.0;
      }
      if (initObj.hasOwnProperty('led')) {
        this.led = initObj.led
      }
      else {
        this.led = 0;
      }
      if (initObj.hasOwnProperty('button')) {
        this.button = initObj.button
      }
      else {
        this.button = 0;
      }
      if (initObj.hasOwnProperty('torque')) {
        this.torque = initObj.torque
      }
      else {
        this.torque = false;
      }
      if (initObj.hasOwnProperty('left_encoder')) {
        this.left_encoder = initObj.left_encoder
      }
      else {
        this.left_encoder = 0;
      }
      if (initObj.hasOwnProperty('right_encoder')) {
        this.right_encoder = initObj.right_encoder
      }
      else {
        this.right_encoder = 0;
      }
      if (initObj.hasOwnProperty('battery')) {
        this.battery = initObj.battery
      }
      else {
        this.battery = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [bumper]
    bufferOffset = _serializer.uint8(obj.bumper, buffer, bufferOffset);
    // Serialize message field [cliff]
    bufferOffset = _serializer.float32(obj.cliff, buffer, bufferOffset);
    // Serialize message field [sonar]
    bufferOffset = _serializer.float32(obj.sonar, buffer, bufferOffset);
    // Serialize message field [illumination]
    bufferOffset = _serializer.float32(obj.illumination, buffer, bufferOffset);
    // Serialize message field [led]
    bufferOffset = _serializer.uint8(obj.led, buffer, bufferOffset);
    // Serialize message field [button]
    bufferOffset = _serializer.uint8(obj.button, buffer, bufferOffset);
    // Serialize message field [torque]
    bufferOffset = _serializer.bool(obj.torque, buffer, bufferOffset);
    // Serialize message field [left_encoder]
    bufferOffset = _serializer.int32(obj.left_encoder, buffer, bufferOffset);
    // Serialize message field [right_encoder]
    bufferOffset = _serializer.int32(obj.right_encoder, buffer, bufferOffset);
    // Serialize message field [battery]
    bufferOffset = _serializer.float32(obj.battery, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorState
    let len;
    let data = new SensorState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [bumper]
    data.bumper = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cliff]
    data.cliff = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sonar]
    data.sonar = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [illumination]
    data.illumination = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [led]
    data.led = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [button]
    data.button = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [torque]
    data.torque = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_encoder]
    data.left_encoder = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [right_encoder]
    data.right_encoder = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [battery]
    data.battery = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'transbot_msgs/SensorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7250c1dc0b61c4190e78f528f599285f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ########################################
    # CONSTANTS
    ########################################
    # Bumper states (states are combined, when multiple bumpers are pressed)
    uint8 BUMPER_FORWARD  = 1
    uint8 BUMPER_BACKWARD = 2
    
    # Cliff sensor states (states are combined, when multiple cliff sensors are triggered)
    uint8 CLIFF = 1
    
    # Sonar sensor states (states are combined, when multiple sonar sensors are triggered)
    uint8 SONAR = 1
    
    # Illumination sensor (states are combined, when multiple illumination sensors are triggered) 
    uint8 ILLUMINATION = 1
    
    # Button states (states are combined, when multiple buttons are pressed)
    uint8 BUTTON0 = 1
    uint8 BUTTON1 = 2
    
    # Motor errors
    uint8 ERROR_LEFT_MOTOR  = 1
    uint8 ERROR_RIGHT_MOTOR = 2
    
    # Motor torque
    uint8 TORQUE_ON  = 1
    uint8 TORQUE_OFF = 2
    
    ########################################
    # Messages
    ########################################
    Header  header
    uint8   bumper
    float32 cliff
    float32 sonar
    float32 illumination
    uint8   led
    uint8  button
    bool   torque
    int32  left_encoder    # (-2,147,483,648 ~ 2,147,483,647)
    int32  right_encoder   # (-2,147,483,648 ~ 2,147,483,647)
    float32  battery
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SensorState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.bumper !== undefined) {
      resolved.bumper = msg.bumper;
    }
    else {
      resolved.bumper = 0
    }

    if (msg.cliff !== undefined) {
      resolved.cliff = msg.cliff;
    }
    else {
      resolved.cliff = 0.0
    }

    if (msg.sonar !== undefined) {
      resolved.sonar = msg.sonar;
    }
    else {
      resolved.sonar = 0.0
    }

    if (msg.illumination !== undefined) {
      resolved.illumination = msg.illumination;
    }
    else {
      resolved.illumination = 0.0
    }

    if (msg.led !== undefined) {
      resolved.led = msg.led;
    }
    else {
      resolved.led = 0
    }

    if (msg.button !== undefined) {
      resolved.button = msg.button;
    }
    else {
      resolved.button = 0
    }

    if (msg.torque !== undefined) {
      resolved.torque = msg.torque;
    }
    else {
      resolved.torque = false
    }

    if (msg.left_encoder !== undefined) {
      resolved.left_encoder = msg.left_encoder;
    }
    else {
      resolved.left_encoder = 0
    }

    if (msg.right_encoder !== undefined) {
      resolved.right_encoder = msg.right_encoder;
    }
    else {
      resolved.right_encoder = 0
    }

    if (msg.battery !== undefined) {
      resolved.battery = msg.battery;
    }
    else {
      resolved.battery = 0.0
    }

    return resolved;
    }
};

// Constants for message
SensorState.Constants = {
  BUMPER_FORWARD: 1,
  BUMPER_BACKWARD: 2,
  CLIFF: 1,
  SONAR: 1,
  ILLUMINATION: 1,
  BUTTON0: 1,
  BUTTON1: 2,
  ERROR_LEFT_MOTOR: 1,
  ERROR_RIGHT_MOTOR: 2,
  TORQUE_ON: 1,
  TORQUE_OFF: 2,
}

module.exports = SensorState;
