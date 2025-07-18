// Auto-generated. Do not edit!

// (in-package yahboomcar_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Target {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frame_id = null;
      this.stamp = null;
      this.scores = null;
      this.ptx = null;
      this.pty = null;
      this.distw = null;
      this.disth = null;
      this.centerx = null;
      this.centery = null;
    }
    else {
      if (initObj.hasOwnProperty('frame_id')) {
        this.frame_id = initObj.frame_id
      }
      else {
        this.frame_id = '';
      }
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('scores')) {
        this.scores = initObj.scores
      }
      else {
        this.scores = 0.0;
      }
      if (initObj.hasOwnProperty('ptx')) {
        this.ptx = initObj.ptx
      }
      else {
        this.ptx = 0.0;
      }
      if (initObj.hasOwnProperty('pty')) {
        this.pty = initObj.pty
      }
      else {
        this.pty = 0.0;
      }
      if (initObj.hasOwnProperty('distw')) {
        this.distw = initObj.distw
      }
      else {
        this.distw = 0.0;
      }
      if (initObj.hasOwnProperty('disth')) {
        this.disth = initObj.disth
      }
      else {
        this.disth = 0.0;
      }
      if (initObj.hasOwnProperty('centerx')) {
        this.centerx = initObj.centerx
      }
      else {
        this.centerx = 0.0;
      }
      if (initObj.hasOwnProperty('centery')) {
        this.centery = initObj.centery
      }
      else {
        this.centery = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Target
    // Serialize message field [frame_id]
    bufferOffset = _serializer.string(obj.frame_id, buffer, bufferOffset);
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [scores]
    bufferOffset = _serializer.float32(obj.scores, buffer, bufferOffset);
    // Serialize message field [ptx]
    bufferOffset = _serializer.float32(obj.ptx, buffer, bufferOffset);
    // Serialize message field [pty]
    bufferOffset = _serializer.float32(obj.pty, buffer, bufferOffset);
    // Serialize message field [distw]
    bufferOffset = _serializer.float32(obj.distw, buffer, bufferOffset);
    // Serialize message field [disth]
    bufferOffset = _serializer.float32(obj.disth, buffer, bufferOffset);
    // Serialize message field [centerx]
    bufferOffset = _serializer.float32(obj.centerx, buffer, bufferOffset);
    // Serialize message field [centery]
    bufferOffset = _serializer.float32(obj.centery, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Target
    let len;
    let data = new Target(null);
    // Deserialize message field [frame_id]
    data.frame_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [scores]
    data.scores = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ptx]
    data.ptx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pty]
    data.pty = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distw]
    data.distw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [disth]
    data.disth = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [centerx]
    data.centerx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [centery]
    data.centery = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.frame_id.length;
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yahboomcar_msgs/Target';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ed563213df8d0c63d0090b5a0b306f53';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string frame_id
    time stamp
    float32 scores
    float32 ptx
    float32 pty
    float32 distw
    float32 disth
    float32 centerx
    float32 centery
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Target(null);
    if (msg.frame_id !== undefined) {
      resolved.frame_id = msg.frame_id;
    }
    else {
      resolved.frame_id = ''
    }

    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.scores !== undefined) {
      resolved.scores = msg.scores;
    }
    else {
      resolved.scores = 0.0
    }

    if (msg.ptx !== undefined) {
      resolved.ptx = msg.ptx;
    }
    else {
      resolved.ptx = 0.0
    }

    if (msg.pty !== undefined) {
      resolved.pty = msg.pty;
    }
    else {
      resolved.pty = 0.0
    }

    if (msg.distw !== undefined) {
      resolved.distw = msg.distw;
    }
    else {
      resolved.distw = 0.0
    }

    if (msg.disth !== undefined) {
      resolved.disth = msg.disth;
    }
    else {
      resolved.disth = 0.0
    }

    if (msg.centerx !== undefined) {
      resolved.centerx = msg.centerx;
    }
    else {
      resolved.centerx = 0.0
    }

    if (msg.centery !== undefined) {
      resolved.centery = msg.centery;
    }
    else {
      resolved.centery = 0.0
    }

    return resolved;
    }
};

module.exports = Target;
