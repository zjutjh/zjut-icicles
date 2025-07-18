// Auto-generated. Do not edit!

// (in-package transbot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class General {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Graphics = null;
      this.TrackState = null;
    }
    else {
      if (initObj.hasOwnProperty('Graphics')) {
        this.Graphics = initObj.Graphics
      }
      else {
        this.Graphics = '';
      }
      if (initObj.hasOwnProperty('TrackState')) {
        this.TrackState = initObj.TrackState
      }
      else {
        this.TrackState = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type General
    // Serialize message field [Graphics]
    bufferOffset = _serializer.string(obj.Graphics, buffer, bufferOffset);
    // Serialize message field [TrackState]
    bufferOffset = _serializer.string(obj.TrackState, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type General
    let len;
    let data = new General(null);
    // Deserialize message field [Graphics]
    data.Graphics = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [TrackState]
    data.TrackState = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.Graphics.length;
    length += object.TrackState.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'transbot_msgs/General';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '17ceb36a2d5cf93a882109ffc0506c61';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Graphics
    string TrackState
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new General(null);
    if (msg.Graphics !== undefined) {
      resolved.Graphics = msg.Graphics;
    }
    else {
      resolved.Graphics = ''
    }

    if (msg.TrackState !== undefined) {
      resolved.TrackState = msg.TrackState;
    }
    else {
      resolved.TrackState = ''
    }

    return resolved;
    }
};

module.exports = General;
