// Auto-generated. Do not edit!

// (in-package learning_topic.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Information {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.company = null;
      this.city = null;
    }
    else {
      if (initObj.hasOwnProperty('company')) {
        this.company = initObj.company
      }
      else {
        this.company = '';
      }
      if (initObj.hasOwnProperty('city')) {
        this.city = initObj.city
      }
      else {
        this.city = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Information
    // Serialize message field [company]
    bufferOffset = _serializer.string(obj.company, buffer, bufferOffset);
    // Serialize message field [city]
    bufferOffset = _serializer.string(obj.city, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Information
    let len;
    let data = new Information(null);
    // Deserialize message field [company]
    data.company = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [city]
    data.city = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.company.length;
    length += object.city.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'learning_topic/Information';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f9332e7eae53dbcb74dc13ad7572af4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string company
    string city  
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Information(null);
    if (msg.company !== undefined) {
      resolved.company = msg.company;
    }
    else {
      resolved.company = ''
    }

    if (msg.city !== undefined) {
      resolved.city = msg.city;
    }
    else {
      resolved.city = ''
    }

    return resolved;
    }
};

module.exports = Information;
