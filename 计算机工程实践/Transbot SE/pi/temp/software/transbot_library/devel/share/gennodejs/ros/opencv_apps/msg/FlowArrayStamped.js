// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Flow = require('./Flow.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FlowArrayStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.flow = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('flow')) {
        this.flow = initObj.flow
      }
      else {
        this.flow = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlowArrayStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [flow]
    // Serialize the length for message field [flow]
    bufferOffset = _serializer.uint32(obj.flow.length, buffer, bufferOffset);
    obj.flow.forEach((val) => {
      bufferOffset = Flow.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlowArrayStamped
    let len;
    let data = new FlowArrayStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [flow]
    // Deserialize array length for message field [flow]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.flow = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.flow[i] = Flow.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 32 * object.flow.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/FlowArrayStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b55faf909449963372b92417925b68cc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    Flow[] flow
    
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
    
    ================================================================================
    MSG: opencv_apps/Flow
    Point2D point
    Point2D velocity
    
    ================================================================================
    MSG: opencv_apps/Point2D
    float64 x
    float64 y
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FlowArrayStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.flow !== undefined) {
      resolved.flow = new Array(msg.flow.length);
      for (let i = 0; i < resolved.flow.length; ++i) {
        resolved.flow[i] = Flow.Resolve(msg.flow[i]);
      }
    }
    else {
      resolved.flow = []
    }

    return resolved;
    }
};

module.exports = FlowArrayStamped;
