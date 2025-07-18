// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Contour = require('./Contour.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ContourArrayStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.contours = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('contours')) {
        this.contours = initObj.contours
      }
      else {
        this.contours = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ContourArrayStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [contours]
    // Serialize the length for message field [contours]
    bufferOffset = _serializer.uint32(obj.contours.length, buffer, bufferOffset);
    obj.contours.forEach((val) => {
      bufferOffset = Contour.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ContourArrayStamped
    let len;
    let data = new ContourArrayStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [contours]
    // Deserialize array length for message field [contours]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.contours = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.contours[i] = Contour.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.contours.forEach((val) => {
      length += Contour.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/ContourArrayStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6bcf2733566be102cf11fc89685fd962';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    Contour[] contours
    
    
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
    MSG: opencv_apps/Contour
    Point2D[] points
    
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
    const resolved = new ContourArrayStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.contours !== undefined) {
      resolved.contours = new Array(msg.contours.length);
      for (let i = 0; i < resolved.contours.length; ++i) {
        resolved.contours[i] = Contour.Resolve(msg.contours[i]);
      }
    }
    else {
      resolved.contours = []
    }

    return resolved;
    }
};

module.exports = ContourArrayStamped;
