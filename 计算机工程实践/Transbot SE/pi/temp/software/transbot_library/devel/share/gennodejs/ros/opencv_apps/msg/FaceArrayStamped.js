// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Face = require('./Face.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FaceArrayStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.faces = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('faces')) {
        this.faces = initObj.faces
      }
      else {
        this.faces = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FaceArrayStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [faces]
    // Serialize the length for message field [faces]
    bufferOffset = _serializer.uint32(obj.faces.length, buffer, bufferOffset);
    obj.faces.forEach((val) => {
      bufferOffset = Face.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FaceArrayStamped
    let len;
    let data = new FaceArrayStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [faces]
    // Deserialize array length for message field [faces]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.faces = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.faces[i] = Face.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.faces.forEach((val) => {
      length += Face.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/FaceArrayStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a43dedd70c7b2338df14a8f4de0940ef';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    Face[] faces
    
    
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
    MSG: opencv_apps/Face
    Rect face
    Rect[] eyes
    string label
    float64 confidence
    
    ================================================================================
    MSG: opencv_apps/Rect
    # opencv Rect data type, x-y is center point
    float64 x
    float64 y
    float64 width
    float64 height
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FaceArrayStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.faces !== undefined) {
      resolved.faces = new Array(msg.faces.length);
      for (let i = 0; i < resolved.faces.length; ++i) {
        resolved.faces[i] = Face.Resolve(msg.faces[i]);
      }
    }
    else {
      resolved.faces = []
    }

    return resolved;
    }
};

module.exports = FaceArrayStamped;
