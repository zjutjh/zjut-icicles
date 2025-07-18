// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Moment = require('./Moment.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MomentArrayStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.moments = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('moments')) {
        this.moments = initObj.moments
      }
      else {
        this.moments = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MomentArrayStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [moments]
    // Serialize the length for message field [moments]
    bufferOffset = _serializer.uint32(obj.moments.length, buffer, bufferOffset);
    obj.moments.forEach((val) => {
      bufferOffset = Moment.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MomentArrayStamped
    let len;
    let data = new MomentArrayStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [moments]
    // Deserialize array length for message field [moments]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.moments = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.moments[i] = Moment.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 224 * object.moments.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/MomentArrayStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28ac0beb70b037acf76c3bed71b679a9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    Moment[] moments
    
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
    MSG: opencv_apps/Moment
    # spatial moments
    float64 m00
    float64 m10
    float64 m01
    float64 m20
    float64 m11
    float64 m02
    float64 m30
    float64 m21
    float64 m12
    float64 m03
    
    # central moments
    float64 mu20
    float64 mu11
    float64 mu02
    float64 mu30
    float64 mu21
    float64 mu12
    float64 mu03
    
    # central normalized moments
    float64 nu20
    float64 nu11
    float64 nu02
    float64 nu30
    float64 nu21
    float64 nu12
    float64 nu03
    
    # center of mass m10/m00, m01/m00
    Point2D center
    float64 length
    float64 area
    
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
    const resolved = new MomentArrayStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.moments !== undefined) {
      resolved.moments = new Array(msg.moments.length);
      for (let i = 0; i < resolved.moments.length; ++i) {
        resolved.moments[i] = Moment.Resolve(msg.moments[i]);
      }
    }
    else {
      resolved.moments = []
    }

    return resolved;
    }
};

module.exports = MomentArrayStamped;
