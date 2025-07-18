// Auto-generated. Do not edit!

// (in-package opencv_apps.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Point2D = require('./Point2D.js');

//-----------------------------------------------------------

class Moment {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.m00 = null;
      this.m10 = null;
      this.m01 = null;
      this.m20 = null;
      this.m11 = null;
      this.m02 = null;
      this.m30 = null;
      this.m21 = null;
      this.m12 = null;
      this.m03 = null;
      this.mu20 = null;
      this.mu11 = null;
      this.mu02 = null;
      this.mu30 = null;
      this.mu21 = null;
      this.mu12 = null;
      this.mu03 = null;
      this.nu20 = null;
      this.nu11 = null;
      this.nu02 = null;
      this.nu30 = null;
      this.nu21 = null;
      this.nu12 = null;
      this.nu03 = null;
      this.center = null;
      this.length = null;
      this.area = null;
    }
    else {
      if (initObj.hasOwnProperty('m00')) {
        this.m00 = initObj.m00
      }
      else {
        this.m00 = 0.0;
      }
      if (initObj.hasOwnProperty('m10')) {
        this.m10 = initObj.m10
      }
      else {
        this.m10 = 0.0;
      }
      if (initObj.hasOwnProperty('m01')) {
        this.m01 = initObj.m01
      }
      else {
        this.m01 = 0.0;
      }
      if (initObj.hasOwnProperty('m20')) {
        this.m20 = initObj.m20
      }
      else {
        this.m20 = 0.0;
      }
      if (initObj.hasOwnProperty('m11')) {
        this.m11 = initObj.m11
      }
      else {
        this.m11 = 0.0;
      }
      if (initObj.hasOwnProperty('m02')) {
        this.m02 = initObj.m02
      }
      else {
        this.m02 = 0.0;
      }
      if (initObj.hasOwnProperty('m30')) {
        this.m30 = initObj.m30
      }
      else {
        this.m30 = 0.0;
      }
      if (initObj.hasOwnProperty('m21')) {
        this.m21 = initObj.m21
      }
      else {
        this.m21 = 0.0;
      }
      if (initObj.hasOwnProperty('m12')) {
        this.m12 = initObj.m12
      }
      else {
        this.m12 = 0.0;
      }
      if (initObj.hasOwnProperty('m03')) {
        this.m03 = initObj.m03
      }
      else {
        this.m03 = 0.0;
      }
      if (initObj.hasOwnProperty('mu20')) {
        this.mu20 = initObj.mu20
      }
      else {
        this.mu20 = 0.0;
      }
      if (initObj.hasOwnProperty('mu11')) {
        this.mu11 = initObj.mu11
      }
      else {
        this.mu11 = 0.0;
      }
      if (initObj.hasOwnProperty('mu02')) {
        this.mu02 = initObj.mu02
      }
      else {
        this.mu02 = 0.0;
      }
      if (initObj.hasOwnProperty('mu30')) {
        this.mu30 = initObj.mu30
      }
      else {
        this.mu30 = 0.0;
      }
      if (initObj.hasOwnProperty('mu21')) {
        this.mu21 = initObj.mu21
      }
      else {
        this.mu21 = 0.0;
      }
      if (initObj.hasOwnProperty('mu12')) {
        this.mu12 = initObj.mu12
      }
      else {
        this.mu12 = 0.0;
      }
      if (initObj.hasOwnProperty('mu03')) {
        this.mu03 = initObj.mu03
      }
      else {
        this.mu03 = 0.0;
      }
      if (initObj.hasOwnProperty('nu20')) {
        this.nu20 = initObj.nu20
      }
      else {
        this.nu20 = 0.0;
      }
      if (initObj.hasOwnProperty('nu11')) {
        this.nu11 = initObj.nu11
      }
      else {
        this.nu11 = 0.0;
      }
      if (initObj.hasOwnProperty('nu02')) {
        this.nu02 = initObj.nu02
      }
      else {
        this.nu02 = 0.0;
      }
      if (initObj.hasOwnProperty('nu30')) {
        this.nu30 = initObj.nu30
      }
      else {
        this.nu30 = 0.0;
      }
      if (initObj.hasOwnProperty('nu21')) {
        this.nu21 = initObj.nu21
      }
      else {
        this.nu21 = 0.0;
      }
      if (initObj.hasOwnProperty('nu12')) {
        this.nu12 = initObj.nu12
      }
      else {
        this.nu12 = 0.0;
      }
      if (initObj.hasOwnProperty('nu03')) {
        this.nu03 = initObj.nu03
      }
      else {
        this.nu03 = 0.0;
      }
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new Point2D();
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0.0;
      }
      if (initObj.hasOwnProperty('area')) {
        this.area = initObj.area
      }
      else {
        this.area = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Moment
    // Serialize message field [m00]
    bufferOffset = _serializer.float64(obj.m00, buffer, bufferOffset);
    // Serialize message field [m10]
    bufferOffset = _serializer.float64(obj.m10, buffer, bufferOffset);
    // Serialize message field [m01]
    bufferOffset = _serializer.float64(obj.m01, buffer, bufferOffset);
    // Serialize message field [m20]
    bufferOffset = _serializer.float64(obj.m20, buffer, bufferOffset);
    // Serialize message field [m11]
    bufferOffset = _serializer.float64(obj.m11, buffer, bufferOffset);
    // Serialize message field [m02]
    bufferOffset = _serializer.float64(obj.m02, buffer, bufferOffset);
    // Serialize message field [m30]
    bufferOffset = _serializer.float64(obj.m30, buffer, bufferOffset);
    // Serialize message field [m21]
    bufferOffset = _serializer.float64(obj.m21, buffer, bufferOffset);
    // Serialize message field [m12]
    bufferOffset = _serializer.float64(obj.m12, buffer, bufferOffset);
    // Serialize message field [m03]
    bufferOffset = _serializer.float64(obj.m03, buffer, bufferOffset);
    // Serialize message field [mu20]
    bufferOffset = _serializer.float64(obj.mu20, buffer, bufferOffset);
    // Serialize message field [mu11]
    bufferOffset = _serializer.float64(obj.mu11, buffer, bufferOffset);
    // Serialize message field [mu02]
    bufferOffset = _serializer.float64(obj.mu02, buffer, bufferOffset);
    // Serialize message field [mu30]
    bufferOffset = _serializer.float64(obj.mu30, buffer, bufferOffset);
    // Serialize message field [mu21]
    bufferOffset = _serializer.float64(obj.mu21, buffer, bufferOffset);
    // Serialize message field [mu12]
    bufferOffset = _serializer.float64(obj.mu12, buffer, bufferOffset);
    // Serialize message field [mu03]
    bufferOffset = _serializer.float64(obj.mu03, buffer, bufferOffset);
    // Serialize message field [nu20]
    bufferOffset = _serializer.float64(obj.nu20, buffer, bufferOffset);
    // Serialize message field [nu11]
    bufferOffset = _serializer.float64(obj.nu11, buffer, bufferOffset);
    // Serialize message field [nu02]
    bufferOffset = _serializer.float64(obj.nu02, buffer, bufferOffset);
    // Serialize message field [nu30]
    bufferOffset = _serializer.float64(obj.nu30, buffer, bufferOffset);
    // Serialize message field [nu21]
    bufferOffset = _serializer.float64(obj.nu21, buffer, bufferOffset);
    // Serialize message field [nu12]
    bufferOffset = _serializer.float64(obj.nu12, buffer, bufferOffset);
    // Serialize message field [nu03]
    bufferOffset = _serializer.float64(obj.nu03, buffer, bufferOffset);
    // Serialize message field [center]
    bufferOffset = Point2D.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.float64(obj.length, buffer, bufferOffset);
    // Serialize message field [area]
    bufferOffset = _serializer.float64(obj.area, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Moment
    let len;
    let data = new Moment(null);
    // Deserialize message field [m00]
    data.m00 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [m10]
    data.m10 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [m01]
    data.m01 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [m20]
    data.m20 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [m11]
    data.m11 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [m02]
    data.m02 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [m30]
    data.m30 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [m21]
    data.m21 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [m12]
    data.m12 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [m03]
    data.m03 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mu20]
    data.mu20 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mu11]
    data.mu11 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mu02]
    data.mu02 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mu30]
    data.mu30 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mu21]
    data.mu21 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mu12]
    data.mu12 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mu03]
    data.mu03 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [nu20]
    data.nu20 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [nu11]
    data.nu11 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [nu02]
    data.nu02 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [nu30]
    data.nu30 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [nu21]
    data.nu21 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [nu12]
    data.nu12 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [nu03]
    data.nu03 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [center]
    data.center = Point2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [area]
    data.area = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 224;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_apps/Moment';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '560ee3fabfffb4ed4155742d6db8a03c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Moment(null);
    if (msg.m00 !== undefined) {
      resolved.m00 = msg.m00;
    }
    else {
      resolved.m00 = 0.0
    }

    if (msg.m10 !== undefined) {
      resolved.m10 = msg.m10;
    }
    else {
      resolved.m10 = 0.0
    }

    if (msg.m01 !== undefined) {
      resolved.m01 = msg.m01;
    }
    else {
      resolved.m01 = 0.0
    }

    if (msg.m20 !== undefined) {
      resolved.m20 = msg.m20;
    }
    else {
      resolved.m20 = 0.0
    }

    if (msg.m11 !== undefined) {
      resolved.m11 = msg.m11;
    }
    else {
      resolved.m11 = 0.0
    }

    if (msg.m02 !== undefined) {
      resolved.m02 = msg.m02;
    }
    else {
      resolved.m02 = 0.0
    }

    if (msg.m30 !== undefined) {
      resolved.m30 = msg.m30;
    }
    else {
      resolved.m30 = 0.0
    }

    if (msg.m21 !== undefined) {
      resolved.m21 = msg.m21;
    }
    else {
      resolved.m21 = 0.0
    }

    if (msg.m12 !== undefined) {
      resolved.m12 = msg.m12;
    }
    else {
      resolved.m12 = 0.0
    }

    if (msg.m03 !== undefined) {
      resolved.m03 = msg.m03;
    }
    else {
      resolved.m03 = 0.0
    }

    if (msg.mu20 !== undefined) {
      resolved.mu20 = msg.mu20;
    }
    else {
      resolved.mu20 = 0.0
    }

    if (msg.mu11 !== undefined) {
      resolved.mu11 = msg.mu11;
    }
    else {
      resolved.mu11 = 0.0
    }

    if (msg.mu02 !== undefined) {
      resolved.mu02 = msg.mu02;
    }
    else {
      resolved.mu02 = 0.0
    }

    if (msg.mu30 !== undefined) {
      resolved.mu30 = msg.mu30;
    }
    else {
      resolved.mu30 = 0.0
    }

    if (msg.mu21 !== undefined) {
      resolved.mu21 = msg.mu21;
    }
    else {
      resolved.mu21 = 0.0
    }

    if (msg.mu12 !== undefined) {
      resolved.mu12 = msg.mu12;
    }
    else {
      resolved.mu12 = 0.0
    }

    if (msg.mu03 !== undefined) {
      resolved.mu03 = msg.mu03;
    }
    else {
      resolved.mu03 = 0.0
    }

    if (msg.nu20 !== undefined) {
      resolved.nu20 = msg.nu20;
    }
    else {
      resolved.nu20 = 0.0
    }

    if (msg.nu11 !== undefined) {
      resolved.nu11 = msg.nu11;
    }
    else {
      resolved.nu11 = 0.0
    }

    if (msg.nu02 !== undefined) {
      resolved.nu02 = msg.nu02;
    }
    else {
      resolved.nu02 = 0.0
    }

    if (msg.nu30 !== undefined) {
      resolved.nu30 = msg.nu30;
    }
    else {
      resolved.nu30 = 0.0
    }

    if (msg.nu21 !== undefined) {
      resolved.nu21 = msg.nu21;
    }
    else {
      resolved.nu21 = 0.0
    }

    if (msg.nu12 !== undefined) {
      resolved.nu12 = msg.nu12;
    }
    else {
      resolved.nu12 = 0.0
    }

    if (msg.nu03 !== undefined) {
      resolved.nu03 = msg.nu03;
    }
    else {
      resolved.nu03 = 0.0
    }

    if (msg.center !== undefined) {
      resolved.center = Point2D.Resolve(msg.center)
    }
    else {
      resolved.center = new Point2D()
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0.0
    }

    if (msg.area !== undefined) {
      resolved.area = msg.area;
    }
    else {
      resolved.area = 0.0
    }

    return resolved;
    }
};

module.exports = Moment;
