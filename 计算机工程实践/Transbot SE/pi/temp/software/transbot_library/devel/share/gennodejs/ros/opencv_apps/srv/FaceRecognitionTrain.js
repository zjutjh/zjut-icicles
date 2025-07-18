// Auto-generated. Do not edit!

// (in-package opencv_apps.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Rect = require('../msg/Rect.js');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class FaceRecognitionTrainRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.images = null;
      this.rects = null;
      this.labels = null;
    }
    else {
      if (initObj.hasOwnProperty('images')) {
        this.images = initObj.images
      }
      else {
        this.images = [];
      }
      if (initObj.hasOwnProperty('rects')) {
        this.rects = initObj.rects
      }
      else {
        this.rects = [];
      }
      if (initObj.hasOwnProperty('labels')) {
        this.labels = initObj.labels
      }
      else {
        this.labels = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FaceRecognitionTrainRequest
    // Serialize message field [images]
    // Serialize the length for message field [images]
    bufferOffset = _serializer.uint32(obj.images.length, buffer, bufferOffset);
    obj.images.forEach((val) => {
      bufferOffset = sensor_msgs.msg.Image.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [rects]
    // Serialize the length for message field [rects]
    bufferOffset = _serializer.uint32(obj.rects.length, buffer, bufferOffset);
    obj.rects.forEach((val) => {
      bufferOffset = Rect.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [labels]
    bufferOffset = _arraySerializer.string(obj.labels, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FaceRecognitionTrainRequest
    let len;
    let data = new FaceRecognitionTrainRequest(null);
    // Deserialize message field [images]
    // Deserialize array length for message field [images]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.images = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.images[i] = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [rects]
    // Deserialize array length for message field [rects]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.rects = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.rects[i] = Rect.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [labels]
    data.labels = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.images.forEach((val) => {
      length += sensor_msgs.msg.Image.getMessageSize(val);
    });
    length += 32 * object.rects.length;
    object.labels.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'opencv_apps/FaceRecognitionTrainRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba188b4bf792edbaf69c7f296a16e0ec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/Image[] images
    Rect[] rects
    string[] labels
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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
    const resolved = new FaceRecognitionTrainRequest(null);
    if (msg.images !== undefined) {
      resolved.images = new Array(msg.images.length);
      for (let i = 0; i < resolved.images.length; ++i) {
        resolved.images[i] = sensor_msgs.msg.Image.Resolve(msg.images[i]);
      }
    }
    else {
      resolved.images = []
    }

    if (msg.rects !== undefined) {
      resolved.rects = new Array(msg.rects.length);
      for (let i = 0; i < resolved.rects.length; ++i) {
        resolved.rects[i] = Rect.Resolve(msg.rects[i]);
      }
    }
    else {
      resolved.rects = []
    }

    if (msg.labels !== undefined) {
      resolved.labels = msg.labels;
    }
    else {
      resolved.labels = []
    }

    return resolved;
    }
};

class FaceRecognitionTrainResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ok = null;
      this.error = null;
    }
    else {
      if (initObj.hasOwnProperty('ok')) {
        this.ok = initObj.ok
      }
      else {
        this.ok = false;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FaceRecognitionTrainResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.string(obj.error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FaceRecognitionTrainResponse
    let len;
    let data = new FaceRecognitionTrainResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.error.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'opencv_apps/FaceRecognitionTrainResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14d6fca830116fb9833d983a296f00ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ok
    string error
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FaceRecognitionTrainResponse(null);
    if (msg.ok !== undefined) {
      resolved.ok = msg.ok;
    }
    else {
      resolved.ok = false
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: FaceRecognitionTrainRequest,
  Response: FaceRecognitionTrainResponse,
  md5sum() { return 'c47a3ceb75cbe248d69217439e66a8e2'; },
  datatype() { return 'opencv_apps/FaceRecognitionTrain'; }
};
