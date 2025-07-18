// Auto-generated. Do not edit!

// (in-package view_detect.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class YoloResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.image_id = null;
      this.class_names = null;
      this.counts = null;
    }
    else {
      if (initObj.hasOwnProperty('image_id')) {
        this.image_id = initObj.image_id
      }
      else {
        this.image_id = '';
      }
      if (initObj.hasOwnProperty('class_names')) {
        this.class_names = initObj.class_names
      }
      else {
        this.class_names = [];
      }
      if (initObj.hasOwnProperty('counts')) {
        this.counts = initObj.counts
      }
      else {
        this.counts = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YoloResult
    // Serialize message field [image_id]
    bufferOffset = _serializer.string(obj.image_id, buffer, bufferOffset);
    // Serialize message field [class_names]
    bufferOffset = _arraySerializer.string(obj.class_names, buffer, bufferOffset, null);
    // Serialize message field [counts]
    bufferOffset = _arraySerializer.int32(obj.counts, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YoloResult
    let len;
    let data = new YoloResult(null);
    // Deserialize message field [image_id]
    data.image_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [class_names]
    data.class_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [counts]
    data.counts = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.image_id.length;
    object.class_names.forEach((val) => {
      length += 4 + val.length;
    });
    length += 4 * object.counts.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'view_detect/YoloResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '52917fd0d63c30fa5cbba61ad8a19792';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # YOLO检测结果消息
    string image_id        # 图像ID或文件名
    string[] class_names   # 检测到的类别名称
    int32[] counts         # 每个类别的数量 
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new YoloResult(null);
    if (msg.image_id !== undefined) {
      resolved.image_id = msg.image_id;
    }
    else {
      resolved.image_id = ''
    }

    if (msg.class_names !== undefined) {
      resolved.class_names = msg.class_names;
    }
    else {
      resolved.class_names = []
    }

    if (msg.counts !== undefined) {
      resolved.counts = msg.counts;
    }
    else {
      resolved.counts = []
    }

    return resolved;
    }
};

module.exports = YoloResult;
