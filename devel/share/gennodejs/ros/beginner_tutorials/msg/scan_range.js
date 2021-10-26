// Auto-generated. Do not edit!

// (in-package beginner_tutorials.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class scan_range {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.closest_point = null;
      this.farthest_point = null;
    }
    else {
      if (initObj.hasOwnProperty('closest_point')) {
        this.closest_point = initObj.closest_point
      }
      else {
        this.closest_point = 0.0;
      }
      if (initObj.hasOwnProperty('farthest_point')) {
        this.farthest_point = initObj.farthest_point
      }
      else {
        this.farthest_point = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type scan_range
    // Serialize message field [closest_point]
    bufferOffset = _serializer.float64(obj.closest_point, buffer, bufferOffset);
    // Serialize message field [farthest_point]
    bufferOffset = _serializer.float64(obj.farthest_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type scan_range
    let len;
    let data = new scan_range(null);
    // Deserialize message field [closest_point]
    data.closest_point = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [farthest_point]
    data.farthest_point = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beginner_tutorials/scan_range';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d10e7530caaa326bc93d51700c285d9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 closest_point        # minimum range value [m]
    float64 farthest_point		 # maximum range value [m]
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new scan_range(null);
    if (msg.closest_point !== undefined) {
      resolved.closest_point = msg.closest_point;
    }
    else {
      resolved.closest_point = 0.0
    }

    if (msg.farthest_point !== undefined) {
      resolved.farthest_point = msg.farthest_point;
    }
    else {
      resolved.farthest_point = 0.0
    }

    return resolved;
    }
};

module.exports = scan_range;
