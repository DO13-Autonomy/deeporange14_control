// Auto-generated. Do not edit!

// (in-package deeporange14_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MobilityMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.seq = null;
      this.stamp = null;
      this.left_torque_cmd = null;
      this.right_torque_cmd = null;
      this.au_state = null;
      this.brake_effort = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('seq')) {
        this.seq = initObj.seq
      }
      else {
        this.seq = 0;
      }
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('left_torque_cmd')) {
        this.left_torque_cmd = initObj.left_torque_cmd
      }
      else {
        this.left_torque_cmd = 0.0;
      }
      if (initObj.hasOwnProperty('right_torque_cmd')) {
        this.right_torque_cmd = initObj.right_torque_cmd
      }
      else {
        this.right_torque_cmd = 0.0;
      }
      if (initObj.hasOwnProperty('au_state')) {
        this.au_state = initObj.au_state
      }
      else {
        this.au_state = 0;
      }
      if (initObj.hasOwnProperty('brake_effort')) {
        this.brake_effort = initObj.brake_effort
      }
      else {
        this.brake_effort = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MobilityMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [seq]
    bufferOffset = _serializer.uint32(obj.seq, buffer, bufferOffset);
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [left_torque_cmd]
    bufferOffset = _serializer.float64(obj.left_torque_cmd, buffer, bufferOffset);
    // Serialize message field [right_torque_cmd]
    bufferOffset = _serializer.float64(obj.right_torque_cmd, buffer, bufferOffset);
    // Serialize message field [au_state]
    bufferOffset = _serializer.uint8(obj.au_state, buffer, bufferOffset);
    // Serialize message field [brake_effort]
    bufferOffset = _serializer.uint8(obj.brake_effort, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MobilityMsg
    let len;
    let data = new MobilityMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [seq]
    data.seq = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [left_torque_cmd]
    data.left_torque_cmd = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_torque_cmd]
    data.right_torque_cmd = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [au_state]
    data.au_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [brake_effort]
    data.brake_effort = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 30;
  }

  static datatype() {
    // Returns string type for a message object
    return 'deeporange14_msgs/MobilityMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '66e4d75167be5101a8d6b7cdceb2db0b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #This represents a vector in free space -- currently defined to hold the left and right track #velocities
    Header header
      uint32 seq
      time stamp
    
    float64 left_torque_cmd
    float64 right_torque_cmd
    uint8 au_state 
    uint8 brake_effort
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MobilityMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.seq !== undefined) {
      resolved.seq = msg.seq;
    }
    else {
      resolved.seq = 0
    }

    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.left_torque_cmd !== undefined) {
      resolved.left_torque_cmd = msg.left_torque_cmd;
    }
    else {
      resolved.left_torque_cmd = 0.0
    }

    if (msg.right_torque_cmd !== undefined) {
      resolved.right_torque_cmd = msg.right_torque_cmd;
    }
    else {
      resolved.right_torque_cmd = 0.0
    }

    if (msg.au_state !== undefined) {
      resolved.au_state = msg.au_state;
    }
    else {
      resolved.au_state = 0
    }

    if (msg.brake_effort !== undefined) {
      resolved.brake_effort = msg.brake_effort;
    }
    else {
      resolved.brake_effort = 0
    }

    return resolved;
    }
};

module.exports = MobilityMsg;
