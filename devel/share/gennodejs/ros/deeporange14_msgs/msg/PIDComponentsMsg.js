// Auto-generated. Do not edit!

// (in-package deeporange14_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PIDComponentsMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.P_Vx = null;
      this.I_Vx = null;
      this.D_Vx = null;
      this.P_Wz = null;
      this.I_Wz = null;
      this.D_Wz = null;
    }
    else {
      if (initObj.hasOwnProperty('P_Vx')) {
        this.P_Vx = initObj.P_Vx
      }
      else {
        this.P_Vx = 0.0;
      }
      if (initObj.hasOwnProperty('I_Vx')) {
        this.I_Vx = initObj.I_Vx
      }
      else {
        this.I_Vx = 0.0;
      }
      if (initObj.hasOwnProperty('D_Vx')) {
        this.D_Vx = initObj.D_Vx
      }
      else {
        this.D_Vx = 0.0;
      }
      if (initObj.hasOwnProperty('P_Wz')) {
        this.P_Wz = initObj.P_Wz
      }
      else {
        this.P_Wz = 0.0;
      }
      if (initObj.hasOwnProperty('I_Wz')) {
        this.I_Wz = initObj.I_Wz
      }
      else {
        this.I_Wz = 0.0;
      }
      if (initObj.hasOwnProperty('D_Wz')) {
        this.D_Wz = initObj.D_Wz
      }
      else {
        this.D_Wz = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PIDComponentsMsg
    // Serialize message field [P_Vx]
    bufferOffset = _serializer.float64(obj.P_Vx, buffer, bufferOffset);
    // Serialize message field [I_Vx]
    bufferOffset = _serializer.float64(obj.I_Vx, buffer, bufferOffset);
    // Serialize message field [D_Vx]
    bufferOffset = _serializer.float64(obj.D_Vx, buffer, bufferOffset);
    // Serialize message field [P_Wz]
    bufferOffset = _serializer.float64(obj.P_Wz, buffer, bufferOffset);
    // Serialize message field [I_Wz]
    bufferOffset = _serializer.float64(obj.I_Wz, buffer, bufferOffset);
    // Serialize message field [D_Wz]
    bufferOffset = _serializer.float64(obj.D_Wz, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PIDComponentsMsg
    let len;
    let data = new PIDComponentsMsg(null);
    // Deserialize message field [P_Vx]
    data.P_Vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [I_Vx]
    data.I_Vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [D_Vx]
    data.D_Vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [P_Wz]
    data.P_Wz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [I_Wz]
    data.I_Wz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [D_Wz]
    data.D_Wz = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'deeporange14_msgs/PIDComponentsMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc85b3278733449a3886e64952b6b2bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 P_Vx
    float64 I_Vx
    float64 D_Vx
    float64 P_Wz
    float64 I_Wz
    float64 D_Wz
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PIDComponentsMsg(null);
    if (msg.P_Vx !== undefined) {
      resolved.P_Vx = msg.P_Vx;
    }
    else {
      resolved.P_Vx = 0.0
    }

    if (msg.I_Vx !== undefined) {
      resolved.I_Vx = msg.I_Vx;
    }
    else {
      resolved.I_Vx = 0.0
    }

    if (msg.D_Vx !== undefined) {
      resolved.D_Vx = msg.D_Vx;
    }
    else {
      resolved.D_Vx = 0.0
    }

    if (msg.P_Wz !== undefined) {
      resolved.P_Wz = msg.P_Wz;
    }
    else {
      resolved.P_Wz = 0.0
    }

    if (msg.I_Wz !== undefined) {
      resolved.I_Wz = msg.I_Wz;
    }
    else {
      resolved.I_Wz = 0.0
    }

    if (msg.D_Wz !== undefined) {
      resolved.D_Wz = msg.D_Wz;
    }
    else {
      resolved.D_Wz = 0.0
    }

    return resolved;
    }
};

module.exports = PIDComponentsMsg;
