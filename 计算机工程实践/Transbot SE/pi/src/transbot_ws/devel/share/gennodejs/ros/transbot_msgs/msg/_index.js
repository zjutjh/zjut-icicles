
"use strict";

let Edition = require('./Edition.js');
let Arm = require('./Arm.js');
let Image_Msg = require('./Image_Msg.js');
let JoyState = require('./JoyState.js');
let PointArray = require('./PointArray.js');
let PatrolWarning = require('./PatrolWarning.js');
let SensorState = require('./SensorState.js');
let Battery = require('./Battery.js');
let Position = require('./Position.js');
let Joint = require('./Joint.js');
let General = require('./General.js');
let LaserAvoid = require('./LaserAvoid.js');
let Adjust = require('./Adjust.js');
let PWMServo = require('./PWMServo.js');

module.exports = {
  Edition: Edition,
  Arm: Arm,
  Image_Msg: Image_Msg,
  JoyState: JoyState,
  PointArray: PointArray,
  PatrolWarning: PatrolWarning,
  SensorState: SensorState,
  Battery: Battery,
  Position: Position,
  Joint: Joint,
  General: General,
  LaserAvoid: LaserAvoid,
  Adjust: Adjust,
  PWMServo: PWMServo,
};
