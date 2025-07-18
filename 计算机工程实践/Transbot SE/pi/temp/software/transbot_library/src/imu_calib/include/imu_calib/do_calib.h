/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Daniel Koch
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**
 * \file do_calib.h
 * \author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Class for performing IMU calibration
 */

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include <string>
#include <vector>
#include <queue>

#include <imu_calib/accel_calib.h>

namespace imu_calib
{

class DoCalib
{
public:
  DoCalib();

  bool running();

private:
  enum DoCalibState { START, SWITCHING, RECEIVING, COMPUTING, DONE };

  AccelCalib calib_;

  DoCalibState state_;

  int measurements_per_orientation_;
  int measurements_received_;

  double reference_acceleration_;
  std::string output_file_;

  std::queue<AccelCalib::Orientation> orientations_;
  AccelCalib::Orientation current_orientation_;

  std::string orientation_labels_[6];

  ros::Subscriber imu_sub_;
  void imuCallback(sensor_msgs::Imu::ConstPtr imu);
};

} // namespace accel_calib
