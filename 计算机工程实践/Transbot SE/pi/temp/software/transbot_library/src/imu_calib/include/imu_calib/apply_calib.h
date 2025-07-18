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
 * \file apply_calib.h
 * \author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Class for applying a previously computed calibration to IMU data
 */

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <transbot_msgs/Buzzer.h>
#include <imu_calib/accel_calib.h>
#include <string>
namespace imu_calib
{

class ApplyCalib
{
public:
  ApplyCalib();

private:
  AccelCalib calib_;

  ros::Subscriber raw_imu_sub_;
  ros::Subscriber raw_mag_sub_;

  ros::Publisher corrected_pub_;
  ros::Publisher mag_pub_;
  ros::ServiceClient client_Buzzer;
  void rawImuCallback(sensor_msgs::Imu::ConstPtr raw);
  void rawMagCallback(sensor_msgs::MagneticField::ConstPtr raw);

  bool calibrate_gyros_;
  bool is_multi_;
  int gyro_calib_samples_;
  int gyro_sample_count_;

  double gyro_bias_x_;
  double gyro_bias_y_;
  double gyro_bias_z_;
  std::string namespace_;
};

} // namespace accel_calib
