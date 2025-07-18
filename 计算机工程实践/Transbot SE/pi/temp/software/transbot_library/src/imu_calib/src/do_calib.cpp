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
 * \file do_calib.cpp
 * \author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Class for performing IMU calibration
 */

#include "imu_calib/do_calib.h"

namespace imu_calib
{

DoCalib::DoCalib() :
  state_(START)
{
  ros::NodeHandle nh;
  imu_sub_ = nh.subscribe("/transbot/imu", 1, &DoCalib::imuCallback, this);

  ros::NodeHandle nh_private("~");
  nh_private.param<int>("measurements", measurements_per_orientation_, 500);
  nh_private.param<double>("reference_acceleration", reference_acceleration_, 9.80665);
  nh_private.param<std::string>("output_file", output_file_, "imu_calib.yaml");

  orientations_.push(AccelCalib::XPOS);
  orientations_.push(AccelCalib::XNEG);
  orientations_.push(AccelCalib::YPOS);
  orientations_.push(AccelCalib::YNEG);
  orientations_.push(AccelCalib::ZPOS);
  orientations_.push(AccelCalib::ZNEG);

  orientation_labels_[AccelCalib::XPOS] = "X+ axis - Front side of the robot";
  orientation_labels_[AccelCalib::XNEG] = "X- axis - Rear side of the robot" ;
  orientation_labels_[AccelCalib::YPOS] = "Y+ axis - Right side of the robot";
  orientation_labels_[AccelCalib::YNEG] = "Y- axis - Left side of the robot";
  orientation_labels_[AccelCalib::ZPOS] = "Z+ axis - Top side of the robot";
  orientation_labels_[AccelCalib::ZNEG] = "Z- axis - Bottom side of the robot";
}

bool DoCalib::running()
{
  return state_ != DONE;
}

void DoCalib::imuCallback(sensor_msgs::Imu::ConstPtr imu)
{
  bool accepted;

  switch (state_)
  {
  case START:
    calib_.beginCalib(6*measurements_per_orientation_, reference_acceleration_);
    state_ = SWITCHING;
    break;


  case SWITCHING:
    if (orientations_.empty())
    {
      state_ = COMPUTING;
    }
    else
    {
      current_orientation_ = orientations_.front();

      orientations_.pop();
      measurements_received_ = 0;

      std::cout << "Orient IMU with " << orientation_labels_[current_orientation_] << " facing up. Press [ENTER] once done.";
      std::cin.ignore();
      std::cout << "Calibrating! This may take a while...." << std::endl;

      state_ = RECEIVING;
    }
    break;


  case RECEIVING:
    accepted = calib_.addMeasurement(current_orientation_,
                                     imu->linear_acceleration.x,
                                     imu->linear_acceleration.y,
                                     imu->linear_acceleration.z);

    measurements_received_ += accepted ? 1 : 0;
    if (measurements_received_ >= measurements_per_orientation_)
    {
      std::cout << " Done." << std::endl;
      state_ = SWITCHING;
    }
    break;


  case COMPUTING:
    std::cout << "Computing calibration parameters...";
    if (calib_.computeCalib())
    {
      std::cout << " Success!"  << std::endl;

      std::cout << "Saving calibration file...";
      if (calib_.saveCalib(output_file_))
      {
        std::cout << " Success!" << std::endl;
      }
      else
      {
        std::cout << " Failed." << std::endl;
      }
    }
    else
    {
      std::cout << " Failed.";
      ROS_ERROR("Calibration failed");
      break;
    }
    state_ = DONE;
    break;


  case DONE:
    break;
  }
}

} // namespace accel_calib
