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
 * \file accel_calib.h
 * \author Daniel Koch <danielpkoch@gmail.com>
 *
 * Class for calculating and applying accelerometer calibration parameters
 */

#include <Eigen/Dense>

#include <string>

namespace imu_calib
{

class AccelCalib
{
public:

  enum Orientation { XPOS = 0, XNEG, YPOS, YNEG, ZPOS, ZNEG };

  AccelCalib();
  AccelCalib(std::string calib_file);

  // status
  bool calibReady();

  // file I/O
  bool loadCalib(std::string calib_file);
  bool saveCalib(std::string calib_file);

  // calibration procedure
  void beginCalib(int measurements, double reference_acceleration);
  bool addMeasurement(Orientation orientation, double ax, double ay, double az);
  bool computeCalib();

  // calibration application
  void applyCalib(double raw[3], double corrected[3]);
  void applyCalib(double raw_x, double raw_y, double raw_z, double *corr_x, double *corr_y, double *corr_z);

protected:
  static const int reference_index_[6];
  static const int reference_sign_[6];
  bool calib_ready_;

  Eigen::Matrix3d SM_; //!< combined scale and misalignment parameters
  Eigen::Vector3d bias_; //!< scaled and rotated bias parameters

  double reference_acceleration_; //!< expected acceleration measurement (e.g. 1.0 for unit of g's, 9.80665 for unit of m/s^2)

  bool calib_initialized_;
  int orientation_count_[6];

  Eigen::MatrixXd meas_; //!< least squares measurements matrix
  Eigen::VectorXd ref_; //!< least squares expected measurements vector
  int num_measurements_; //!< number of measurements expected for this calibration
  int measurements_received_; //!< number of measurements received for this calibration
};

} // namespace accel_calib
