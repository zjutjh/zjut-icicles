// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, JSK Lab.
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
*   * Neither the name of the Kei Okada nor the names of its
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
#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/CornerHarrisConfig.h"

// https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/TrackingMotion/cornerHarris_Demo.cpp
/**
 * @function cornerHarris_Demo.cpp
 * @brief Demo code for detecting corners using Harris-Stephens method
 * @author OpenCV team
 */

namespace opencv_apps
{
class CornerHarrisNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::CornerHarrisConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_;
  static bool need_config_update_;

  int threshold_;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    threshold_ = config_.threshold;
  }

  const std::string& frameWithDefault(const std::string& frame, const std::string& image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  static void trackbarCallback(int /*unused*/, void* /*unused*/)
  {
    need_config_update_ = true;
  }

  void doWork(const sensor_msgs::ImageConstPtr& image_msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;

      // Do the work
      cv::Mat dst, dst_norm, dst_norm_scaled;
      dst = cv::Mat::zeros(frame.size(), CV_32FC1);

      cv::Mat src_gray;

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        src_gray = frame;
        cv::cvtColor(src_gray, frame, cv::COLOR_GRAY2BGR);
      }

      /// Detector parameters
      int block_size = 2;
      int aperture_size = 3;
      double k = 0.04;

      /// Detecting corners
      cv::cornerHarris(src_gray, dst, block_size, aperture_size, k, cv::BORDER_DEFAULT);

      /// Normalizing
      cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
      cv::convertScaleAbs(dst_norm, dst_norm_scaled);

      /// Drawing a circle around corners
      for (int j = 0; j < dst_norm.rows; j++)
      {
        for (int i = 0; i < dst_norm.cols; i++)
        {
          if ((int)dst_norm.at<float>(j, i) > threshold_)
          {
            cv::circle(frame, cv::Point(i, j), 5, cv::Scalar(0), 2, 8, 0);
          }
        }
      }

      /// Create window
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        const int max_threshold = 255;
        if (need_config_update_)
        {
          config_.threshold = threshold_;
          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
        cv::createTrackbar("Threshold:", window_name_, &threshold_, max_threshold, trackbarCallback);
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img =
          cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      img_pub_.publish(out_img);
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  }

  void subscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &CornerHarrisNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &CornerHarrisNodelet::imageCallback, this);
  }

  void unsubscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("queue_size", queue_size_, 3);
    pnh_->param("debug_view", debug_view_, false);

    if (debug_view_)
    {
      always_subscribe_ = true;
    }

    window_name_ = "CornerHarris Demo";

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&CornerHarrisNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);

    onInitPostProcess();
  }
};
bool CornerHarrisNodelet::need_config_update_ = false;
}  // namespace opencv_apps

namespace corner_harris
{
class CornerHarrisNodelet : public opencv_apps::CornerHarrisNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet corner_harris/corner_harris is deprecated, "
             "and renamed to opencv_apps/corner_harris.");
    opencv_apps::CornerHarrisNodelet::onInit();
  }
};
}  // namespace corner_harris

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::CornerHarrisNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(corner_harris::CornerHarrisNodelet, nodelet::Nodelet);
