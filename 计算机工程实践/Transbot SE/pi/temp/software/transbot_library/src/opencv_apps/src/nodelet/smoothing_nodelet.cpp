// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
 *   * Neither the name of the JSK Lab nor the names of its
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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/ImgProc/Smoothing.cpp
/**
 * file Smoothing.cpp
 * brief Sample code for simple filters
 * author OpenCV team
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <vector>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/SmoothingConfig.h"

/// Global Variables
int MAX_KERNEL_LENGTH = 31;

namespace opencv_apps
{
class SmoothingNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::SmoothingConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  int kernel_size_;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    kernel_size_ = (config_.kernel_size / 2) * 2 + 1;  // kernel_size must be odd number
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

  void doWork(const sensor_msgs::ImageConstPtr& msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      if (debug_view_)
      {
        /// Create Trackbars for Thresholds
        char kernel_label[] = "Kernel Size : ";

        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::createTrackbar(kernel_label, window_name_, &kernel_size_, MAX_KERNEL_LENGTH, trackbarCallback);
        if (need_config_update_)
        {
          kernel_size_ = (kernel_size_ / 2) * 2 + 1;  // kernel_size must be odd number
          config_.kernel_size = kernel_size_;
          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
      }

      cv::Mat out_image = in_image.clone();
      int i = kernel_size_;
      switch (config_.filter_type)
      {
        case opencv_apps::Smoothing_Homogeneous_Blur:
        {
          /// Applying Homogeneous blur
          ROS_DEBUG_STREAM("Applying Homogeneous blur with kernel size " << i);
          cv::blur(in_image, out_image, cv::Size(i, i), cv::Point(-1, -1));
          break;
        }
        case opencv_apps::Smoothing_Gaussian_Blur:
        {
          /// Applying Gaussian blur
          ROS_DEBUG_STREAM("Applying Gaussian blur with kernel size " << i);
          cv::GaussianBlur(in_image, out_image, cv::Size(i, i), 0, 0);
          break;
        }
        case opencv_apps::Smoothing_Median_Blur:
        {
          /// Applying Median blur
          ROS_DEBUG_STREAM("Applying Median blur with kernel size " << i);
          cv::medianBlur(in_image, out_image, i);
          break;
        }
        case opencv_apps::Smoothing_Bilateral_Filter:
        {
          /// Applying Bilateral Filter
          ROS_DEBUG_STREAM("Applying Bilateral blur with kernel size " << i);
          cv::bilateralFilter(in_image, out_image, i, i * 2, i / 2);
          break;
        }
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, out_image);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg();
      img_pub_.publish(out_img);
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &SmoothingNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &SmoothingNodelet::imageCallback, this);
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
    prev_stamp_ = ros::Time(0, 0);

    window_name_ = "Smoothing Demo";
    kernel_size_ = 7;

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&SmoothingNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);

    onInitPostProcess();
  }
};
bool SmoothingNodelet::need_config_update_ = false;
}  // namespace opencv_apps

namespace smoothing
{
class SmoothingNodelet : public opencv_apps::SmoothingNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet smoothing/smoothing is deprecated, "
             "and renamed to opencv_apps/smoothing.");
    opencv_apps::SmoothingNodelet::onInit();
  }
};
}  // namespace smoothing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::SmoothingNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(smoothing::SmoothingNodelet, nodelet::Nodelet);
