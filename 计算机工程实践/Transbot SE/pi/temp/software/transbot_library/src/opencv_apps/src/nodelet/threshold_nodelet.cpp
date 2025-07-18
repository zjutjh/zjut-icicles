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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/ImgProc/Threshold.cpp
/**
 * This is a demo of threshold image processing,
 */

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv_apps/nodelet.h"
#include "opencv_apps/ThresholdConfig.h"

#include <dynamic_reconfigure/server.h>

namespace opencv_apps
{
class ThresholdNodelet : public opencv_apps::Nodelet
{
  ////////////////////////////////////////////////////////
  // Dynamic Reconfigure
  ////////////////////////////////////////////////////////
  typedef opencv_apps::ThresholdConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_;

  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  boost::mutex mutex_;
  int threshold_type_;
  int max_binary_value_;
  int threshold_value_;
  bool apply_otsu_;

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void subscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &ThresholdNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &ThresholdNodelet::imageCallback, this);
  }

  void unsubscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  void reconfigureCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    threshold_value_ = config.threshold;
    threshold_type_ = config.threshold_type;
    max_binary_value_ = config.max_binary;
    apply_otsu_ = config.apply_otsu;
  }

  void doWork(const sensor_msgs::Image::ConstPtr& image_msg, const std::string& input_frame_from_msg)
  {
    try
    {
      cv::Mat src_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::Mat gray_image;
      cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);
      cv::Mat result_image;

      if (apply_otsu_)
      {
        threshold_type_ |= CV_THRESH_OTSU;
      }
      cv::threshold(gray_image, result_image, threshold_value_, max_binary_value_, threshold_type_);
      //-- Show what you got
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::imshow(window_name_, result_image);
        int c = cv::waitKey(1);
      }
      img_pub_.publish(
          cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::MONO8, result_image).toImageMsg());
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
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
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&ThresholdNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    onInitPostProcess();
  }
};
}  // namespace opencv_apps

namespace threshold
{
class ThresholdNodelet : public opencv_apps::ThresholdNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet threshold/threshold is deprecated, "
             "and renamed to opencv_apps/threshold.");
    opencv_apps::ThresholdNodelet::onInit();
  }
};
}  // namespace threshold

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::ThresholdNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(threshold::ThresholdNodelet, nodelet::Nodelet);
