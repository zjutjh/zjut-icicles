/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
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

// https://github.com/Itseez/opencv/blob/master/samples/cpp/phase_corr.cpp

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/PhaseCorrConfig.h"
#include "opencv_apps/Point2DStamped.h"

namespace opencv_apps
{
class PhaseCorrNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::PhaseCorrConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  cv::Mat curr, prev, curr64f, prev64f, hann;

  std::string window_name_;
  static bool need_config_update_;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
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
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::Point2DStamped shift_msg;
      shift_msg.header = msg->header;

      // Do the work
      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, curr, cv::COLOR_BGR2GRAY);
      }
      else
      {
        curr = frame;
      }

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        if (need_config_update_)
        {
          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
      }

      if (prev.empty())
      {
        prev = curr.clone();
        cv::createHanningWindow(hann, curr.size(), CV_64F);
      }

      prev.convertTo(prev64f, CV_64F);
      curr.convertTo(curr64f, CV_64F);

      cv::Point2d shift = cv::phaseCorrelate(prev64f, curr64f, hann);
      double radius = cv::sqrt(shift.x * shift.x + shift.y * shift.y);

      if (radius > 0)
      {
        // draw a circle and line indicating the shift direction...
        cv::Point center(curr.cols >> 1, curr.rows >> 1);
#ifndef CV_VERSION_EPOCH
        cv::circle(frame, center, (int)(radius * 5), cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        cv::line(frame, center, cv::Point(center.x + (int)(shift.x * 5), center.y + (int)(shift.y * 5)),
                 cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
#else
        cv::circle(frame, center, (int)(radius * 5), cv::Scalar(0, 255, 0), 3, CV_AA);
        cv::line(frame, center, cv::Point(center.x + (int)(shift.x * 5), center.y + (int)(shift.y * 5)),
                 cv::Scalar(0, 255, 0), 3, CV_AA);
#endif

        //
        shift_msg.point.x = shift.x;
        shift_msg.point.y = shift.y;
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        int c = cv::waitKey(1);
      }

      prev = curr.clone();

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(shift_msg);
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
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &PhaseCorrNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &PhaseCorrNodelet::imageCallback, this);
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

    window_name_ = "phase shift";

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&PhaseCorrNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::Point2DStamped>(*pnh_, "shift", 1);

    onInitPostProcess();
  }
};
bool PhaseCorrNodelet::need_config_update_ = false;
}  // namespace opencv_apps

namespace phase_corr
{
class PhaseCorrNodelet : public opencv_apps::PhaseCorrNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet phase_corr/phase_corr is deprecated, "
             "and renamed to opencv_apps/phase_corr.");
    opencv_apps::PhaseCorrNodelet::onInit();
  }
};
}  // namespace phase_corr

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::PhaseCorrNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(phase_corr::PhaseCorrNodelet, nodelet::Nodelet);
