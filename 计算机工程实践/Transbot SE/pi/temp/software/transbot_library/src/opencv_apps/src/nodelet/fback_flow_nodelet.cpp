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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/fback.cpp
/*
 * This program demonstrates dense optical flow algorithm by Gunnar Farneback
 * Mainly the function: calcOpticalFlowFarneback()
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/FBackFlowConfig.h"
#include "opencv_apps/FlowArrayStamped.h"

namespace opencv_apps
{
class FBackFlowNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::FBackFlowConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  cv::Mat prevgray, gray, flow, cflow;

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
      opencv_apps::FlowArrayStamped flows_msg;
      flows_msg.header = msg->header;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        if (need_config_update_)
        {
          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
      }

      // Check if clock is jumped back
      if (ros::Time::isSimTime() && prev_stamp_ > msg->header.stamp)
      {
        NODELET_WARN_STREAM("Detected jump back in time of " << msg->header.stamp << ". Clearing optical flow cache.");
        prevgray = cv::Mat();
      }

      // Do the work
      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        frame.copyTo(gray);
      }
      if (prevgray.data)
      {
        cv::calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        cv::cvtColor(prevgray, cflow, cv::COLOR_GRAY2BGR);
        // drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));
        int step = 16;
        cv::Scalar color = cv::Scalar(0, 255, 0);
        for (int y = 0; y < cflow.rows; y += step)
          for (int x = 0; x < cflow.cols; x += step)
          {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            cv::line(cflow, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
            cv::circle(cflow, cv::Point(x, y), 2, color, -1);

            opencv_apps::Flow flow_msg;
            opencv_apps::Point2D point_msg;
            opencv_apps::Point2D velocity_msg;
            point_msg.x = x;
            point_msg.y = y;
            velocity_msg.x = fxy.x;
            velocity_msg.y = fxy.y;
            flow_msg.point = point_msg;
            flow_msg.velocity = velocity_msg;
            flows_msg.flow.push_back(flow_msg);
          }
      }

      std::swap(prevgray, gray);

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, cflow);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", cflow).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(flows_msg);
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
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &FBackFlowNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &FBackFlowNodelet::imageCallback, this);
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

    window_name_ = "flow";

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&FBackFlowNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::FlowArrayStamped>(*pnh_, "flows", 1);

    onInitPostProcess();
  }
};
bool FBackFlowNodelet::need_config_update_ = false;
}  // namespace opencv_apps

namespace fback_flow
{
class FBackFlowNodelet : public opencv_apps::FBackFlowNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet fback_flow/fback_flow is deprecated, "
             "and renamed to opencv_apps/fback_flow.");
    opencv_apps::FBackFlowNodelet::onInit();
  }
};
}  // namespace fback_flow

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::FBackFlowNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(fback_flow::FBackFlowNodelet, nodelet::Nodelet);
