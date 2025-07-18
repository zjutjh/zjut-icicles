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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/simpleflow_demo.cpp
/**
 * This is a demo of SimpleFlow optical flow algorithm
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#if CV_MAJOR_VERSION >= 3
#include <opencv2/optflow.hpp>
#endif

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/SimpleFlowConfig.h"
#include "opencv_apps/FlowArrayStamped.h"

namespace opencv_apps
{
class SimpleFlowNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::SimpleFlowConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  int subscriber_count_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;
  int scale_;

  cv::Mat color, prevColor;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    scale_ = config_.scale;
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
      cv::Mat frame_src = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      /// Convert it to 3 channel
      cv::Mat frame;
      if (frame_src.channels() > 1)
      {
        frame = frame_src;
      }
      else
      {
        cv::cvtColor(frame_src, frame, cv::COLOR_GRAY2BGR);
      }

      cv::resize(frame, color, cv::Size(frame.cols / (double)MAX(1, scale_), frame.rows / (double)MAX(1, scale_)), 0, 0,
                 CV_INTER_AREA);
      if (prevColor.empty())
        color.copyTo(prevColor);

      if (color.rows != prevColor.rows && color.cols != prevColor.cols)
      {
        NODELET_WARN("Images should be of equal sizes");
        color.copyTo(prevColor);
      }

      if (frame.type() != CV_8UC3)
      {
        NODELET_ERROR("Images should be of equal type CV_8UC3");
      }
      // Messages
      opencv_apps::FlowArrayStamped flows_msg;
      flows_msg.header = msg->header;

      // Do the work
      cv::Mat flow;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("Scale", window_name_, &scale_, 24, trackbarCallback);
        if (need_config_update_)
        {
          config_.scale = scale_;
          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
      }

      float start = (float)cv::getTickCount();
#if CV_MAJOR_VERSION >= 3
      cv::optflow::calcOpticalFlowSF(color, prevColor,
#else
      cv::calcOpticalFlowSF(color, prevColor,
#endif
                                     flow, 3, 2, 4, 4.1, 25.5, 18, 55.0, 25.5, 0.35, 18, 55.0, 25.5, 10);
      NODELET_INFO("calcOpticalFlowSF : %lf sec", (cv::getTickCount() - start) / cv::getTickFrequency());

      // writeOpticalFlowToFile(flow, file);
      int cols = flow.cols;
      int rows = flow.rows;
      double scale_col = frame.cols / (double)flow.cols;
      double scale_row = frame.rows / (double)flow.rows;

      for (int i = 0; i < rows; ++i)
      {
        for (int j = 0; j < cols; ++j)
        {
          cv::Vec2f flow_at_point = flow.at<cv::Vec2f>(i, j);
          cv::line(frame, cv::Point(scale_col * j, scale_row * i),
                   cv::Point(scale_col * (j + flow_at_point[0]), scale_row * (i + flow_at_point[1])),
                   cv::Scalar(0, 255, 0), 1, 8, 0);

          opencv_apps::Flow flow_msg;
          opencv_apps::Point2D point_msg;
          opencv_apps::Point2D velocity_msg;
          point_msg.x = scale_col * j;
          point_msg.y = scale_row * i;
          velocity_msg.x = scale_col * flow_at_point[0];
          velocity_msg.y = scale_row * flow_at_point[1];
          flow_msg.point = point_msg;
          flow_msg.velocity = velocity_msg;
          flows_msg.flow.push_back(flow_msg);
        }
      }

      // cv::cvtColor( frame, src_gray, cv::COLOR_BGR2GRAY );
      /// Apply Canny edge detector
      // Canny( src_gray, edges, 50, 200, 3 );

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        int c = cv::waitKey(1);
      }

      cv::swap(prevColor, color);
      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
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
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &SimpleFlowNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &SimpleFlowNodelet::imageCallback, this);
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

    window_name_ = "simpleflow_demo";
    scale_ = 4.0;

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&SimpleFlowNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::FlowArrayStamped>(*pnh_, "flows", 1);

    onInitPostProcess();
  }
};
bool SimpleFlowNodelet::need_config_update_ = false;
}  // namespace opencv_apps

namespace simple_flow
{
class SimpleFlowNodelet : public opencv_apps::SimpleFlowNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet simple_flow/simple_flow is deprecated, "
             "and renamed to opencv_apps/simple_flow.");
    opencv_apps::SimpleFlowNodelet::onInit();
  }
};
}  // namespace simple_flow

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::SimpleFlowNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(simple_flow::SimpleFlowNodelet, nodelet::Nodelet);
