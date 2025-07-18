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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ShapeDescriptors/hull_demo.cpp
/**
 * @function hull_demo.cpp
 * @brief Demo code to find contours in an image
 * @author OpenCV team
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/ConvexHullConfig.h"
#include "opencv_apps/Contour.h"
#include "opencv_apps/ContourArray.h"
#include "opencv_apps/ContourArrayStamped.h"

namespace opencv_apps
{
class ConvexHullNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::ConvexHullConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  int threshold_;

  std::string window_name_;
  static bool need_config_update_;

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

  void doWork(const sensor_msgs::ImageConstPtr& msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::ContourArrayStamped contours_msg;
      contours_msg.header = msg->header;

      // Do the work
      cv::Mat src_gray;

      /// Convert image to gray and blur it
      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_RGB2GRAY);
      }
      else
      {
        src_gray = frame;
      }
      cv::blur(src_gray, src_gray, cv::Size(3, 3));

      /// Create window
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      cv::Mat threshold_output;
      int max_thresh = 255;
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::RNG rng(12345);

      /// Detect edges using Threshold
      cv::threshold(src_gray, threshold_output, threshold_, 255, cv::THRESH_BINARY);

      /// Find contours
      cv::findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      /// Find the convex hull object for each contour
      std::vector<std::vector<cv::Point> > hull(contours.size());
      for (size_t i = 0; i < contours.size(); i++)
      {
        cv::convexHull(cv::Mat(contours[i]), hull[i], false);
      }

      /// Draw contours + hull results
      cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
      for (size_t i = 0; i < contours.size(); i++)
      {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::drawContours(drawing, contours, (int)i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        cv::drawContours(drawing, hull, (int)i, color, 4, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

        opencv_apps::Contour contour_msg;
        for (const cv::Point& j : hull[i])
        {
          opencv_apps::Point2D point_msg;
          point_msg.x = j.x;
          point_msg.y = j.y;
          contour_msg.points.push_back(point_msg);
        }
        contours_msg.contours.push_back(contour_msg);
      }

      /// Create a Trackbar for user to enter threshold
      if (debug_view_)
      {
        if (need_config_update_)
        {
          config_.threshold = threshold_;
          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
        cv::createTrackbar("Threshold:", window_name_, &threshold_, max_thresh, trackbarCallback);

        cv::imshow(window_name_, drawing);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, drawing).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(contours_msg);
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
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &ConvexHullNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &ConvexHullNodelet::imageCallback, this);
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

    window_name_ = "Hull Demo";
    threshold_ = 100;

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&ConvexHullNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::ContourArrayStamped>(*pnh_, "hulls", 1);
    onInitPostProcess();
  }
};
bool ConvexHullNodelet::need_config_update_ = false;
}  // namespace opencv_apps

namespace convex_hull
{
class ConvexHullNodelet : public opencv_apps::ConvexHullNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet convex_hull/convex_hull is deprecated, "
             "and renamed to opencv_apps/convex_hull.");
    opencv_apps::ConvexHullNodelet::onInit();
  }
};
}  // namespace convex_hull

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::ConvexHullNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(convex_hull::ConvexHullNodelet, nodelet::Nodelet);
