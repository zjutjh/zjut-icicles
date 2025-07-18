// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughCircle_Demo.cpp
/**
 * @file HoughCircle_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/HoughCirclesConfig.h"
#include "opencv_apps/Circle.h"
#include "opencv_apps/CircleArray.h"
#include "opencv_apps/CircleArrayStamped.h"

namespace opencv_apps
{
class HoughCirclesNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::HoughCirclesConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  // initial and max values of the parameters of interests.
  int canny_threshold_initial_value_;
  int accumulator_threshold_initial_value_;
  int max_accumulator_threshold_;
  int max_canny_threshold_;
  double canny_threshold_;
  int canny_threshold_int;  // for trackbar
  double accumulator_threshold_;
  int accumulator_threshold_int;
  int gaussian_blur_size_;
  double gaussian_sigma_x_;
  int gaussian_sigma_x_int;
  double gaussian_sigma_y_;
  int gaussian_sigma_y_int;
  int voting_threshold_;
  double min_distance_between_circles_;
  int min_distance_between_circles_int;
  double dp_;
  int dp_int;
  int min_circle_radius_;
  int max_circle_radius_;

  image_transport::Publisher debug_image_pub_;
  int debug_image_type_;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    canny_threshold_ = config_.canny_threshold;
    accumulator_threshold_ = config_.accumulator_threshold;
    gaussian_blur_size_ = config_.gaussian_blur_size;
    gaussian_sigma_x_ = config_.gaussian_sigma_x;
    gaussian_sigma_y_ = config_.gaussian_sigma_y;

    dp_ = config_.dp;
    min_circle_radius_ = config_.min_circle_radius;
    max_circle_radius_ = config_.max_circle_radius;
    debug_image_type_ = config_.debug_image_type;
    min_distance_between_circles_ = config_.min_distance_between_circles;
    canny_threshold_int = int(canny_threshold_);
    accumulator_threshold_int = int(accumulator_threshold_);
    gaussian_sigma_x_int = int(gaussian_sigma_x_);
    gaussian_sigma_y_int = int(gaussian_sigma_y_);
    min_distance_between_circles_int = int(min_distance_between_circles_);
    dp_int = int(dp_);
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

  static void trackbarCallback(int value, void* userdata)
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
      opencv_apps::CircleArrayStamped circles_msg;
      circles_msg.header = msg->header;

      // Do the work
      std::vector<cv::Rect> faces;
      cv::Mat src_gray, edges;

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        src_gray = frame;
      }

      // create the main window, and attach the trackbars
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

        cv::createTrackbar("Canny Threshold", window_name_, &canny_threshold_int, max_canny_threshold_,
                           trackbarCallback);
        cv::createTrackbar("Accumulator Threshold", window_name_, &accumulator_threshold_int,
                           max_accumulator_threshold_, trackbarCallback);
        cv::createTrackbar("Gaussian Blur Size", window_name_, &gaussian_blur_size_, 30, trackbarCallback);
        cv::createTrackbar("Gaussian Sigam X", window_name_, &gaussian_sigma_x_int, 10, trackbarCallback);
        cv::createTrackbar("Gaussian Sigma Y", window_name_, &gaussian_sigma_y_int, 10, trackbarCallback);
        cv::createTrackbar("Min Distance between Circles", window_name_, &min_distance_between_circles_int, 100,
                           trackbarCallback);
        cv::createTrackbar("Dp", window_name_, &dp_int, 100, trackbarCallback);
        cv::createTrackbar("Min Circle Radius", window_name_, &min_circle_radius_, 500, trackbarCallback);
        cv::createTrackbar("Max Circle Radius", window_name_, &max_circle_radius_, 2000, trackbarCallback);

        if (need_config_update_)
        {
          config_.canny_threshold = canny_threshold_ = (double)canny_threshold_int;
          config_.accumulator_threshold = accumulator_threshold_ = (double)accumulator_threshold_int;
          config_.gaussian_blur_size = gaussian_blur_size_;
          config_.gaussian_sigma_x = gaussian_sigma_x_ = (double)gaussian_sigma_x_int;
          config_.gaussian_sigma_y = gaussian_sigma_y_ = (double)gaussian_sigma_y_int;
          config_.min_distance_between_circles = min_distance_between_circles_ =
              (double)min_distance_between_circles_int;
          config_.dp = dp_ = (double)dp_int;
          config_.min_circle_radius = min_circle_radius_;
          config_.max_circle_radius = max_circle_radius_;
          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
      }

      // Reduce the noise so we avoid false circle detection
      // gaussian_blur_size_ must be odd number
      if (gaussian_blur_size_ % 2 != 1)
      {
        gaussian_blur_size_ = gaussian_blur_size_ + 1;
      }
      cv::GaussianBlur(src_gray, src_gray, cv::Size(gaussian_blur_size_, gaussian_blur_size_), gaussian_sigma_x_,
                       gaussian_sigma_y_);

      // those paramaters cannot be =0
      // so we must check here
      canny_threshold_ = std::max(canny_threshold_, 1.0);
      accumulator_threshold_ = std::max(accumulator_threshold_, 1.0);

      if (debug_view_)
      {
        // https://github.com/Itseez/opencv/blob/2.4.8/modules/imgproc/src/hough.cpp#L817
        cv::Canny(frame, edges, MAX(canny_threshold_ / 2, 1), canny_threshold_, 3);
      }
      if (min_distance_between_circles_ == 0)
      {  // set inital value
        min_distance_between_circles_ = src_gray.rows / 8;
        config_.min_distance_between_circles = min_distance_between_circles_;
        reconfigure_server_->updateConfig(config_);
      }
      // runs the detection, and update the display
      // will hold the results of the detection
      std::vector<cv::Vec3f> circles;
      // runs the actual detection
      cv::HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, dp_, min_distance_between_circles_, canny_threshold_,
                       accumulator_threshold_, min_circle_radius_, max_circle_radius_);

      cv::Mat out_image;
      if (frame.channels() == 1)
      {
        cv::cvtColor(frame, out_image, cv::COLOR_GRAY2BGR);
      }
      else
      {
        out_image = frame;
      }

      // clone the colour, input image for displaying purposes
      for (const cv::Vec3f& i : circles)
      {
        cv::Point center(cvRound(i[0]), cvRound(i[1]));
        int radius = cvRound(i[2]);
        // circle center
        circle(out_image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        circle(out_image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

        opencv_apps::Circle circle_msg;
        circle_msg.center.x = center.x;
        circle_msg.center.y = center.y;
        circle_msg.radius = radius;
        circles_msg.circles.push_back(circle_msg);
      }

      // shows the results
      if (debug_view_ || debug_image_pub_.getNumSubscribers() > 0)
      {
        cv::Mat debug_image;
        switch (debug_image_type_)
        {
          case 1:
            debug_image = src_gray;
            break;
          case 2:
            debug_image = edges;
            break;
          default:
            debug_image = frame;
            break;
        }
        if (debug_view_)
        {
          cv::imshow(window_name_, debug_image);
          int c = cv::waitKey(1);
          if (c == 's')
          {
            debug_image_type_ = (++debug_image_type_) % 3;
            config_.debug_image_type = debug_image_type_;
            reconfigure_server_->updateConfig(config_);
          }
        }
        if (debug_image_pub_.getNumSubscribers() > 0)
        {
          sensor_msgs::Image::Ptr out_debug_img =
              cv_bridge::CvImage(msg->header, msg->encoding, debug_image).toImageMsg();
          debug_image_pub_.publish(out_debug_img);
        }
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(circles_msg);
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
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &HoughCirclesNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &HoughCirclesNodelet::imageCallback, this);
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

    debug_image_type_ = 0;
    pnh_->param("queue_size", queue_size_, 3);
    pnh_->param("debug_view", debug_view_, false);
    if (debug_view_)
    {
      always_subscribe_ = debug_view_;
    }
    prev_stamp_ = ros::Time(0, 0);

    window_name_ = "Hough Circle Detection Demo";
    canny_threshold_initial_value_ = 200;
    accumulator_threshold_initial_value_ = 50;
    max_accumulator_threshold_ = 200;
    max_canny_threshold_ = 255;
    min_distance_between_circles_ = 0;

    // declare and initialize both parameters that are subjects to change
    canny_threshold_ = canny_threshold_initial_value_;
    accumulator_threshold_ = accumulator_threshold_initial_value_;

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&HoughCirclesNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::CircleArrayStamped>(*pnh_, "circles", 1);

    debug_image_type_ = 0;
    debug_image_pub_ = advertiseImage(*pnh_, "debug_image", 1);

    onInitPostProcess();
  }
};
bool HoughCirclesNodelet::need_config_update_ = false;
}  // namespace opencv_apps

namespace hough_circles
{
class HoughCirclesNodelet : public opencv_apps::HoughCirclesNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet hough_circles/hough_circles is deprecated, "
             "and renamed to opencv_apps/hough_circles.");
    opencv_apps::HoughCirclesNodelet::onInit();
  }
};
}  // namespace hough_circles

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::HoughCirclesNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(hough_circles::HoughCirclesNodelet, nodelet::Nodelet);
