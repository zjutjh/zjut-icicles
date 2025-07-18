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

// http://github.com/Itseez/opencv/blob/master/samples/cpp/tutorial_code/TrackingMotion/goodFeaturesToTrack_Demo.cpp
/**
 * @function goodFeaturesToTrack_Demo.cpp
 * @brief Demo code for detecting corners using Shi-Tomasi method
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
#include "opencv_apps/GoodfeatureTrackConfig.h"
#include "opencv_apps/Point2D.h"
#include "opencv_apps/Point2DArrayStamped.h"

namespace opencv_apps
{
class GoodfeatureTrackNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::GoodfeatureTrackConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  int max_corners_;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    max_corners_ = config_.max_corners;
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
      opencv_apps::Point2DArrayStamped corners_msg;
      corners_msg.header = msg->header;

      // Do the work
      cv::Mat src_gray;
      int max_trackbar = 100;

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        src_gray = frame;
        cv::cvtColor(src_gray, frame, cv::COLOR_GRAY2BGR);
      }

      if (debug_view_)
      {
        /// Create Trackbars for Thresholds
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("Max corners", window_name_, &max_corners_, max_trackbar, trackbarCallback);
        if (need_config_update_)
        {
          config_.max_corners = max_corners_;
          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
      }

      /// void goodFeaturesToTrack_Demo( int, void* )
      if (max_corners_ < 1)
      {
        max_corners_ = 1;
      }

      /// Parameters for Shi-Tomasi algorithm
      std::vector<cv::Point2f> corners;
      double quality_level = 0.01;
      double min_distance = 10;
      int block_size = 3;
      bool use_harris_detector = false;
      double k = 0.04;

      cv::RNG rng(12345);

      /// Apply corner detection
      cv::goodFeaturesToTrack(src_gray, corners, max_corners_, quality_level, min_distance, cv::Mat(), block_size,
                              use_harris_detector, k);

      /// Draw corners detected
      NODELET_INFO_STREAM("** Number of corners detected: " << corners.size());
      int r = 4;
      for (const cv::Point2f& corner : corners)
      {
        cv::circle(frame, corner, r, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1, 8,
                   0);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        int c = cv::waitKey(1);
      }

      // Create msgs
      for (const cv::Point2f& i : corners)
      {
        opencv_apps::Point2D corner;
        corner.x = i.x;
        corner.y = i.y;
        corners_msg.points.push_back(corner);
      }
      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(corners_msg);
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
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &GoodfeatureTrackNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &GoodfeatureTrackNodelet::imageCallback, this);
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

    window_name_ = "Image";
    max_corners_ = 23;

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&GoodfeatureTrackNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::Point2DArrayStamped>(*pnh_, "corners", 1);

    onInitPostProcess();
  }
};
bool GoodfeatureTrackNodelet::need_config_update_ = false;
}  // namespace opencv_apps

namespace goodfeature_track
{
class GoodfeatureTrackNodelet : public opencv_apps::GoodfeatureTrackNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet goodfeature_track/goodfeature_track is deprecated, "
             "and renamed to opencv_apps/goodfeature_track.");
    opencv_apps::GoodfeatureTrackNodelet::onInit();
  }
};
}  // namespace goodfeature_track

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::GoodfeatureTrackNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(goodfeature_track::GoodfeatureTrackNodelet, nodelet::Nodelet);
