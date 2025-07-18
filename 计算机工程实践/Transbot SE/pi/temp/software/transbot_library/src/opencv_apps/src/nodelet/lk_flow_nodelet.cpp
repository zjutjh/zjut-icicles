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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/lk_demo.cpp
/**
 * This is a demo of Lukas-Kanade optical flow lkdemo(),
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <dynamic_reconfigure/server.h>
#include "std_srvs/Empty.h"
#include "opencv_apps/LKFlowConfig.h"
#include "opencv_apps/FlowArrayStamped.h"

namespace opencv_apps
{
class LKFlowNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;
  ros::ServiceServer initialize_points_service_;
  ros::ServiceServer delete_points_service_;
  ros::ServiceServer toggle_night_mode_service_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::LKFlowConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  int MAX_COUNT;
  bool needToInit;
  bool nightMode;
  cv::Point2f point;
  bool addRemovePt;
  cv::Mat gray, prevGray;
  std::vector<cv::Point2f> points[2];

  float quality_level_;
  int min_distance_;
  int block_size_;
  float harris_k_;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    quality_level_ = config_.quality_level;
    min_distance_ = config_.min_distance;
    block_size_ = config_.block_size;
    harris_k_ = config_.harris_k;
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
#if 0
  static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
  {
    if( event == CV_EVENT_LBUTTONDOWN )
    {
      point = Point2f((float)x, (float)y);
      addRemovePt = true;
    }
  }
#endif
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
      cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::FlowArrayStamped flows_msg;
      flows_msg.header = msg->header;

      if (debug_view_)
      {
        /// Create Trackbars for Thresholds
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

        cv::createTrackbar("Min Distance", window_name_, &min_distance_, 100, trackbarCallback);
        cv::createTrackbar("Block Size", window_name_, &block_size_, 100, trackbarCallback);

        // cv::setMouseCallback( window_name_, onMouse, 0 );
        if (need_config_update_)
        {
          reconfigure_server_->updateConfig(config_);
          config_.quality_level = quality_level_;
          config_.min_distance = min_distance_;
          config_.block_size = block_size_;
          config_.harris_k = harris_k_;
          need_config_update_ = false;
        }
      }

      // Do the work
      cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03);
      cv::Size sub_pix_win_size(10, 10), win_size(31, 31);

      if (image.channels() > 1)
      {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        image.copyTo(gray);
      }

      if (nightMode)
        image = cv::Scalar::all(0);

      if (needToInit)
      {
        // automatic initialization
        cv::goodFeaturesToTrack(gray, points[1], MAX_COUNT, quality_level_, min_distance_, cv::Mat(), block_size_,
                                false, harris_k_);
        cv::cornerSubPix(gray, points[1], sub_pix_win_size, cv::Size(-1, -1), termcrit);
        addRemovePt = false;
      }
      else if (!points[0].empty())
      {
        std::vector<uchar> status;
        std::vector<float> err;
        if (prevGray.empty())
          gray.copyTo(prevGray);
        cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, win_size, 3, termcrit, 0, 0.001);
        size_t i, k;
        for (i = k = 0; i < points[1].size(); i++)
        {
          if (addRemovePt)
          {
            if (cv::norm(point - points[1][i]) <= 5)
            {
              addRemovePt = false;
              continue;
            }
          }

          if (!status[i])
            continue;

          points[1][k++] = points[1][i];
          cv::circle(image, points[1][i], 3, cv::Scalar(0, 255, 0), -1, 8);
          cv::line(image, points[1][i], points[0][i], cv::Scalar(0, 255, 0), 1, 8, 0);

          opencv_apps::Flow flow_msg;
          opencv_apps::Point2D point_msg;
          opencv_apps::Point2D velocity_msg;
          point_msg.x = points[1][i].x;
          point_msg.y = points[1][i].y;
          velocity_msg.x = points[1][i].x - points[0][i].x;
          velocity_msg.y = points[1][i].y - points[0][i].y;
          flow_msg.point = point_msg;
          flow_msg.velocity = velocity_msg;
          flows_msg.flow.push_back(flow_msg);
        }
        points[1].resize(k);
      }

      if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
      {
        std::vector<cv::Point2f> tmp;
        tmp.push_back(point);
        cv::cornerSubPix(gray, tmp, win_size, cv::Size(-1, -1), termcrit);
        points[1].push_back(tmp[0]);
        addRemovePt = false;
      }

      needToInit = false;
      if (debug_view_)
      {
        cv::imshow(window_name_, image);

        char c = (char)cv::waitKey(1);
        // if( c == 27 )
        //    break;
        switch (c)
        {
          case 'r':
            needToInit = true;
            break;
          case 'c':
            points[0].clear();
            points[1].clear();
            break;
          case 'n':
            nightMode = !nightMode;
            break;
        }
      }
      std::swap(points[1], points[0]);
      cv::swap(prevGray, gray);

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, image).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(flows_msg);
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  bool initializePointsCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    needToInit = true;
    return true;
  }

  bool deletePointsCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    points[0].clear();
    points[1].clear();
    return true;
  }

  bool toggleNightModeCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    nightMode = !nightMode;
    return true;
  }

  void subscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &LKFlowNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &LKFlowNodelet::imageCallback, this);
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

    window_name_ = "LK Demo";
    MAX_COUNT = 500;
    needToInit = true;
    nightMode = false;
    addRemovePt = false;

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&LKFlowNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::FlowArrayStamped>(*pnh_, "flows", 1);
    initialize_points_service_ = pnh_->advertiseService("initialize_points", &LKFlowNodelet::initializePointsCb, this);
    delete_points_service_ = pnh_->advertiseService("delete_points", &LKFlowNodelet::deletePointsCb, this);
    toggle_night_mode_service_ = pnh_->advertiseService("toggle_night_mode", &LKFlowNodelet::toggleNightModeCb, this);

    NODELET_INFO("Hot keys: ");
    NODELET_INFO("\tESC - quit the program");
    NODELET_INFO("\tr - auto-initialize tracking");
    NODELET_INFO("\tc - delete all the points");
    NODELET_INFO("\tn - switch the \"night\" mode on/off");
    // NODELET_INFO("To add/remove a feature point click it");

    onInitPostProcess();
  }
};
bool LKFlowNodelet::need_config_update_ = false;
}  // namespace opencv_apps

namespace lk_flow
{
class LKFlowNodelet : public opencv_apps::LKFlowNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet lk_flow/lk_flow is deprecated, "
             "and renamed to opencv_apps/lk_flow.");
    opencv_apps::LKFlowNodelet::onInit();
  }
};
}  // namespace lk_flow

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::LKFlowNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(lk_flow::LKFlowNodelet, nodelet::Nodelet);
