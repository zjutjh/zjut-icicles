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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/peopledetect.cpp
/**
 * Demonstrate the use of the HoG descriptor using
 * HOGDescriptor::hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/PeopleDetectConfig.h"
#include "opencv_apps/Rect.h"
#include "opencv_apps/RectArrayStamped.h"

namespace opencv_apps
{
class PeopleDetectNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::PeopleDetectConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  cv::HOGDescriptor hog_;

  double hit_threshold_;
  int win_stride_;
  int padding_;
  double scale0_;
  int group_threshold_;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    hit_threshold_ = config_.hit_threshold;
    win_stride_ = config_.win_stride;
    padding_ = config_.padding;
    scale0_ = config_.scale0;
    group_threshold_ = config_.group_threshold;
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
      opencv_apps::RectArrayStamped found_msg;
      found_msg.header = msg->header;

      // Do the work
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      std::vector<cv::Rect> found, found_filtered;
      double t = (double)cv::getTickCount();
      // run the detector with default parameters. to get a higher hit-rate
      // (and more false alarms, respectively), decrease the hitThreshold and
      // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
      hog_.detectMultiScale(frame, found, hit_threshold_, cv::Size(win_stride_, win_stride_),
                            cv::Size(padding_, padding_), scale0_, group_threshold_);
      t = (double)cv::getTickCount() - t;
      NODELET_INFO("tdetection time = %gms", t * 1000. / cv::getTickFrequency());
      size_t i, j;
      for (i = 0; i < found.size(); i++)
      {
        cv::Rect r = found[i];
        for (j = 0; j < found.size(); j++)
          if (j != i && (r & found[j]) == r)
            break;
        if (j == found.size())
          found_filtered.push_back(r);
      }
      for (i = 0; i < found_filtered.size(); i++)
      {
        cv::Rect r = found_filtered[i];
        // the HOG detector returns slightly larger rectangles than the real objects.
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width * 0.1);
        r.width = cvRound(r.width * 0.8);
        r.y += cvRound(r.height * 0.07);
        r.height = cvRound(r.height * 0.8);
        cv::rectangle(frame, r.tl(), r.br(), cv::Scalar(0, 255, 0), 3);

        opencv_apps::Rect rect_msg;
        rect_msg.x = r.x;
        rect_msg.y = r.y;
        rect_msg.width = r.width;
        rect_msg.height = r.height;
        found_msg.rects.push_back(rect_msg);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(found_msg);
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
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &PeopleDetectNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &PeopleDetectNodelet::imageCallback, this);
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

    window_name_ = "people detector";

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&PeopleDetectNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::RectArrayStamped>(*pnh_, "found", 1);

    onInitPostProcess();
  }
};
bool PeopleDetectNodelet::need_config_update_ = false;
}  // namespace opencv_apps

namespace people_detect
{
class PeopleDetectNodelet : public opencv_apps::PeopleDetectNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet people_detect/people_detect is deprecated, "
             "and renamed to opencv_apps/people_detect.");
    opencv_apps::PeopleDetectNodelet::onInit();
  }
};
}  // namespace people_detect

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::PeopleDetectNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(people_detect::PeopleDetectNodelet, nodelet::Nodelet);
