// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) JSK, 2016 Lab
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

// https://github.com/opencv/opencv/tree/2.4/samples/cpp/tutorial_code/ImgProc/AddingImages.cpp
/**
 * This is a demo of adding image (linear blending).
 */

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "opencv_apps/AddingImagesConfig.h"
#include "opencv_apps/nodelet.h"

namespace opencv_apps
{
class AddingImagesNodelet : public opencv_apps::Nodelet
{
private:
  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
      SyncPolicyWithCameraInfo;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo>
      ApproxSyncPolicyWithCameraInfo;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxSyncPolicy;

  ////////////////////////////////////////////////////////
  // Dynamic Reconfigure
  ////////////////////////////////////////////////////////
  typedef opencv_apps::AddingImagesConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  std::string window_name_;

  image_transport::SubscriberFilter sub_image1_, sub_image2_;
  image_transport::Publisher img_pub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicyWithCameraInfo> > sync_with_info_;
  boost::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicyWithCameraInfo> > async_with_info_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  boost::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy> > async_;
  boost::mutex mutex_;

  bool approximate_sync_;
  double alpha_;
  double beta_;
  double gamma_;

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2,
                             const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    doWork(msg1, msg2, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2)
  {
    doWork(msg1, msg2, msg1->header.frame_id);
  }

  void subscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Subscribing to image topic.");
    sub_image1_.subscribe(*it_, "image1", 3);
    sub_image2_.subscribe(*it_, "image2", 3);
    sub_camera_info_.subscribe(*nh_, "camera_info", 3);
    if (config_.use_camera_info)
    {
      if (approximate_sync_)
      {
        async_with_info_ =
            boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicyWithCameraInfo> >(queue_size_);
        async_with_info_->connectInput(sub_image1_, sub_image2_, sub_camera_info_);
        async_with_info_->registerCallback(boost::bind(&AddingImagesNodelet::imageCallbackWithInfo, this, _1, _2, _3));
      }
      else
      {
        sync_with_info_ = boost::make_shared<message_filters::Synchronizer<SyncPolicyWithCameraInfo> >(queue_size_);
        sync_with_info_->connectInput(sub_image1_, sub_image2_, sub_camera_info_);
        sync_with_info_->registerCallback(boost::bind(&AddingImagesNodelet::imageCallbackWithInfo, this, _1, _2, _3));
      }
    }
    else
    {
      if (approximate_sync_)
      {
        async_ = boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(queue_size_);
        async_->connectInput(sub_image1_, sub_image2_);
        async_->registerCallback(boost::bind(&AddingImagesNodelet::imageCallback, this, _1, _2));
      }
      else
      {
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
        sync_->connectInput(sub_image1_, sub_image2_);
        sync_->registerCallback(boost::bind(&AddingImagesNodelet::imageCallback, this, _1, _2));
      }
    }
  }

  void unsubscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    sub_image1_.unsubscribe();
    sub_image2_.unsubscribe();
    sub_camera_info_.unsubscribe();
  }

  void reconfigureCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    alpha_ = config.alpha;
    if (config.auto_beta)
    {
      beta_ = 1.0 - alpha_;
      config.beta = beta_;
    }
    else
    {
      beta_ = config.beta;
    }
    gamma_ = config.gamma;
  }

  void doWork(const sensor_msgs::Image::ConstPtr& image_msg1, const sensor_msgs::Image::ConstPtr& image_msg2,
              const std::string& input_frame_from_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // Work on the image.
    try
    {
      cv::Mat image1 = cv_bridge::toCvCopy(image_msg1, image_msg1->encoding)->image;
      cv::Mat image2 = cv_bridge::toCvCopy(image_msg2, image_msg1->encoding)->image;
      if (cv_bridge::getCvType(image_msg1->encoding) != cv_bridge::getCvType(image_msg2->encoding))
      {
        NODELET_ERROR("Encoding of input images must be same type: %s, %s", image_msg1->encoding.c_str(),
                      image_msg2->encoding.c_str());
        return;
      }

      cv::Mat result_image;
      if (image1.rows != image2.rows || image1.cols != image2.cols)
      {
        int new_rows = std::max(image1.rows, image2.rows);
        int new_cols = std::max(image1.cols, image2.cols);
        // if ( new_rows != image1.rows || new_cols != image1.cols ) {
        cv::Mat image1 = cv::Mat(new_rows, new_cols, image1.type());
        image1.copyTo(image1(cv::Rect(0, 0, image1.cols, image1.rows)));
        image1 = image1.clone();  // need clone becuase toCvShare??

        // if ( new_rows != image2.rows || new_cols != image2.cols ) {
        cv::Mat image2 = cv::Mat(new_rows, new_cols, image2.type());
        image2.copyTo(image2(cv::Rect(0, 0, image2.cols, image2.rows)));
        image2 = image2.clone();
      }
      cv::addWeighted(image1, alpha_, image2, beta_, gamma_, result_image);
      //-- Show what you got
      sensor_msgs::ImagePtr image_msg3 =
          cv_bridge::CvImage(image_msg1->header, image_msg1->encoding, result_image).toImageMsg();
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
#ifdef CV_BRIDGE_CVT_COLOR_FOR_DISPLAY_IS_NOT_SUPPORTED
        cv::imshow(window_name_, result_image);
#else
#ifdef CV_BRIDGE_CVT_COLOR_FOR_DISPLAY_OPTION_IS_NOT_SUPPORTED
        cv::imshow(window_name_,
                   cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg3, image_msg3->encoding))->image);
#else
        cv_bridge::CvtColorForDisplayOptions options;
        if (sensor_msgs::image_encodings::bitDepth(image_msg1->encoding) == 32 ||
            sensor_msgs::image_encodings::bitDepth(image_msg1->encoding) == 64)
        {
          // float or double image
          options.do_dynamic_scaling = true;
        }
        cv::imshow(window_name_, cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg3), "", options)->image);
#endif
#endif
        int c = cv::waitKey(1);
      }
      img_pub_.publish(image_msg3);
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = image_msg1->header.stamp;
  }

public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("debug_view", debug_view_, false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = ros::Time(0, 0);

    window_name_ = "AddingImages Demo";
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&AddingImagesNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    pnh_->param("approximate_sync", approximate_sync_, true);
    pnh_->param("queue_size", queue_size_, 100);
    img_pub_ = advertiseImage(*pnh_, "image", 1);
    onInitPostProcess();
  }
};
}  // namespace opencv_apps

namespace adding_images
{
class AddingImagesNodelet : public opencv_apps::AddingImagesNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet adding_images/adding_images is deprecated, "
             "and renamed to opencv_apps/adding_images.");
    opencv_apps::AddingImagesNodelet::onInit();
  }
};
}  // namespace adding_images

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::AddingImagesNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(adding_images::AddingImagesNodelet, nodelet::Nodelet);
