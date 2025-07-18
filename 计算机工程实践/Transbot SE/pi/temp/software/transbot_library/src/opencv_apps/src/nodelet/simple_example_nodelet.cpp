/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Tal Regev.
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
// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
/**
 * This is a demo of Simple Example from wiki tutorial
 */

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>

namespace opencv_apps
{
namespace simple_example
{
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int queue_size_;
  bool debug_view_;

public:
  ImageConverter() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("image", queue_size_, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video/raw", 1);

    ros::NodeHandle pnh("~");
    pnh.param("queue_size", queue_size_, 1);
    pnh.param("debug_view", debug_view_, false);
    if (debug_view_)
    {
      cv::namedWindow(OPENCV_WINDOW);
    }
  }

  ~ImageConverter()
  {
    if (debug_view_)
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 110 && cv_ptr->image.cols > 110)
      cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols / 2, cv_ptr->image.rows / 2), 100, CV_RGB(255, 0, 0));

    if (debug_view_)
    {
      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);
    }

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

}  // namespace simple_example

class SimpleExampleNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    simple_example::ImageConverter ic;
    ros::spin();
  }
};

}  // namespace opencv_apps

namespace simple_example
{
class SimpleExampleNodelet : public opencv_apps::SimpleExampleNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet simple_example/simple_example is deprecated, "
             "and renamed to opencv_apps/simple_example.");
    opencv_apps::SimpleExampleNodelet::onInit();
  }
};
}  // namespace simple_example

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::SimpleExampleNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(simple_example::SimpleExampleNodelet, nodelet::Nodelet);
