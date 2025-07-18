#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

class CameraViewer {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    
public:
    CameraViewer() : it_(nh_) {
        // 订阅usb_cam发布的图像话题
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
            &CameraViewer::imageCallback, this);
            
        ROS_INFO("相机查看器已启动，等待图像数据...");
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // 将ROS图像消息转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // 显示图像
            cv::imshow("相机图像", cv_ptr->image);
            cv::waitKey(3);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge异常: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_viewer_node");
    
    CameraViewer viewer;
    
    ros::spin();
    
    cv::destroyAllWindows();
    return 0;
}
