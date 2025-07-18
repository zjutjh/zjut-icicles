/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Yuki Furuta.
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

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/version.hpp>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <string>
#include <opencv2/opencv.hpp>
#include <pwd.h>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv_apps/nodelet.h>
#include <opencv_apps/FaceArrayStamped.h>
#include <opencv_apps/FaceRecognitionTrain.h>
#include <opencv_apps/FaceRecognitionConfig.h>

namespace enc = sensor_msgs::image_encodings;
#if BOOST_VERSION < 105000
namespace fs = boost::filesystem3;  // for hydro
#else
namespace fs = boost::filesystem;
#endif

#if CV_MAJOR_VERSION >= 3
#include <opencv2/face.hpp>
namespace face = cv::face;
#else
namespace face = cv;
#endif

#if CV_MAJOR_VERSION >= 4
#include <opencv2/imgcodecs/legacy/constants_c.h>  // include CV_LOAD_IMAGE_COLOR
#include <opencv2/imgproc/imgproc_c.h>             // include CV_AA
#endif

// utility for resolving path
namespace boost
{
#if BOOST_VERSION < 105000
namespace filesystem3
{  // for hydro
#else
namespace filesystem
{
#endif
template <>
path& path::append<typename path::iterator>(typename path::iterator lhs, typename path::iterator rhs,
                                            const codecvt_type& cvt)
{
  for (; lhs != rhs; ++lhs)
    *this /= *lhs;
  return *this;
}
path user_expanded_path(const path& p)
{
  path::const_iterator it(p.begin());
  std::string user_dir = (*it).string();
  if (user_dir.length() == 0 || user_dir[0] != '~')
    return p;
  path ret;
  char* homedir;
  if (user_dir.length() == 1)
  {
    homedir = getenv("HOME");
    if (homedir == nullptr)
    {
      homedir = getpwuid(getuid())->pw_dir;
    }
  }
  else
  {
    std::string uname = user_dir.substr(1, user_dir.length());
    passwd* pw = getpwnam(uname.c_str());
    if (pw == nullptr)
      return p;
    homedir = pw->pw_dir;
  }
  ret = path(std::string(homedir));
  return ret.append(++it, p.end(), path::codecvt());
}
}  // namespace filesystem
}  // namespace boost

namespace opencv_apps
{
class LabelMapper
{
  std::map<std::string, int> m_;

public:
  void add(std::vector<std::string>& l)
  {
    int id = 0;
    for (std::map<std::string, int>::const_iterator it = m_.begin(); it != m_.end(); ++it)
    {
      if (id < it->second)
        id = it->second + 1;
    }
    for (const std::string& i : l)
    {
      if (m_.find(i) == m_.end())
      {
        m_[i] = id;
        id++;
      }
    }
  }
  std::vector<int> get(std::vector<std::string>& v)
  {
    std::vector<int> ret(v.size());
    for (size_t i = 0; i < v.size(); ++i)
    {
      ret[i] = m_[v[i]];
    }
    return ret;
  }
  std::string lookup(int id)
  {
    for (std::map<std::string, int>::const_iterator it = m_.begin(); it != m_.end(); ++it)
    {
      if (it->second == id)
        return it->first;
    }
    return "nan";
  }
  const std::map<std::string, int>& getMap() const
  {
    return m_;
  }

  void debugPrint()
  {
    ROS_WARN_STREAM("label mapper: debug print:");
    for (std::map<std::string, int>::const_iterator it = m_.begin(); it != m_.end(); ++it)
    {
      ROS_WARN_STREAM("\t" << it->first << ": " << it->second);
    }
    ROS_WARN_STREAM("label mapper: debug print end");
  }
};

class Storage
{
  fs::path base_dir_;

public:
  Storage(const fs::path& base_dir)
  {
    base_dir_ = fs::user_expanded_path(base_dir);
    if (!fs::exists(base_dir_))
    {
      init();
    }
    if (!fs::is_directory(base_dir_))
    {
      std::stringstream ss;
      ss << base_dir_ << " is not a directory";
      throw std::runtime_error(ss.str());
    }
  };
  void init()
  {
    if (!fs::create_directories(base_dir_))
    {
      std::stringstream ss;
      ss << "failed to initialize directory: " << base_dir_;
      throw std::runtime_error(ss.str());
    }
  }

  void load(std::vector<cv::Mat>& images, std::vector<std::string>& labels, bool append = true)
  {
    if (!append)
    {
      images.clear();
      labels.clear();
    }
    fs::directory_iterator end;
    for (fs::directory_iterator it(base_dir_); it != end; ++it)
    {
      if (fs::is_directory(*it))
      {
        std::string label = it->path().stem().string();
        for (fs::directory_iterator cit(it->path()); cit != end; ++cit)
        {
          if (fs::is_directory(*cit))
            continue;
          fs::path file_path = cit->path();
          try
          {
            cv::Mat img = cv::imread(file_path.string(), CV_LOAD_IMAGE_COLOR);
            labels.push_back(label);
            images.push_back(img);
          }
          catch (cv::Exception& e)
          {
            ROS_ERROR_STREAM("Error: load image from " << file_path << ": " << e.what());
          }
        }
      }
    }
  }

  void save(const std::vector<cv::Mat>& images, const std::vector<std::string>& labels)
  {
    if (images.size() != labels.size())
    {
      throw std::length_error("images.size() != labels.size()");
    }
    for (size_t i = 0; i < images.size(); ++i)
    {
      save(images[i], labels[i]);
    }
  }

  void save(const cv::Mat& image, const std::string& label)
  {
    fs::path img_dir = base_dir_ / fs::path(label);
    if (!fs::exists(img_dir) && !fs::create_directories(img_dir))
    {
      std::stringstream ss;
      ss << "failed to initialize directory: " << img_dir;
      throw std::runtime_error(ss.str());
    }
    fs::directory_iterator end;
    int file_count = 0;
    for (fs::directory_iterator it(img_dir); it != end; ++it)
    {
      if (!fs::is_directory(*it))
        file_count++;
    }

    std::stringstream ss;
    // data_dir/person_label/person_label_123456.jpg
    ss << label << "_" << std::setw(6) << std::setfill('0') << file_count << ".jpg";
    fs::path file_path = img_dir / ss.str();
    ROS_INFO_STREAM("saving image to :" << file_path);
    try
    {
      cv::imwrite(file_path.string(), image);
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR_STREAM("Error: save image to " << file_path << ": " << e.what());
    }
  }
};

class FaceRecognitionNodelet : public opencv_apps::Nodelet
{
  typedef opencv_apps::FaceRecognitionConfig Config;
  typedef dynamic_reconfigure::Server<Config> Server;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, opencv_apps::FaceArrayStamped> SyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, opencv_apps::FaceArrayStamped>
      ApproximateSyncPolicy;

  Config config_;
  boost::shared_ptr<Server> cfg_srv_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
  image_transport::SubscriberFilter img_sub_;
  message_filters::Subscriber<opencv_apps::FaceArrayStamped> face_sub_;
  ros::Publisher debug_img_pub_;
  ros::Publisher face_pub_;
  ros::ServiceServer train_srv_;

  bool save_train_data_;
  bool use_async_;
  bool use_saved_data_;
  double face_padding_;
  int queue_size_;
  std::string data_dir_;
  boost::mutex mutex_;

  boost::shared_ptr<LabelMapper> label_mapper_;
  boost::shared_ptr<Storage> storage_;
  cv::Size face_model_size_;
  cv::Ptr<face::FaceRecognizer> model_;

  void drawFace(cv::Mat& img, const opencv_apps::Face& face)
  {
    cv::Rect r(int(face.face.x - face.face.width / 2.0 - face.face.width * face_padding_ / 2.0),
               int(face.face.y - face.face.height / 2.0 - face.face.height * face_padding_ / 2.0),
               int(face.face.width + face.face.width * face_padding_),
               int(face.face.height + face.face.height * face_padding_));
    cv::Scalar color(0.0, 0.0, 255.0);
    int boldness = 2;
    cv::rectangle(img, r.tl(), r.br(), color, boldness, CV_AA);

    double font_scale = 1.5;
    int text_height = 20;
    cv::Point text_bl;
    if (r.br().y + text_height > img.rows)
      text_bl = r.tl() + cv::Point(0, -text_height);
    else
      text_bl = r.br() + cv::Point(-r.width, text_height);
    std::stringstream ss;
    ss << face.label << " (" << std::fixed << std::setprecision(2) << face.confidence << ")";
    cv::putText(img, ss.str(), text_bl, cv::FONT_HERSHEY_PLAIN, font_scale, color, boldness, CV_AA);
  }

  void extractImage(const cv::Mat& img, const opencv_apps::Rect& rect, cv::Mat& ret, double padding = 0.0)
  {
    int x = std::max(0, int(rect.x - rect.width / 2.0 - rect.width * padding / 2.0));
    int y = std::max(0, int(rect.y - rect.height / 2.0 - rect.height * padding / 2.0));
    cv::Rect r(x, y, std::min(img.cols - x, int(rect.width + rect.width * padding)),
               std::min(img.rows - y, int(rect.height + rect.height * padding)));
    ret = img(r);
  }

  void extractImage(const cv::Mat& img, const opencv_apps::Face& face, cv::Mat& ret, double padding = 0.0)
  {
    extractImage(img, face.face, ret, padding);
  }

  void retrain()
  {
    NODELET_DEBUG("retrain");
    std::vector<cv::Mat> images;
    std::vector<std::string> labels;
    train(images, labels);
  }

  void train(std::vector<cv::Mat>& images, std::vector<std::string>& labels)
  {
    size_t new_image_size = images.size();

    if (use_saved_data_)
    {
      storage_->load(images, labels);
    }

    if (images.empty())
      return;

    std::vector<cv::Mat> resized_images(images.size());
    for (int i = 0; i < images.size(); ++i)
    {
      cv::resize(images[i], resized_images[i], face_model_size_, 0, 0, cv::INTER_CUBIC);
      cv::cvtColor(resized_images[i], resized_images[i], CV_BGR2GRAY);
    }

    label_mapper_->add(labels);
    std::vector<int> ids = label_mapper_->get(labels);
    NODELET_INFO_STREAM("training with " << images.size() << " images");
    // label_mapper_->debugPrint();
    model_->train(resized_images, ids);

    if (save_train_data_ && new_image_size > 0)
    {
      std::vector<cv::Mat> new_images(images.begin(), images.begin() + new_image_size);
      std::vector<std::string> new_labels(labels.begin(), labels.begin() + new_image_size);
      storage_->save(new_images, new_labels);
    }
  }

  void predict(const cv::Mat& img, int& label, double& confidence)
  {
    cv::Mat resized_img;
    cv::resize(img, resized_img, face_model_size_, 0, 0, cv::INTER_CUBIC);
    cv::cvtColor(resized_img, resized_img, CV_BGR2GRAY);
    model_->predict(resized_img, label, confidence);
  }

  void faceImageCallback(const sensor_msgs::Image::ConstPtr& image,
                         const opencv_apps::FaceArrayStamped::ConstPtr& faces)
  {
    NODELET_DEBUG("faceImageCallback");
    boost::mutex::scoped_lock lock(mutex_);

    // check if the face data is being trained
    if (label_mapper_->getMap().empty())
    {
      NODELET_WARN_THROTTLE(1.0, "Face data is not trained. Please train first.");
      return;
    }

    // check if need to draw and publish debug image
    bool publish_debug_image = debug_img_pub_.getNumSubscribers() > 0;

    try
    {
      cv::Mat cv_img = cv_bridge::toCvShare(image, enc::BGR8)->image;
      opencv_apps::FaceArrayStamped ret_msg = *faces;
      for (size_t i = 0; i < faces->faces.size(); ++i)
      {
        cv::Mat face_img, resized_image;
        extractImage(cv_img, faces->faces[i], face_img, face_padding_);

        int label_id = -1;
        double confidence = 0.0;
        predict(face_img, label_id, confidence);
        if (label_id == -1)
          continue;
        ret_msg.faces[i].label = label_mapper_->lookup(label_id);
        ret_msg.faces[i].confidence = confidence;

        // draw debug image
        if (publish_debug_image)
          drawFace(cv_img, ret_msg.faces[i]);
      }
      face_pub_.publish(ret_msg);
      if (publish_debug_image)
      {
        sensor_msgs::Image::Ptr debug_img = cv_bridge::CvImage(image->header, enc::BGR8, cv_img).toImageMsg();
        debug_img_pub_.publish(debug_img);
        NODELET_DEBUG("Published debug image");
      }
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR_STREAM("error at image processing: " << e.err << " " << e.func << " " << e.file << " " << e.line);
    }
  }

  bool trainCallback(opencv_apps::FaceRecognitionTrain::Request& req, opencv_apps::FaceRecognitionTrain::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    try
    {
      std::vector<cv::Mat> images(req.images.size());
      bool use_roi = !req.rects.empty();

      if (use_roi && req.images.size() != req.rects.size())
      {
        throw std::length_error("req.images.size() != req.rects.size()");
      }

      for (size_t i = 0; i < req.images.size(); ++i)
      {
        sensor_msgs::Image img = req.images[i];
        images[i] = cv_bridge::toCvCopy(img, enc::BGR8)->image;
        if (use_roi)
        {
          cv::Mat face_img;
          extractImage(images[i], req.rects[i], face_img);
          images[i] = face_img;
        }
      }
      std::vector<std::string> labels(req.labels.begin(), req.labels.end());
      train(images, labels);
      res.ok = true;
      return true;
    }
    catch (cv::Exception& e)
    {
      std::stringstream ss;
      ss << "error at training: " << e.err << " " << e.func << " " << e.file << " " << e.line;
      res.ok = false;
      res.error = ss.str();
    }
    return false;
  }

  void configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    bool need_recreate_model = false;
    bool need_retrain = false;

    use_saved_data_ = config.use_saved_data;
    save_train_data_ = config.save_train_data;
    face_padding_ = config.face_padding;

    if (face_model_size_.width != config.face_model_width)
    {
      face_model_size_.width = config.face_model_width;
      need_retrain = true;
    }
    if (face_model_size_.height != config.face_model_height)
    {
      face_model_size_.height = config.face_model_height;
      need_retrain = true;
    }

    if (data_dir_ != config.data_dir)
    {
      data_dir_ = config.data_dir;
      need_retrain = true;
      label_mapper_.reset(new LabelMapper());
      storage_.reset(new Storage(fs::path(data_dir_)));
    }

    if (config_.model_method != config.model_method)
    {
      need_recreate_model = true;
    }

    if (config_.model_num_components != config.model_num_components)
    {
      need_recreate_model = true;
    }

    if (config.model_method == "LBPH" &&
        (config_.lbph_radius != config.lbph_radius || config_.lbph_neighbors != config.lbph_neighbors ||
         config_.lbph_grid_x != config.lbph_grid_x || config_.lbph_grid_y != config.lbph_grid_y))
    {
      need_recreate_model = true;
    }

    if (need_recreate_model)
    {
      try
      {
        if (config.model_method == "eigen")
        {
// https://docs.opencv.org/3.3.1/da/d60/tutorial_face_main.html
#if (CV_MAJOR_VERSION >= 4) || (CV_MAJOR_VERSION >= 3 && CV_MINOR_VERSION >= 3)
          model_ = face::EigenFaceRecognizer::create(config.model_num_components, config.model_threshold);
#else
          model_ = face::createEigenFaceRecognizer(config.model_num_components, config.model_threshold);
#endif
        }
        else if (config.model_method == "fisher")
        {
#if (CV_MAJOR_VERSION >= 4) || (CV_MAJOR_VERSION >= 3 && CV_MINOR_VERSION >= 3)
          model_ = face::FisherFaceRecognizer::create(config.model_num_components, config.model_threshold);
#else
          model_ = face::createFisherFaceRecognizer(config.model_num_components, config.model_threshold);
#endif
        }
        else if (config.model_method == "LBPH")
        {
#if (CV_MAJOR_VERSION >= 4) || (CV_MAJOR_VERSION >= 3 && CV_MINOR_VERSION >= 3)
          model_ = face::LBPHFaceRecognizer::create(config.lbph_radius, config.lbph_neighbors, config.lbph_grid_x,
                                                    config.lbph_grid_y);
#else
          model_ = face::createLBPHFaceRecognizer(config.lbph_radius, config.lbph_neighbors, config.lbph_grid_x,
                                                  config.lbph_grid_y);
#endif
        }
        need_retrain = true;
      }
      catch (cv::Exception& e)
      {
        NODELET_ERROR_STREAM("Error: create face recognizer: " << e.what());
      }
    }

    if (need_retrain)
    {
      try
      {
        retrain();
      }
      catch (cv::Exception& e)
      {
        NODELET_ERROR_STREAM("Error: train: " << e.what());
      }
    }

    if (config_.model_threshold != config.model_threshold)
    {
      try
      {
#if CV_MAJOR_VERSION >= 3
        if (face::BasicFaceRecognizer* model = dynamic_cast<face::BasicFaceRecognizer*>(model_.get()))
        {
          model->setThreshold(config.model_threshold);
        }
        else if (face::LBPHFaceRecognizer* model = dynamic_cast<face::LBPHFaceRecognizer*>(model_.get()))
        {
          model->setThreshold(config.model_threshold);
        }
#else
        model_->set("threshold", config.model_threshold);
#endif
      }
      catch (cv::Exception& e)
      {
        NODELET_ERROR_STREAM("Error: set threshold: " << e.what());
      }
    }
    config_ = config;
  }

  void subscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("subscribe");
    img_sub_.subscribe(*it_, "image", 1);
    face_sub_.subscribe(*nh_, "faces", 1);
    if (use_async_)
    {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
      async_->connectInput(img_sub_, face_sub_);
      async_->registerCallback(boost::bind(&FaceRecognitionNodelet::faceImageCallback, this, _1, _2));
    }
    else
    {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
      sync_->connectInput(img_sub_, face_sub_);
      sync_->registerCallback(boost::bind(&FaceRecognitionNodelet::faceImageCallback, this, _1, _2));
    }
  }

  void unsubscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("unsubscribe");
    img_sub_.unsubscribe();
    face_sub_.unsubscribe();
  }

public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    Nodelet::onInit();

    // variables
    face_model_size_ = cv::Size(190, 90);

    // dynamic reconfigures
    cfg_srv_ = boost::make_shared<Server>(*pnh_);
    Server::CallbackType f = boost::bind(&FaceRecognitionNodelet::configCallback, this, _1, _2);
    cfg_srv_->setCallback(f);

    // parameters
    pnh_->param("approximate_sync", use_async_, false);
    pnh_->param("queue_size", queue_size_, 100);

    // advertise
    debug_img_pub_ = advertise<sensor_msgs::Image>(*pnh_, "debug_image", 1);
    face_pub_ = advertise<opencv_apps::FaceArrayStamped>(*pnh_, "output", 1);
    train_srv_ = pnh_->advertiseService("train", &FaceRecognitionNodelet::trainCallback, this);
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

namespace face_recognition
{
class FaceRecognitionNodelet : public opencv_apps::FaceRecognitionNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    ROS_WARN("DeprecationWarning: Nodelet face_recognition/face_recognition is deprecated, "
             "and renamed to opencv_apps/face_recognition.");
    opencv_apps::FaceRecognitionNodelet::onInit();
  }
};
}  // namespace face_recognition

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::FaceRecognitionNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(face_recognition::FaceRecognitionNodelet, nodelet::Nodelet);
