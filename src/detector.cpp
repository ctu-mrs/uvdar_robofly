#define MAX_POINTS_PER_IMAGE 100

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <uvdar_robofly/ImagePointsWithFloatStamped.h>
#include <mrs_lib/image_publisher.h>
#include <mrs_lib/param_loader.h>
#include <boost/filesystem/operations.hpp>
/* #include <experimental/filesystem> */
#include <mutex>

#include "detect/uv_led_detect_fast_cpu.h"
#include "detect/uv_led_detect_fast_gpu.h"

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {
class UVDARDetector : public nodelet::Nodelet{
public:

/* onInit() //{ */

  /**
   * @brief Initializer - loads parameters and initializes necessary structures
   */
  void onInit() {

    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    mrs_lib::ParamLoader param_loader(nh_, "UVDARDetector");

    param_loader.loadParam("uav_name", _uav_name_);

    param_loader.loadParam("debug", _debug_, bool(false));
    param_loader.loadParam("gui", _gui_, bool(false));
    param_loader.loadParam("publish_visualization", _publish_visualization_, bool(false));

    param_loader.loadParam("threshold", _threshold_, 200);

    param_loader.loadParam("initial_delay", _initial_delay_, 5.0);

    /* subscribe to cameras //{ */
    std::vector<std::string> _camera_topics;
    param_loader.loadParam("camera_topics", _camera_topics, _camera_topics);
    if (_camera_topics.empty()) {
      ROS_ERROR("[UVDARDetector]: No camera topics were supplied!");
      return;
    }
    _camera_count_ = (unsigned int)(_camera_topics.size());

    /* prepare masks if necessary //{ */
    param_loader.loadParam("use_masks", _use_masks_, bool(false));
    if (_use_masks_){
      param_loader.loadParam("mask_file_names", _mask_file_names_, _mask_file_names_);

      if (_mask_file_names_.size() != _camera_count_){
        ROS_ERROR_STREAM("[UVDARDetector]: Masks are enabled, but the number of mask filenames provided does not match the number of camera topics (" << _camera_count_ << ")!");
        return;
      }

      if (!loadMasks()){
        ROS_ERROR("[UVDARDetector]: Masks are enabled, but the mask files could not be loaded!");
        return;
      }
    }
    //}
    

    // Create callbacks, timers and process objects for each camera
    for (unsigned int i = 0; i < _camera_count_; ++i) {
      image_callback_t callback = [image_index=i,this] (const sensor_msgs::ImageConstPtr& image_msg) { 
        callbackImage(image_msg, image_index);
      };
      cals_image_.push_back(callback);

      timer_process_.push_back(ros::Timer());

      camera_image_sizes_.push_back(cv::Size(0,0));

      images_current_.push_back(cv::Mat());

      detected_points_.push_back(std::vector<cv::Point>());
      sun_points_.push_back(std::vector<cv::Point>());

      /* image_yet_received_.push_back(false); */
      /* initial_delay_start_.push_back(ros::Time::now()); */

      /* mutex_camera_image_.push_back(std::make_unique<std::mutex>()); */

      ROS_INFO("[UVDARDetector]: Initializing FAST-based marker detection...");
      uvdf_ = std::make_unique<UVDARLedDetectFASTGPU>(
            _gui_,
            _debug_,
            _threshold_,
            _threshold_ / 2,
            150,
            _masks_
            );
      if (!uvdf_){
        ROS_ERROR("[UVDARDetector]: Failed to initialize FAST-based marker detection!");
        return;
      }
    }

    // Subscribe to corresponding topics
    for (size_t i = 0; i < _camera_topics.size(); ++i) {
      sub_images_.push_back(nh_.subscribe(_camera_topics[i], 1, cals_image_[i]));
    }

    //}

    
    /* create pubslishers //{ */
    param_loader.loadParam("publish_sun_points", _publish_sun_points_, bool(false));

    std::vector<std::string> _points_seen_topics;
    param_loader.loadParam("points_seen_topics", _points_seen_topics, _points_seen_topics);
    if (_points_seen_topics.size() != _camera_count_) {
      ROS_ERROR_STREAM("[UVDARDetector] The number of output topics (" << _points_seen_topics.size()  << ") does not match the number of cameras (" << _camera_count_ << ")!");
      return;
    }

    // Create the publishers
    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
      pub_candidate_points_.push_back(nh_.advertise<uvdar_robofly::ImagePointsWithFloatStamped>(_points_seen_topics[i], 1));

      if (_publish_sun_points_){
        pub_sun_points_.push_back(nh_.advertise<uvdar_robofly::ImagePointsWithFloatStamped>(_points_seen_topics[i]+"/sun", 1));
      }
    }

    if (_publish_visualization_){
      pub_visualization_ = std::make_unique<mrs_lib::ImagePublisher>(boost::make_shared<ros::NodeHandle>(nh_));
    }
    //}
    //
    if (_gui_ || _publish_visualization_){
      timer_visualization_ = nh_.createTimer(ros::Duration(0.1), &UVDARDetector::VisualizationThread, this, false);
    }



    ROS_INFO("[UVDARDetector]: Waiting for time...");
    ros::Time::waitForValid();

    initialized_ = true;
    ROS_INFO("[UVDARDetector]: Initialized.");
  }
  //}

  /* destructor //{ */
  /**
   * @brief destructor
   */
  ~UVDARDetector() {
  }
  //}

private:

    /* loadMasks //{ */
  /**
   * @brief Load the mask files - either form absolute path or composite filename found in the mrs_uav_general package.
   *
   * @return success
   */
    bool loadMasks(){
      std::string file_name;
      for (unsigned int i=0; i<_camera_count_; i++){

        file_name = _mask_file_names_[i];

        ROS_INFO_STREAM("[UVDARDetector]: Loading mask file [" << file_name << "]");
        if (!(boost::filesystem::exists(file_name))){
          ROS_ERROR_STREAM("[UVDARDetector]: Mask [" << file_name << "] does not exist!");
          return false;
        }

        _masks_.push_back(cv::imread(file_name, cv::IMREAD_GRAYSCALE));
        if (!(_masks_.back().data)){
          ROS_ERROR_STREAM("[UVDARDetector]: Mask [" << file_name << "] could not be loaded!");
          return false;
        }

      }
      return true;
    }
    //}

    /* callbackImage //{ */
    /**
     * @brief Callback for the input raw image topic from camera
     *
     * @param image_msg - current image message
     * @param image_index - index of the camera that produced this image message
     */
  void callbackImage(const sensor_msgs::ImageConstPtr& image_msg, int image_index) {
    cv_bridge::CvImageConstPtr image;
    image = cv_bridge::toCvShare(image_msg, enc::MONO8);
    ros::NodeHandle nh("~");
    timer_process_[image_index] = nh.createTimer(ros::Duration(0), boost::bind(&UVDARDetector::processSingleImage, this, _1, image, image_index), true, true);
    camera_image_sizes_[image_index] = image->image.size();

    if (!all_cameras_detected_){
      unsigned int i = 0;
      for (auto sz : camera_image_sizes_){
        if ( (sz.width > 0) && (sz.height > 0) ){
          i++;
        }
      }

      if ( i == _camera_count_){
        all_cameras_detected_ = true;
      }
    }
  }
  //}


  /* processSingleImage //{ */

  /**
   * @brief Extracts small bright points from input image and publishes them. Optionally also publishes points corresponding to the sun.
   *
   * @param te - timer event - necessary for use of this method as a timer callback
   * @param image - the input image
   * @param image_index - index of the camera that produced this image
   */
  void processSingleImage([[maybe_unused]] const ros::TimerEvent& te, const cv_bridge::CvImageConstPtr image, int image_index) {

    if (!all_cameras_detected_){
      ROS_WARN_STREAM_THROTTLE(1.0, "[UVDARDetector]: Not all cameras have produced input, waiting...");
      return;
    }

    if (!initial_delay_started_){
      initial_delay_start_ = ros::Time::now();
      initial_delay_started_ = true;
    }

    /* double initial_delay = 5.0; //seconds. This delay is necessary to avoid strange segmentation faults with software rendering backend for OpenGL used in the buildfarm testing. */
    if ((ros::Time::now() - initial_delay_start_).toSec() < _initial_delay_){
      ROS_WARN_STREAM_THROTTLE(1.0, "[UVDARDetector]: Ignoring message for "<< _initial_delay_ <<"s...");
      return;
    }


    if (!initialized_){
      ROS_WARN_STREAM_THROTTLE(1.0,"[UVDARDetector]: Not yet initialized, dropping message...");
      return;
    }

      /* ROS_INFO_STREAM("[UVDARDetector]: Locking cam image mutex " << image_index << "..."); */
    {
      /* std::scoped_lock lock(*mutex_camera_image_[image_index]); */
      std::scoped_lock lock(mutex_camera_image_);

      if (!uvdf_was_initialized_){
        if (!uvdf_->initDelayed(image->image)){
          ROS_WARN_STREAM_THROTTLE(1.0,"[UVDARDetector]: Failed to initialize, dropping message...");
          nh_.shutdown();
          ros::Duration(2.0).sleep();
          ros::shutdown(); // if the graphics renderer fails to load, we have to kill the program (ROS nodelet manager and its nodes) and let them restart by setting respawn="true" in the launcher. I could find no better solution to this spurious issue.
          return;
        }
        uvdf_was_initialized_ = true;
      }
      
      images_current_[image_index] = image->image;
      sun_points_[image_index].clear();
      detected_points_[image_index].clear();

      if ( ! (uvdf_->processImage(
              image->image,
              detected_points_[image_index],
              sun_points_[image_index],
              _use_masks_?image_index:-1
              )
            )
         ){
        ROS_WARN_STREAM("Failed to extract markers from the image!");
        return;
      }
      /* ROS_INFO_STREAM("Cam" << image_index << ". There are " << detected_points_[image_index].size() << " detected points."); */

      if (sun_points_[image_index].size() > 30){
        ROS_ERROR_STREAM("There are " << sun_points_[image_index].size() << " detected potential sun points! Check your exposure!");
      }
      /* ROS_INFO_STREAM("There are " << sun_points_[image_index].size() << " detected potential sun points."); */
    }
    /* ROS_INFO_STREAM("[UVDARDetector]: Unlocking cam image mutex " << image_index << "..."); */

    if (detected_points_[image_index].size()>MAX_POINTS_PER_IMAGE){
      ROS_WARN_STREAM("[UVDARDetector]: Over " << MAX_POINTS_PER_IMAGE << " points received. Skipping noisy image.");
      return;
    }

    {
      std::scoped_lock lock(mutex_pub_);
      if (_publish_sun_points_){
        uvdar_robofly::ImagePointsWithFloatStamped msg_sun;
        msg_sun.stamp = image->header.stamp;
        msg_sun.image_width = image->image.cols;
        msg_sun.image_height = image->image.rows;
        for (auto& sun_point : sun_points_[image_index]) {
          uvdar_robofly::Point2DWithFloat point;
          point.x = sun_point.x;
          point.y = sun_point.y;
          msg_sun.points.push_back(point);
        }
        pub_sun_points_[image_index].publish(msg_sun);
      }

      uvdar_robofly::ImagePointsWithFloatStamped msg_detected;
      msg_detected.stamp = image->header.stamp;
      msg_detected.image_width = image->image.cols;
      msg_detected.image_height = image->image.rows;
      for (auto& detected_point : detected_points_[image_index]) {
        uvdar_robofly::Point2DWithFloat point;
        point.x = detected_point.x;
        point.y = detected_point.y;
        msg_detected.points.push_back(point);
      }
      pub_candidate_points_[image_index].publish(msg_detected);
    }

  }
  //}

  /* VisualizationThread() //{ */
  void VisualizationThread([[maybe_unused]] const ros::TimerEvent& te) {
    if (initialized_){
      generateVisualization(image_visualization_);
      if ((image_visualization_.cols != 0) && (image_visualization_.rows != 0)){
        if (_publish_visualization_){
          pub_visualization_->publish("uvdar_detection_visualization", 0.01, image_visualization_, true);
        }
        if (_gui_){
          cv::imshow("ocv_uvdar_detection_" + _uav_name_, image_visualization_);
          cv::waitKey(25);
        }
      }
    }
  }
  //}

  /* generateVisualization //{ */
  int generateVisualization(cv::Mat& output_image) {
    int max_image_height = 0;
    int sum_image_width = 0;
    std::vector<int> start_widths;
    for (auto curr_size : camera_image_sizes_){
      if (max_image_height < curr_size.height){
        max_image_height = curr_size.height;
      }
      start_widths.push_back(sum_image_width);
      sum_image_width += curr_size.width;
    }

    output_image = cv::Mat(cv::Size(sum_image_width+((int)(camera_image_sizes_.size())-1), max_image_height),CV_8UC3);
    output_image = cv::Scalar(255, 255, 255);

    int image_index = 0;
    for ([[maybe_unused]] auto curr_size : camera_image_sizes_){
      /* std::scoped_lock lock(*(mutex_camera_image_[image_index])); */
      std::scoped_lock lock(mutex_camera_image_);
      cv::Point start_point = cv::Point(start_widths[image_index]+image_index, 0);
      cv::Mat image_rgb;
      cv::cvtColor(images_current_[image_index], image_rgb, cv::COLOR_GRAY2BGR);
      image_rgb.copyTo(output_image(cv::Rect(start_point.x,0,images_current_[image_index].cols,images_current_[image_index].rows)));

      for (int j = 0; j < (int)(detected_points_[image_index].size()); j++) {
        cv::circle(output_image, detected_points_[image_index][j]+start_point, 5, cv::Scalar(255,0,0));
      }
      for (int j = 0; j < (int)(sun_points_[image_index].size()); j++) {
        cv::circle(output_image, sun_points_[image_index][j]+start_point, 10, cv::Scalar(0,0,255));
      }

      image_index++;
    }

    if ( (output_image.cols == 0) || (output_image.rows == 0) ){
      return -1;
    }
    else {
      return 0;
    }
  }
  //}

  
private:
  std::string _uav_name_;
  bool initialized_ = false;

  ros::NodeHandle nh_;

    std::vector<ros::Subscriber> sub_images_;
  unsigned int _camera_count_;
  using image_callback_t = boost::function<void (const sensor_msgs::ImageConstPtr&)>;
  std::vector<image_callback_t> cals_image_;


  bool _publish_sun_points_ = false;

  std::vector<ros::Publisher> pub_sun_points_;
  std::vector<ros::Publisher> pub_candidate_points_;


  bool _debug_;

  std::vector<cv::Mat> images_current_;
  std::vector<std::vector<cv::Point>> detected_points_;
  std::vector<std::vector<cv::Point>> sun_points_;

  bool _gui_;
  bool _publish_visualization_;
  std::unique_ptr<mrs_lib::ImagePublisher> pub_visualization_;
  /* std::vector<std::unique_ptr<std::mutex>>  mutex_camera_image_; */
  std::mutex  mutex_camera_image_;
  ros::Timer timer_visualization_;
  ros::Timer timer_gui_visualization_;
  ros::Timer timer_publish_visualization_;
  cv::Mat image_visualization_;
  std::mutex mutex_visualization_;

  std::vector<cv::Size> camera_image_sizes_;

  bool all_cameras_detected_ = false;

  int  _threshold_;

  double _initial_delay_ = 5.0;

  bool _use_masks_;
  std::vector<std::string> _mask_file_names_;
  std::vector<cv::Mat> _masks_;

  /* std::vector<std::unique_ptr<UVDARLedDetectFAST>> uvdf_; */
  std::unique_ptr<UVDARLedDetectFAST> uvdf_;
  std::mutex  mutex_pub_;
  std::vector<ros::Timer> timer_process_;

  bool uvdf_was_initialized_ = false;
  bool initial_delay_started_ = false;
  ros::Time initial_delay_start_;


};


} //namespace uvdar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDARDetector, nodelet::Nodelet)
