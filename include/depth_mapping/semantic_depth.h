#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <boost/bind.hpp>
#include "ros/ros.h"
#include "image_geometry/pinhole_camera_model.h"
#include <darknet_ros_msgs/DetectionCoordinate.h>
#include <darknet_ros_msgs/DetectionCoordinates.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <bits/stdc++.h>
#include <iostream>
#include <boost/filesystem.hpp>
// #include <dir.h>
// #include <process.h>
// #include <stdio.h>



struct cam_point{
  double cx;
  double cz;
  int id;
  cam_point(): cx(0),cz(0),id(0){};
  cam_point(double x , double z):cx(x),cz(z){};
};

struct obj_detections{
  std::vector<cam_point> detection_vec;
  int id;
  obj_detections(){};
  obj_detections(std::vector<cam_point> cam_p, int id): detection_vec(cam_p), id(id){};
};

namespace depth_detect{
class SemanticDepth{

public:

  SemanticDepth(ros::NodeHandle& pnh):pnh_(pnh),coordinate_sub(pnh_.subscribe("/darknet_ros/coordinates",1,&depth_detect::SemanticDepth::detectCb, this)),
                                                info_sub(pnh_.subscribe("camera/color/camera_info", 1, &depth_detect::SemanticDepth::cameraCb,this)),
                                                free_sub(pnh_.subscribe("/camera/depth/image_rect_raw", 1, &depth_detect::SemanticDepth::freespaceCb,this)){
    // cv::Mat mat1 = cv::Mat::ones(3,4,CV_32FC1);
    // mat1 = mat1 * 200;
    // cv::imshow("test", mat1);
    // cv::waitKey(0);
    // cv::Mat test_data_{cv::Mat::zeros(width_,height_,CV_8UC1)};
    // cv::imshow( "Display window", test_data_ );
    // cv::waitKey(0);


    ROS_INFO("semantic depth map creator is set up. Will start publishing the grid now .....");
    boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
    dir_name_ = "/media/siddharth/DATA1/project_data/"+iso_time_str;
    boost::filesystem::create_directories(dir_name_ );
    // if (mkdir(dir_name_, 0777) == -1)
    //      {std::cerr << "Error :  " << strerror(errno) << "\n";}
    //  else
    //      {std::cout << "Directory created"; }
  }

private:

  ros::NodeHandle pnh_;
  image_geometry::PinholeCameraModel cam_model_; ///< image_geometry helper class for managing sensor_msgs/CameraInfo messages.
  ros::Subscriber coordinate_sub;
  ros::Subscriber info_sub;
  ros::Subscriber free_sub;


  void detectCb(const darknet_ros_msgs::DetectionCoordinates& msg){
        std::vector<darknet_ros_msgs::DetectionCoordinate> detections_ = msg.detection_coordinates;

        if(detections_.size()!= 0){
        // onePixelToReal(detections_);
        pixelToReal(detections_);
      }
      return;
      }

  void cameraCb(const sensor_msgs::CameraInfoConstPtr& info_msg){
    cam_model_.fromCameraInfo(info_msg);
    return;
  }

  void freespaceCb(const sensor_msgs::ImageConstPtr& msg){
    depth_data_ = msg->data;
    img_ht_ = msg->height;
    img_wd_ = msg->width;
    const std::string& enc = msg->encoding;
    try {

      depth_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      depth_ptr_->image.convertTo(depth_mat_, CV_32F, 1);

  } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  return;
  }

  int height_ = 100;
  int width_ = 100;
  int resolution_ = 15;
  long counter_ = 0;
  uint32_t img_ht_;
  uint32_t img_wd_;
  std::string dir_name_;
  std::vector<uint8_t> depth_data_;
  cv_bridge::CvImageConstPtr depth_ptr_;
  cv::Mat depth_mat_;
  // cv::Mat map_data_{cv::Mat::zeros(width_,height_,CV_8UC1)};
  void pixelToReal(std::vector<darknet_ros_msgs::DetectionCoordinate> detections);
  void onePixelToReal(std::vector<darknet_ros_msgs::DetectionCoordinate> detections);
  double angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2);
  double magnitudeofRay(const cv::Point3d& ray);
  void updateObjectGrid(std::vector<obj_detections> detected_objects);
  void onePixelGridUpdate(std::vector<cam_point> detection_vec);
  int* coord2grid(double cx, double cz);
  void detectSpace(cv::Mat& map_data_);
  void buildGrids();
  void publishGrids();
  void publishImages();
};
}
