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
#include <iostream>



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

  SemanticDepth(ros::NodeHandle& pnh):pnh_(pnh),coordinate_sub(pnh_.subscribe("/darknet_ros/coordinates",1,&depth_detect::SemanticDepth::depthCb, this)),
                                                info_sub(pnh_.subscribe("camera/color/camera_info", 1, &depth_detect::SemanticDepth::cameraCb,this)){
    // cv::Mat mat1 = cv::Mat::ones(3,4,CV_32FC1);
    // mat1 = mat1 * 200;
    // cv::imshow("test", mat1);
    // cv::waitKey(0);
    // cv::Mat test_data_{cv::Mat::zeros(width_,height_,CV_8UC1)};
    // cv::imshow( "Display window", test_data_ );
    // cv::waitKey(0);


    ROS_INFO("semantic depth map creator is set up. Will start publishing the grid now .....");
  }

private:

  ros::NodeHandle pnh_;
  image_geometry::PinholeCameraModel cam_model_; ///< image_geometry helper class for managing sensor_msgs/CameraInfo messages.
  ros::Subscriber coordinate_sub;
  ros::Subscriber info_sub;


  void depthCb(const darknet_ros_msgs::DetectionCoordinates& msg){
        std::vector<darknet_ros_msgs::DetectionCoordinate> detections_ = msg.detection_coordinates;

        if(detections_.size()!= 0){
        // onePixelToReal(detections_);
        pixelToReal(detections_);
      }

  }

  void cameraCb(const sensor_msgs::CameraInfoConstPtr& info_msg){
    cam_model_.fromCameraInfo(info_msg);
  }

  int height_ = 50;
  int width_ = 50;
  int resolution_ = 5;
  // cv::Mat map_data_{cv::Mat::zeros(width_,height_,CV_8UC1)};
  void pixelToReal(std::vector<darknet_ros_msgs::DetectionCoordinate> detections);
  void onePixelToReal(std::vector<darknet_ros_msgs::DetectionCoordinate> detections);
  double angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2);
  double magnitudeofRay(const cv::Point3d& ray);
  void updateObjectGrid(std::vector<obj_detections> detected_objects);
  void onePixelGridUpdate(std::vector<cam_point> detection_vec);
  int* coord2grid(double cx, double cz);
  void buildGrids();
  void publishGrids();
  void publishImages();
};
}
