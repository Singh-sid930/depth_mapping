#ifndef DEPTH_DETECT_H
#define DEPTH_DETECT_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <math.h>
// #include <nav_msg/OccupancyGrid.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>

namespace depth_detect{
class DetectFreeSpace{
public:
  DetectFreeSpace(const ros::NodeHandle& pnh, const float cloud_width, const float cloud_height):pnh_(pnh),
                                                                                        cloud_sub_(pnh_.subscribe("/white_lines",1000,&depth_detect::DetectFreeSpace::cloud_cb,this)),
                                                                                        cloud_width_(cloud_width),
                                                                                        cloud_height_(cloud_height),
                                                                                        cloud_pub_(pnh_.advertise<sensor_msgs::PointCloud>("/converted_cloud",1000)){

      ROS_INFO("the cloud subscriber is set up with cloud height and cloud width");

  }
private:
  #define ANGLE_REDUCTION 0.02
  const uint8_t P_HIT  =  60;
  const uint8_t P_MISS = 30;
  Eigen::MatrixXi dynamic_grid_{Eigen::MatrixXi::Zero(1000,1000)};
  Eigen::MatrixXi static_grid_;
  sensor_msgs::PointCloud out_pointcloud_;
  std::vector<uint8_t> map_data_{std::vector<uint8_t>(1000000,50)}; //initialize prior with 0.5 or 50
  // nav_msg::OccupancyGrid confidence_grid_;
  ros::NodeHandle pnh_;
  float cloud_width_;
  float cloud_height_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
  image_transport::Publisher image_pub_;
  std::map<float,float> scan_map_;
  void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
        // ROS_INFO("in the callback");
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud_);
        conv2dScan();
        // ROS_INFO("the z height of the point cloud is = %lu",out_pointcloud_.points.size());
  }
  void conv2dScan();
  int* trans2grid(float x, float y);
  void updateGrid(float angle_min, float angle_max);
  void updateStaticGrid();
  void updateConfidenceGrid(std::vector<int>index_arr,std::vector<uint8_t> prior_arr);
  void publishGrids();
  uint8_t updateProb(uint8_t p_prior, uint8_t p_curr);

};
}


#endif
