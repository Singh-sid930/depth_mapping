#ifndef DEPTH_DETECT_H
#define DEPTH_DETECT_H

// ros dependencies
#include <ros/ros.h>

//c++ headers
#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>
//eigen headers
#include <Eigen/Dense>
#include <Eigen/Geometry>



//Opencv headers
// #include <opencv2/core/eigen.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//sensormsg headers
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>


#include <nav_msgs/OccupancyGrid.h>

//Global constants
#define ANGLE_REDUCTION 0.0




namespace depth_detect{

class DetectFreeSpace{
public:
  DetectFreeSpace(const ros::NodeHandle& pnh, const float cloud_width, const float cloud_height):pnh_(pnh),
                                                                                        cloud_sub_(pnh_.subscribe("/white_lines",1,&depth_detect::DetectFreeSpace::cloud_cb,this)),
                                                                                        cloud_width_(cloud_width),
                                                                                        cloud_height_(cloud_height),
                                                                                        cloud_pub_(pnh_.advertise<sensor_msgs::PointCloud>("/converted_cloud",1)),
                                                                                        grid_pub_(pnh_.advertise<nav_msgs::OccupancyGrid>("/confidence_grid",1)),
                                                                                        it_(pnh_),
                                                                                        image_pub_(it_.advertise("out_image_base_topic", 1)){


      ROS_INFO("the cloud subscriber is set up with cloud height and cloud width");

  }
private:
  //Probability for hitting or missing a cell
  const int8_t P_HIT  =  60;
  const int8_t P_MISS = 30;
  const int height = 100;
  const int width = 100;
  const int resolution = 5;

  //map grids and map data
  Eigen::MatrixXi dynamic_grid_{Eigen::MatrixXi::Zero(height,width)};
  Eigen::MatrixXi static_grid_;
  std::map<float,float> scan_map_;
  std::vector<int8_t> map_data_{std::vector<int8_t>(height*width,50)}; //initialize prior with 0.5 or 50

  //ros node handles and publishers
  ros::NodeHandle pnh_;
  sensor_msgs::PointCloud out_pointcloud_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
  ros::Publisher grid_pub_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  nav_msgs::OccupancyGrid confidence_map_;

  //useful variables
  float cloud_width_;
  float cloud_height_;

  //callback for getting 3d point cloud data.
  void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud_);
        conv2dScan();
        scan_map_.clear();
        // out_pointcloud_.header.clear();
        out_pointcloud_.points.clear();
        out_pointcloud_.channels.clear();
  }

  //functions implemented
  void conv2dScan(); //convert 3d scan to 2d scan
  int* trans2grid(float x, float y); //convert world coordinates to grid coordinates
  void updateGrid(float angle_min, float angle_max); //update dynamic grid with obstacles and free spaces
  void updateStaticGrid();// update static grid from the map if it exists
  void updateConfidenceGrid(std::vector<int>index_arr,std::vector<int8_t> prior_arr); //update confidence grid as an image
  void publishGrids();// publish the grids as cv images
  int8_t updateProb(int8_t p_prior, int8_t p_curr); //update the probility

};
}


#endif
