

//C++ and ros headers
#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>
#include "ros/ros.h"

// Sensor Message headers
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

//Image transport headers
#include <image_transport/image_transport.h>
#include "image_geometry/pinhole_camera_model.h"
//boost headers
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

//reconfiguration server headers
#include <dynamic_reconfigure/server.h>

// navigation and planning messages
#include <nav_msgs/OccupancyGrid.h>

// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <depth_mapping/object_detect.h>






struct point{
  int x;
  int y;
  int z;
  point(): x(0),y(0),z(0){};
  point(int x, int y, int z):x(x),y(y),z(z){};
};


class SemanticDepth{
public:
  SemanticDepth(ros::NodeHandle& pnh):pnh_(pnh),
                                      image_object_sub(pnh_.subscribe("/detections",1,&SemanticDepth::objectDepthCb,this))
                                      {
    ROS_INFO("semantic depth map creator is set up. Will start publishing the grid now ..............");
  }
private:
  const int8_t P_HIT  =  60;
  const int8_t P_MISS = 30;
  const int height = 100;
  const int width = 100;
  const int resolution = 5;

  ros::NodeHandle pnh_;
  ros::Subscriber image_object_sub;
  sensor_msgs::Image image;
  sensor_msgs::CameraInfo cinfo;
  std::vector<float> detections;

  nav_msgs::OccupancyGrid detection_confidence_map_;
  nav_msgs::OccupancyGrid freespace_confidence_map_;

  std::vector<int8_t> free_space_data_{std::vector<int8_t>(height*width,100)};
  std::vector<int8_t> object_detect_data {std::vector<int8_t>(height*width,50)};

  // std::vector<point> free_space_coord;
  // std::vector<point> object_space_coord;

  void objectDepthCb(const depth_mapping::object_detect::ConstPtr& msg)
          {

        }

  void freeSpaceCoord(const sensor_msgs::ImageConstPtr& depth_msg,const sensor_msgs::ImageConstPtr& segment_msg,image_geometry::PinholeCameraModel& cam_model);
  void objectCoord(const sensor_msgs::ImageConstPtr& depth_msg,image_geometry::PinholeCameraModel& cam_model);
  void creatingBoundingGrid(std::vector<point> object_space_centroids);
  void updateFreeSpaceGrid(std::vector<point> free_space_coord);
  void updateObjectGrid(std::vector<point> object_space_coord);
  int* coord2grid(int8_t x, int8_t y);
  void updateProb(int8_t p_prior, int8_t p_curr);
  void buildPublishFreeSpaceGrids();
  void buildPublishFreeObjectGrids();

};
