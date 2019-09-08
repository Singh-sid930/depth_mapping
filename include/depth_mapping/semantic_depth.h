#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <boost/bind.hpp>
#include "ros/ros.h"

struct point{
  int x;
  int y;
  int z;
  point(): x(0),y(0),z(0){};
  point(int x, int y, int z):x(x),y(y),z(z){};
}

class SemanticDepth{
public:
  SemanticDepth(ros::NodeHandle& pnh):pnh_(pnh),
                                      it_(pnh_),
                                      image_sub_(pnh_, "/image", 1),
                                      segment_sub(pnh_, "/segmented_image", 1),
                                      info_sub(pnh_, "/camera_info", 1),
                                      object_sub(pnh_,"/object_list",1),
                                      sync(image_sub, info_sub,object_sub 5)
                                      {
    ROS_INFO("semantic depth map creator is set up. Will start publishing the grid now ..............")
  }
private:
  ros::NodeHandle pnh_;
  ros::Subscriber object_sub;
  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  message_filters::Subscriber<sensor_msgs::Image> segment_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
  message_filters::Subscriber<std_msgs::Float64> object_sub;
  TimeSynchronizer<Image, CameraInfo> sync;
  sync.registerCallback(boost::bind(&depthCb,this, _1, _2,_3))
  // image_transport::ImageTransport it_;
  // image_transport::CameraSubscriber image_sub_;
  nav_msgs::OccupancyGrid detection_confidence_map_;
  nav_msgs::OccupancyGrid freespace_confidence_map_;
  std::vector<int8_t> free_space_data;
  std::vector<int8_t> object_detect_data;


  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
          const sensor_msgs::ImageConstPtr& segment_msg,
	        const sensor_msgs::CameraInfoConstPtr& info_msg,
          const std_msgs:Float64::ConstPtr& objects){
          freeSpaceCoord(depth_msg,segment_msg,info_msg);
          objectCoord(depth_msg,objects,info_msg);

  int* freeSpaceCoord(const sensor_msgs::ImageConstPtr& depth_msg,
                const sensor_msgs::ImageConstPtr& segment_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg,
                const std_msgs:Float64::ConstPtr& objects);

  int* objectCoord(const sensor_msgs::ImageConstPtr& depth_msg,
                const std_msgs:Float64::ConstPtr& objects,
                const sensor_msgs::CameraInfoConstPtr& info_msg);

  void updateFreeSpaceGrid(std::vector<point> *free_space_coord);
  void updateObjectGrid(std::vector<point> *object_space_coord);
  int* coord2grid(int8_t x, int8_t y);
  void updateProb(int8_t p_prior, int8_t p_curr);
  void buildGrids();
  void publishGrids();


};
