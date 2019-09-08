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
#
#include "ros/ros.h"

class SemanticDepth{
public:
  SemanticDepth(ros::NodeHandle& pnh):pnh_(pnh),
                                      it_(pnh_),
                                      image_sub_(pnh_, "/image", 1),
                                      info_sub(pnh_, "/camera_info", 1)
                                      object_sub(pnh_,"/object_list",1)
                                      {
    ROS_INFO("semantic depth map creator is set up. Will start publishing the grid now ..............")
  }
private:
  ros::NodeHandle pnh_;
  ros::Subscriber object_sub;
  message_filters::Subscriber<sensor_msgs::Image> image_sub();
  message_filters::Subscriber<sensor_msgsCameraInfo> info_sub;
  message_filters::Subscriber<std_msgs::Float64> object_sub;
  TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub,object_sub 5);
  sync.registerCallback(boost::bind(&depthCb,this, _1, _2))
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  nav_msgs::OccupancyGrid confidence_map_;

  depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
	      const sensor_msgs::CameraInfoConstPtr& info_msg, const std_msgs:Float64::ConstPtr& objects){
          convert2coord(depth_msg,info_msg,objects);
        }
      convert2coord(const sensor_msgs::ImageConstPtr& depth_msg,
    	      const sensor_msgs::CameraInfoConstPtr& info_msg, const std_msgs:Float64::ConstPtr& objects);



};
