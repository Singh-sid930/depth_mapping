// ros dependencies
#include <ros/ros.h>

//c++ headers
#include <vector>
#include <iostream>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <depth_mapping/scan_range.h>
#include <std_msgs/Float64.h>

float range_min;
float range_max;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	 range_min = msg->range_min;
   range_max = msg->range_max;
   std::set<float> scan_set;
   std::vector<float> scan_vec = msg->ranges;
   for(auto& i:msg->ranges){

    scan_set.insert(i);

   }

   auto it_s = scan_set.begin()++;
   ++it_s;
   auto it_l = scan_set.begin()--;
   --it_l;
   range_min = *it_s;
   range_max = *it_l;


}


int main(int argc, char** argv) {

    ros::init(argc, argv, "lidar_processing_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
    ros::Publisher range_pub = n.advertise<depth_mapping::scan_range>("/scan_range", 10);
    ros::Publisher close_pub = n.advertise<std_msgs::Float64>("/closest_point", 10);
    ros::Publisher far_pub = n.advertise<std_msgs::Float64>("/farthest_point", 10);

    depth_mapping::scan_range range;
    std_msgs::Float64 close_point;
    std_msgs::Float64 far_point;




    while(ros::ok()){
      range.min_val = range_min;
      range.max_val = range_max*10e41;
      close_point.data = range_min;
      far_point.data = range_max*10e41;
      range_pub.publish(range);
      close_pub.publish(close_point);
      far_pub.publish(far_point);

      loop_rate.sleep();

      ros::spinOnce();
    }

    return 0;

  }
