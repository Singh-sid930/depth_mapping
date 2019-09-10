#include "depth_mapping/semantic_depth.h"


int* depth_detect::SemanticDepth::freeSpaceCoord(const sensor_msgs::ImageConstPtr& depth_msg,
              const sensor_msgs::ImageConstPtr& segment_msg,
              const sensor_msgs::CameraInfoConstPtr& info_msg,
              const std_msgs:Float64::ConstPtr& objects){

                //create point vector of points of free space
              }

int* depth_detect::SemanticDepth::objectCoord(const sensor_msgs::ImageConstPtr& depth_msg,
              const std_msgs:Float64::ConstPtr& objects,
              const sensor_msgs::CameraInfoConstPtr& info_msg);

void depth_detect::SemanticDepth::updateFreeSpaceGrid(std::vector<point> *free_space_coord);
{

}
void depth_detect::SemanticDepth::updateObjectGrid(std::vector<point> *object_space_coord);
{

}
int* depth_detect::SemanticDepth::coord2grid(int8_t x, int8_t y);
{

}
void depth_detect::SemanticDepth::updateProb(int8_t p_prior, int8_t p_curr);
{

}
void depth_detect::SemanticDepth::buildGrids();{

}
void depth_detect::SemanticDepth::publishGrids();{

}

int main(int argc, char** argv) {
    int8_t label = 100

    ros::init(argc, argv, "semantic_depth_node");
    ros::NodeHandle n;

    depth_detect::SemanticDepth semantic_obj(n,label);

    while(ros::ok()){
      ros::spinOnce();
    }

    return 0;

  }
