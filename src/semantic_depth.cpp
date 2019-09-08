#include "depth_mapping/semantic_depth.h"


int* SemanticDepth::freeSpaceCoord(const sensor_msgs::ImageConstPtr& depth_msg,
              const sensor_msgs::ImageConstPtr& segment_msg,
              const sensor_msgs::CameraInfoConstPtr& info_msg,
              const std_msgs:Float64::ConstPtr& objects){
                //create point vector of points of free space 
              }

int* SemanticDepth::objectCoord(const sensor_msgs::ImageConstPtr& depth_msg,
              const std_msgs:Float64::ConstPtr& objects,
              const sensor_msgs::CameraInfoConstPtr& info_msg);

void SemanticDepth::updateFreeSpaceGrid(std::vector<point> *free_space_coord);
void SemanticDepth::updateObjectGrid(std::vector<point> *object_space_coord);
int* SemanticDepth::coord2grid(int8_t x, int8_t y);
void SemanticDepth::updateProb(int8_t p_prior, int8_t p_curr);
void SemanticDepth::buildGrids();
void SemanticDepth::publishGrids();
