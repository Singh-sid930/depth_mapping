#include "depth_mapping/semantic_depth.h"


int* SemanticDepth::freeSpaceCoord(const sensor_msgs::ImageConstPtr& depth_msg,
              const sensor_msgs::ImageConstPtr& segment_msg){
                int n = depth_msg.data.size()<segment_msg.data.size()?depth_msg.data.size():segment_msg.data.size();
                for (int i = 0; i<n; i++){
                  if(segment_msg.data[i]) ==100{
                    Point segment_coord;

                    float x =
                  }
                }
                //create point vector of points of free space
              }

int* SemanticDepth::objectCoord(const sensor_msgs::ImageConstPtr& depth_msg,
              const std_msgs:Float64::ConstPtr& objects){
                //create point vector of object space coordinates
              }

void SemanticDepth::updateFreeSpaceGrid(std::vector<point> *free_space_coord){
  //update free space from the free space point vector
}
void SemanticDepth::updateObjectGrid(std::vector<point> *object_space_coord){
  //update object space coordinaet from the space point vector
}
int* SemanticDepth::coord2grid(int8_t x, int8_t y){
  //convert world coordinates to grid coordinates
}
void SemanticDepth::updateProb(int8_t p_prior, int8_t p_curr){
  //update probability of the grids based on confidence
}
void SemanticDepth::buildGrids(){
  //layer the grids on top of each other
}
void SemanticDepth::publishGrids(){
  //publish the grids
}
