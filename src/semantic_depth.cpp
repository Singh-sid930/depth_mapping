#include "depth_mapping/semantic_depth.h"


int* SemanticDepth::freeSpaceCoord(const sensor_msgs::ImageConstPtr& depth_msg,
              const sensor_msgs::ImageConstPtr& segment_msg){
                int n = depth_msg.data.size()<segment_msg.data.size()?depth_msg.data.size():segment_msg.data.size();
                int len = depth_msg.length;
                int width = depth_msg.width;
                for (int i = 0; i<n; i++){
                  if(segment_msg.data[i]) ==100{
                    Point segment_coord;
                    int pix_x = i/len;
                    int pix_y = i%width;
                    cv::Point2d raw_pixel(pix_x, pix_y);
                    cv::Point2d rect_pixel= cam_model_.rectifyPoint(raw_pixel);
                    cv::Point3d ray = cam_model_.projectPixelTo3dRay(rect_pixel);
                    segment_coord.x = ray.x;
                    segment_coord.y = ray.y;
                    segment_coord.z = ray.z;
                    std::vector<point> free_space_coord;
                    free_space_coord.push_back(segment_coord);
                  }
                }
                updateFreeSpaceGrid(free_space_coord);
                //create point vector of points of free space
              }

int* SemanticDepth::objectCoord(const sensor_msgs::ImageConstPtr& depth_msg,
              const std_msgs:Float64::ConstPtr& objects){
                int n = objects.size()
                int len = depth_msg.length;
                int width = depth_msg.width;
                for (int i = 0; i<n; i++){
                  if(segment_msg.data[i]) ==100{
                    Point object_coord;
                    int pix_x = i/len;
                    int pix_y = i%width;
                    cv::Point2d raw_pixel(pix_x, pix_y);
                    cv::Point2d rect_pixel= cam_model_.rectifyPoint(raw_pixel);
                    cv::Point3d ray = cam_model_.projectPixelTo3dRay(rect_pixel);
                    object_coord.x = ray.x;
                    object_coord.y = ray.y;
                    object_coord.z = ray.z;
                    std::vector<point> object_space_centroids
                    object_space_centroids.push_back(object_coord);
                  }
                }
                //create point vector of object space coordinates
                creatingBoundingGrid(object_space_centroids);
              }

void SemanticDepth::creatingBoundingGrid(std::vector<point> object_space_centroids){

              std::vector<point> object_space_coord
              Point object_coord;
              for(auto& centroid:object_space_centroids){
                object_space_coord.push_back(centroid);
                object_coord(centroid.x + 1,centroid.y + 1);
                object_space_coord.push_back(object_coord);
                object_coord(centroid.x - 1,centroid.y + 1);
                object_space_coord.push_back(centroid);
                object_coord(centroid.x - 1,centroid.y - 1);
                object_space_coord.push_back(object_coord);
                object_coord(centroid.x + 1,centroid.y - 1);
                object_space_coord.push_back(object_coord);
              }
              updateObjectGrid(object_space_coord);
              }


void SemanticDepth::updateFreeSpaceGrid(std::vector<point>& free_space_coord){
  //update free space from the free space point vector
  for(int i = 0; i < free_space_coord.size(); i++){
    int* coordinates = coord2grid(free_space_coord[i].x,free_space_coord[i].y);
    int index = coordinates[1] + coordinates[0]*width;
    free_space_data_[index] = 0;
  }
}
void SemanticDepth::updateObjectGrid(std::vector<point>& object_space_coord){
  for(int i = 0; i < object_space_coord.size(); i++){
      int* coordinates = coord2grid(object_space_coord[i].x,object_space_coord[i].y);
      int index = coordinates[1] + coordinates[0]*width;
      object_space_data_[index] = updateProb(object_space_data_[index], P_HIT);
  }
  //update object space coordinate from the space point vector
}
int* SemanticDepth::coord2grid(int8_t x, int8_t y){
  x = x*resolution*5;
  y = y*resolution*5;
  x = round(x);
  y = round(y);
  y = y+(width/2);
  y = y>(width-1)?(width-1):y;
  y = y<0?0:y;
  x = x>(height-1)?(height-1):x;
  static int coord[2];
  coord[0] = x;
  coord[1] = y;
  return coord;
  //convert world coordinates to grid coordinates
}
int8_t SemanticDepth::updateProb(int8_t p_prior, int8_t p_curr){
  float prior_prob = static_cast<float>(p_prior);
  prior_prob = prior_prob/100;
  float curr_prob = static_cast<float>(p_curr);
  curr_prob = curr_prob/100;
  float res = (prior_prob/(1-prior_prob)) * (curr_prob/(1-curr_prob));
  res = res/(1+res);
  res = res*100;

  res = res<=0?1:res;
  res = res>=100?99:res;
  return (int8_t)res;
  //update probability of the grids based on confidence
}
void SemanticDepth::buildPublishGrids(){
  detection_confidence_map_.info.resolution = 0.1;         // float32
  detection_confidence_map_.info.width      = width;           // uint32
  detection_confidence_map_.info.height     = height;           // uint32
  detection_confidence_map_.data = object_detect_data;
  object_grid_pub_.publish(object_confidence_map_);

  freespace_confidence_map_.info.resolution = 0.1;         // float32
  freespace_confidence_map_.info.width      = width;           // uint32
  freespace_confidence_map_.info.height     = height;           // uint32
  freespace_confidence_map_.data = free_space_data_;
  freespace_grid_pub_.publish(freespace_confidence_map_);
  //layer the grids on top of each other
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
