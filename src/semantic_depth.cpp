#include "depth_mapping/semantic_depth.h"


void SemanticDepth::freeSpaceCoord(const sensor_msgs::ImageConstPtr& depth_msg,
              const sensor_msgs::ImageConstPtr& segment_msg,image_geometry::PinholeCameraModel& cam_model){
                // int n = depth_msg.data.size()<segment_msg.data.size()?depth_msg.data.size():segment_msg.data.size();
                // int height = depth_msg.height;
                // int width = depth_msg.width;
                // for (int i = 0; i<n; i++){
                //   if(segment_msg.data[i]) ==100{
                //     Point segment_coord;
                //     int pix_x = i/height;
                //     int pix_y = i%width;
                //     ///// *********************** Check here ************************ ////
                //     cv::Point2d raw_pixel(pix_x, pix_y);
                //     cv::Point2d rect_pixel= cam_model_.rectifyPoint(raw_pixel);
                //     cv::Point3d ray = cam_model_.projectPixelTo3dRay(rect_pixel);
                //     segment_coord.x = ray.x;
                //     segment_coord.y = ray.y;
                //     segment_coord.z = depth_msg.data[i];
                //     std::vector<point> free_space_coord;
                //     free_space_coord.push_back(segment_coord);
                //   }
                // }
                // updateFreeSpaceGrid(free_space_coord);
                // //create point vector of points of free space
              }

void SemanticDepth::objectCoord(const sensor_msgs::ImageConstPtr& depth_msg,image_geometry::PinholeCameraModel& cam_model){
                // int n = objects.size()
                // int height = depth_msg.height;
                // int width = depth_msg.width;
                // for (int i = 0; i<n; i++){
                //   // if(segment_msg.data[i]) ==100{
                //   if(depth_msg.data[i])<2.0{
                //     Point object_coord;
                //     int pix_x = i/height;
                //     int pix_y = i%width;
                //     cv::Point2d raw_pixel(pix_x, pix_y);
                //     cv::Point2d rect_pixel= cam_model_.rectifyPoint(raw_pixel);
                //     cv::Point3d ray = cam_model_.projectPixelTo3dRay(rect_pixel);
                //     object_coord.x = ray.x;
                //     object_coord.y = ray.y;
                //     object_coord.z = depth_msg.data[i]
                //     std::vector<point> object_space_centroids;
                //     object_space_centroids.push_back(object_coord);
                //   }
                // }
                // //create point vector of object space coordinates
                // creatingBoundingGrid(object_space_centroids);
              }

void SemanticDepth::creatingBoundingGrid(std::vector<point> object_space_centroids){

              // std::vector<point> object_space_coord;
              // Point object_coord;
              // for(auto& centroid:object_space_centroids){
              //   object_space_coord.push_back(centroid);
              //   object_coord(centroid.x + 1,centroid.y + 1);
              //   object_space_coord.push_back(object_coord);
              //   object_coord(centroid.x - 1,centroid.y + 1);
              //   object_space_coord.push_back(centroid);
              //   object_coord(centroid.x - 1,centroid.y - 1);
              //   object_space_coord.push_back(object_coord);
              //   object_coord(centroid.x + 1,centroid.y - 1);
              //   object_space_coord.push_back(object_coord);
              // }
              // updateObjectGrid(object_space_coord);
              }


void SemanticDepth::updateFreeSpaceGrid(std::vector<point> free_space_coord){
  //update free space from the free space point vector
  // for(int i = 0; i < free_space_coord.size(); i++){
  //   int* coordinates = coord2grid(free_space_coord[i].x,free_space_coord[i].y);
  //   int index = coordinates[1] + coordinates[0]*width;
  //   free_space_data_[index] = 0;
  // }
  // buildPublishFreeSpaceGrids();
}
void SemanticDepth::updateObjectGrid(std::vector<point> object_space_coord){
  // std::vector<int> index_arr;
  // for(int i = 0; i < object_space_coord.size(); i++){
  //     int* coordinates = coord2grid(object_space_coord[i].x,object_space_coord[i].y);
  //     int index = coordinates[1] + coordinates[0]*width;
  //     index_arr.push_back(index);
  // }
  //
  // for(int i = 0; i<object_space_data_.size();i++){
  //   if (std::find(index_arr.begin(), index_arr.end(),i)!=index_arr.end()){
  //     int8_t val = updateProb(object_space_data_[i],P_HIT);
  //     map_data_[i] = val;
  //   }
  //   else{
  //     int8_t val = updateProb(object_space_data_[i],P_MISS);
  //     if(val < 10){
  //       val = 20;
  //     }
  //     object_space_data_[i] = val;
  //   }
  // }
  //
  // //update object space coordinate from the space point vector
}
int* SemanticDepth::coord2grid(int8_t x, int8_t y){
  // x = x*resolution*5;
  // y = y*resolution*5;
  // x = round(x);
  // y = round(y);
  // y = y+(width/2);
  // y = y>(width-1)?(width-1):y;
  // y = y<0?0:y;
  // x = x>(height-1)?(height-1):x;
  // static int coord[2];
  // coord[0] = x;
  // coord[1] = y;
  // return coord;
  // //convert world coordinates to grid coordinates
}
void SemanticDepth::updateProb(int8_t p_prior, int8_t p_curr){
  // float prior_prob = static_cast<float>(p_prior);
  // prior_prob = prior_prob/100;
  // float curr_prob = static_cast<float>(p_curr);
  // curr_prob = curr_prob/100;
  // float res = (prior_prob/(1-prior_prob)) * (curr_prob/(1-curr_prob));
  // res = res/(1+res);
  // res = res*100;
  //
  // res = res<=0?1:res;
  // res = res>=100?99:res;
  // return (int8_t)res;
  // //update probability of the grids based on confidence
}
void SemanticDepth::buildPublishFreeObjectGrids(){
  // detection_confidence_map_.info.resolution = 0.1;         // float32
  // detection_confidence_map_.info.width      = width;           // uint32
  // detection_confidence_map_.info.height     = height;           // uint32
  // detection_confidence_map_.data = object_detect_data;
  // object_grid_pub_.publish(object_confidence_map_);
  //
  // //layer the grids on top of each other
}

void SemanticDepth::buildPublishFreeSpaceGrids(){

  // freespace_confidence_map_.info.resolution = 0.1;         // float32
  // freespace_confidence_map_.info.width      = width;           // uint32
  // freespace_confidence_map_.info.height     = height;           // uint32
  // freespace_confidence_map_.data = free_space_data_;
  // freespace_grid_pub_.publish(freespace_confidence_map_);


}


int main(int argc, char** argv) {
    int8_t label = 100;

    ros::init(argc, argv, "semantic_depth_node");
    ros::NodeHandle n;

    SemanticDepth semantic_obj(n);

    while(ros::ok()){
      ros::spinOnce();
    }

    return 0;

  }
