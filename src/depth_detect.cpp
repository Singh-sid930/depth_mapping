#include "depth_mapping/depth_detect.h"
// #include <math.h>



void depth_detect::DetectFreeSpace::conv2dScan(){

  for (auto i:out_pointcloud_.points){
    // ROS_INFO("in the scan conversion function with cloud height: %f and cloud width: %f", cloud_height_, cloud_width_);
    if (i.y <cloud_height_+cloud_width_ && i.y>cloud_height_-cloud_width_){

      float x = i.x;
      float y = i.y;
      float z = i.z;
      float theta = atan2(x,z);
      float range = hypotf(x,z);
      scan_map_[theta] = range;
    }
  }


  auto it_s = scan_map_.begin()++;
  auto it_l = scan_map_.end()--;
  float angle_min = it_s->first;
  float angle_max = it_l->first;
  angle_min = angle_min + ANGLE_REDUCTION;
  angle_max = angle_max - ANGLE_REDUCTION;
  updateGrid(angle_min,angle_max);
  updateConfidenceGrid(angle_min,angle_max);

}
void depth_detect::DetectFreeSpace::updateGrid(float angle_min, float angle_max){
std::vector<int> index_arr;
std::vector<uint8_t> prior_arr;
auto it_upper = scan_map_.lower_bound(angle_max);
auto it_lower = scan_map_.upper_bound(angle_min);

std::map<float,float>::const_iterator it = it_lower;
while(it != it_upper){
  float x = it->second * cos(it->first);
  float y = it->second * sin(it->first);
  int* coordinates = trans2grid(x,y);
  float prior_prob = dynamic_grid_(coordinates[0],coordinates[1]);
  prior_arr.push_back(prior_prob);
  dynamic_grid_(coordinates[0],coordinates[1]) = 1;
  int index = coordinates[0] + coordinates[1]*1000;
  index_arr.push_back(index);
  it++;
}

updateConfidenceGrid(index_arr,prior_arr);

}

int* depth_detect::DetectFreeSpace::trans2grid(float x, float y){
  x = x*200;
  y = y*200;
  x = round(x);
  y = round(y);
  y = y+500;
  y = y>1000?1000:y;
  y = y<0?0:y;
  x = x>1000?1000:x;
  static int coord[2];
  coord[0] = x;
  coord[1] = y;
  return coord;

}



void depth_detect::DetectFreeSpace::updateStaticGrid(){

}


void depth_detect::DetectFreeSpace::updateConfidenceGrid(vector<int>index_arr,float prior_prob){

  for(int i = 0; i<map_data_.size();i++){

    map_data_[i] = updateProb(map_data_i,P_MISS)

  }

}

void depth_detect::DetectFreeSpace::updateProb(){

}


void depth_detect::DetectFreeSpace::publishGrids(){

}




int main(int argc, char** argv) {

    ros::init(argc, argv, "zed_video_subscriber");
    ros::NodeHandle n;

    depth_detect::DetectFreeSpace detect_obj(n,0.2,0.0);

    while(ros::ok()){

      ros::spinOnce();
    }

    ros::spin();

    return 0;

  }
