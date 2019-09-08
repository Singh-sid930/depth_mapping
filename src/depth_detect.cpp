#include "depth_mapping/depth_detect.h"


//******************************* convert 3d point cloud to 2d point scan *********************************************//
void depth_detect::DetectFreeSpace::conv2dScan(){

  for (auto& i:out_pointcloud_.points){
    if (i.y <cloud_height_+cloud_width_ && i.y>cloud_height_-cloud_width_){

      float x = i.x;
      float y = i.y;
      float z = i.z;
      float theta = atan2(x,z);
      float range = hypotf(x,z);
      scan_map_.insert({theta,range}); // map the angles and ranges to a sorted ordered map angle:range
    }
  }

  auto it_s = scan_map_.begin()++;
  auto it_l = scan_map_.end()--;
  it_l--;
  it_s++;
  float angle_min = it_s->first;
  float angle_max = it_l->first;
  angle_min = angle_min + ANGLE_REDUCTION; //reduce the angle ranges to make sure we get rid of unnecessary noise close to the camera
  angle_max = angle_max - ANGLE_REDUCTION; // especially necessary when we are dealing with point cloud from stereo cameras.

  // PublishScan(angle_min,angle_max);

  updateGrid(angle_min,angle_max);


}

// void depth_detect::DetectFreeSpace::PublishScan(float angle_min, float angle_max){
//
// }

//****************************   update the eigen grid with 0s and 1s for empty and filled spaces and go on to update the confidence grid ******************************//

void depth_detect::DetectFreeSpace::updateGrid(float angle_min, float angle_max){

/// ***************** reinitialize the eigen grid with 0s ****************////
dynamic_grid_ = Eigen::MatrixXi::Zero(height,width);
std::vector<int> index_arr;
std::vector<int8_t> prior_arr;
auto it_upper = scan_map_.lower_bound(angle_max);
auto it_lower = scan_map_.upper_bound(angle_min);
std::map<float,float>::const_iterator it = it_lower;
while(it != it_upper){
  float x = it->second * cos(it->first);
  float y = it->second * sin(it->first);
  int* coordinates = trans2grid(x,y);
  dynamic_grid_(coordinates[0],coordinates[1]) = 1;
  int index = coordinates[1] + coordinates[0]*width;
  index_arr.push_back(index); // array for indexes which are hits.
  int8_t prior_prob = map_data_[index];
  prior_arr.push_back(prior_prob);// array for probability of indexes(grids) which are hits
  it++;
}
updateConfidenceGrid(index_arr,prior_arr); // update the confidence grid based on the index array and prior probability array
index_arr.clear();
prior_arr.clear();
}

//********************************* convert camera coordinates to grid coordinates. *****************************************//

int* depth_detect::DetectFreeSpace::trans2grid(float x, float y){
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

}

//***********************************  convert map to a static grid  ****************************************//

void depth_detect::DetectFreeSpace::updateStaticGrid(){

}

//***********************************  update the dynamic confidence grid, call the update probability function to update probabilities of the grid **************************************//
void depth_detect::DetectFreeSpace::updateConfidenceGrid(std::vector<int>index_arr,std::vector<int8_t> prior_arr){
  int8_t p_hit = 60;
  int8_t p_miss = 30;
  // for(int i = 0; i<map_data_.size();i++){
  //   // int8_t val = 0;
  //   int8_t val = updateProb(map_data_[i],p_miss);
  //   // if(val<50){
  //   // map_data_[i] = 0;
  //   // }
  //   // else{
  //   // map_data_[i] = 99;
  //   // }
  // map_data_[i] = val;
  // }
  // for(int i = 0; i<index_arr.size();i++){
  //   // int8_t val = 100;
  //   int8_t val = updateProb(prior_arr[i],p_hit);
  //   // if(val>50){
  //   //   map_data_[index_arr[i]] = 99;
  //   // }
  //   // else{
  //   //   map_data_[index_arr[i]] = 0;
  //   // }
  //   map_data_[index_arr[i]] = val;
  //   }
  //***********************************************************************////
  for(int i = 0; i<map_data_.size();i++){
    // int8_t val = 0;
    if (std::find(index_arr.begin(), index_arr.end(),i)!=index_arr.end()){
      int8_t val = updateProb(map_data_[i],p_hit);
      map_data_[i] = val;
    }
    else{
      int8_t val = updateProb(map_data_[i],p_miss);
      if(val < 10){
        val = 20;
      }
      map_data_[i] = val;
    }

  }
publishGrids();

}

//********************************** return probability based on the previous probability and current probability (hit or miss)**************************************///

int8_t depth_detect::DetectFreeSpace::updateProb(int8_t p_prior, int8_t p_curr ){
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
}

//************************************ Publish the grids after converting them into cv images *****************************************///
void depth_detect::DetectFreeSpace::publishGrids(){

  confidence_map_.info.resolution = 0.1;         // float32
  confidence_map_.info.width      = width;           // uint32
  confidence_map_.info.height     = height;           // uint32
  confidence_map_.data = map_data_;
  grid_pub_.publish(confidence_map_);
  // cv::Mat dynamic_grid = cv::Mat(100, 100, CV_8UC1, map_data_.data());
  // sensor_msgs::ImagePtr dynamic_grid_image;
  // dynamic_grid_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dynamic_grid).toImageMsg();
  // image_pub_.publish(dynamic_grid_image);

}

///*************************************** driver function *************************************////

int main(int argc, char** argv) {

    ros::init(argc, argv, "zed_video_subscriber");
    ros::NodeHandle n;

    depth_detect::DetectFreeSpace detect_obj(n,0.2,0.0);

    while(ros::ok()){

      ros::spinOnce();
    }

    return 0;

  }
