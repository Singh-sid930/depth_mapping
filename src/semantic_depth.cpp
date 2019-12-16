#include "depth_mapping/semantic_depth.h"

double depth_detect::SemanticDepth::angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2){

  double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
  double magnitude1 = magnitudeofRay(ray1);
  double magnitude2 = magnitudeofRay(ray2);
  double theta  = acos(dot_product / (magnitude1 * magnitude2));
  return theta;

}
double depth_detect::SemanticDepth::magnitudeofRay(const cv::Point3d& ray){
  return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
}

void depth_detect::SemanticDepth::onePixelToReal(std::vector<darknet_ros_msgs::DetectionCoordinate> detections){
  int d_size = detections.size();
  double x_camera;
  double y_camera;
  int x_pixel;
  int y_pixel;
  double depth_val;

  cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
  cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
  cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

  std::vector<cam_point> detection_vec;

  for(int i = 0; i < d_size; i++){

    x_pixel = detections[i].xc_pixel;
    y_pixel = detections[i].yc_pixel;
    depth_val = detections[i].depth;
    int id = detections[i].id;

    cam_point cm_point;

    cv::Point2d raw_pixel_point(x_pixel, y_pixel);
    cv::Point2d rect_pixel_point = cam_model_.rectifyPoint(raw_pixel_point);
    cv::Point3d pixel_ray = cam_model_.projectPixelTo3dRay(rect_pixel_point);

    double theta = angleBetweenRays(pixel_ray,center_ray);

    ROS_INFO("The x pixel to check is : %d", x_pixel);
    // ROS_INFO("The center pixel to check is : %f", cam_model_.cx());

    if(x_pixel<cam_model_.cx()){
      theta = -theta;
    }
    ROS_INFO("the angle between the rays is : %f", theta);

    //check for negative and positive angles of theta



    double zc_real = depth_val;
    double xc_real = depth_val * tan(theta);

    cm_point.cx = xc_real;
    cm_point.cz = zc_real;
    cm_point.id = id;

    detection_vec.push_back(cm_point);
  }
  // ROS_INFO("the detected coordinate x center is %f",detection_vec[0].cx);
  // ROS_INFO("the detected coordinate y center is %f",detection_vec[0].cz);
  onePixelGridUpdate(detection_vec);
}

void depth_detect::SemanticDepth::onePixelGridUpdate(std::vector<cam_point> detection_vec){
cv::Mat map_data_(width_,height_,CV_8UC3,cv::Scalar(0,0,0));
// map_data_ = map_data_ * 250;
  for(auto point:detection_vec){
    double cx = point.cx;
    double cz = point.cz;
    int id = point.id;
    int* grid_coord = coord2grid(cx,cz);
    int y_coord = grid_coord[1];
    int x_coord = grid_coord[0];
    // map_data_.at<Vec3b>(x_coord,y_coord)[0] = rand()%255;
  if(id == 0){
    map_data_.at<cv::Vec3b>(x_coord,y_coord)[0] = 0;
  }
  else if (id ==1){
    map_data_.at<cv::Vec3b>(x_coord,y_coord)[1] = 0;
  }
  else {
    map_data_.at<cv::Vec3b>(x_coord,y_coord)[2] = 0;
  }
  }
  cv::resize(map_data_, map_data_, cv::Size(), 5, 5);
  // cv::namedWindow( "Display window");// Create a window for display.
  cv::imshow( "Display window", map_data_ );
  cv::waitKey(10);

}

void depth_detect::SemanticDepth::pixelToReal(std::vector<darknet_ros_msgs::DetectionCoordinate> detections)
{

  int d_size = detections.size();
  std::vector<double> x_camera;
  std::vector<double> y_camera;
  std::vector<int> x_pixels;
  std::vector<int> y_pixels;
  std::vector<double> depth_vals;

  std::vector<obj_detections> detected_objects;

  cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
  cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
  cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

  for(int i = 0; i < d_size; i++){
    std::vector<cam_point> detection_vec;

    x_pixels = detections[i].x_pixels;
    y_pixels = detections[i].y_pixels;
    depth_vals = detections[i].depths;
    int id = detections[i].id;

    int pixel_size = x_pixels.size();

    for(int p = 0; p<pixel_size; p++){
      cam_point cm_point;

      int x_pixel = x_pixels[p];
      int y_pixel = y_pixels[p];

      cv::Point2d raw_pixel_point(x_pixel, y_pixel);
      cv::Point2d rect_pixel_point = cam_model_.rectifyPoint(raw_pixel_point);
      cv::Point3d pixel_ray = cam_model_.projectPixelTo3dRay(rect_pixel_point);

      double theta = angleBetweenRays(pixel_ray,center_ray);

      //check for negative and positive angles of theta

      double depth = depth_vals[p];

      // double theta = angleBetweenRays(pixel_ray,center_ray);
      if(x_pixel<cam_model_.cx()){
        theta = -theta;
      }
      // ROS_INFO("the angle between the rays is : %f", theta);

      //check for negative and positive angles of theta



      double zc_real = depth;
      double xc_real = depth * tan(theta);

      cm_point.cx = xc_real;
      cm_point.cz = zc_real;
      detection_vec.push_back(cm_point);
    }
    int vec_size = detection_vec.size();
    // int index = vec_size/2;
    // ROS_INFO("the detected coordinate x center is %f",detection_vec[index].cx);
    // ROS_INFO("the detected coordinate y center is %f",detection_vec[index].cz);
    obj_detections one_object;
    one_object.detection_vec = detection_vec;
    one_object.id = id;
    detected_objects.push_back(one_object);
  }
  updateObjectGrid(detected_objects);

}

void depth_detect::SemanticDepth::updateObjectGrid(std::vector<obj_detections> detected_objects)
{
  cv::Mat map_data_ = cv::Mat::zeros(width_,height_,CV_8UC3);
  detectSpace(map_data_);
  int detection_size = detected_objects.size();
  for(auto detection:detected_objects){
    std::vector<cam_point> detection_vec = detection.detection_vec;
    int id = detection.id;
    for(auto point:detection_vec){
      double cx = point.cx;
      double cz = point.cz;
      int* grid_coord = coord2grid(cx,cz);
      int y_coord = grid_coord[1];
      int x_coord = grid_coord[0];
      // map_data_.at<Vec3b>(x_coord,y_coord)[0] = rand()%255;
      if(id == 0){
        map_data_.at<cv::Vec3b>(x_coord,y_coord)[0] = 250;
        map_data_.at<cv::Vec3b>(x_coord,y_coord)[1] = 0;
        map_data_.at<cv::Vec3b>(x_coord,y_coord)[2] = 0;
      }
      else if (id ==1){
        map_data_.at<cv::Vec3b>(x_coord,y_coord)[1] = 250;
        map_data_.at<cv::Vec3b>(x_coord,y_coord)[0] = 0;
        map_data_.at<cv::Vec3b>(x_coord,y_coord)[2] = 0;
      }
      else {
        map_data_.at<cv::Vec3b>(x_coord,y_coord)[2] = 250;
        map_data_.at<cv::Vec3b>(x_coord,y_coord)[1] = 0;
        map_data_.at<cv::Vec3b>(x_coord,y_coord)[0] = 0;
      }
      // map_data_.at<cv::Vec3b>(x_coord,y_coord)[0] = 250;
    }
    // ROS_INFO("got one object yay");
  }

  cv::resize(map_data_, map_data_, cv::Size(), 5, 5);

  // ROS_INFO("the directory is : %s",  dir_name_);

  if(counter_%3==0 ){
    std::string step = std::to_string(counter_/3);
    imwrite(dir_name_+"/"+step+".jpg", map_data_ );
  }
  counter_++;
  // cv::namedWindow( "Display window");// Create a window for display.
  cv::imshow( "Display window", map_data_ );
  cv::waitKey(10);
}

void depth_detect::SemanticDepth::detectSpace(cv::Mat& map_data_){
  //***********Start filling in points which are detected as obstacles*******///

  int y_pixel = (int)round(cam_model_.cy());
  int x_end = img_wd_ - 2;

  cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
  cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
  cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

  for(int x_pixel = 0; x_pixel<x_end;x_pixel++){
    cv::Point2d raw_pixel_point(x_pixel, y_pixel);
    cv::Point2d rect_pixel_point = cam_model_.rectifyPoint(raw_pixel_point);
    cv::Point3d pixel_ray = cam_model_.projectPixelTo3dRay(rect_pixel_point);
    double theta = angleBetweenRays(pixel_ray,center_ray);

    // ROS_INFO("The x pixel to check is : %d", x_pixel);

    if(x_pixel<cam_model_.cx()){
      theta = -theta;
    }
    // ROS_INFO("the angle between the rays is : %f", theta);

    float depth_val = depth_mat_.at<float>(y_pixel, x_pixel);

    double cz = depth_val;
    double cx = depth_val * tan(theta);

    int* grid_coord = coord2grid(cx,cz);
    int y_coord = grid_coord[1];
    int x_coord = grid_coord[0];
    map_data_.at<cv::Vec3b>(x_coord,y_coord)[0] = 250;
    map_data_.at<cv::Vec3b>(x_coord,y_coord)[1] = 250;
    map_data_.at<cv::Vec3b>(x_coord,y_coord)[2] = 250;
  }

}


int* depth_detect::SemanticDepth::coord2grid(double cx, double cz)
{
  double x = cz;
  double y = cx;
  x = x/1000;
  y = y/1000;
  // ROS_INFO("the detected coordinate x center in real frame in m is %f",x);
  // ROS_INFO("the detected coordinate y center in real frame in m is %f",y);
  x = x*resolution_;
  y = y*resolution_;
  x = round(x);
  y = round(y);
  y = y+(width_/2);
  y = y>(width_-1)?(width_-1):y;
  y = y<0?0:y;
  x = x>(height_-1)?(height_-1):x;
  int x_grid = x;
  int y_grid = y;
  // ROS_INFO("the detected coordinate x center in the grid is %d",x_grid);
  // ROS_INFO("the detected coordinate y center in the grid is %d",y_grid);
  static int coord[2];
  coord[0] = x_grid;
  coord[1] = y_grid;
  return coord;
}

void depth_detect::SemanticDepth::buildGrids(){

}
void depth_detect::SemanticDepth::publishGrids(){

}
void depth_detect::SemanticDepth::publishImages(){

}

int main(int argc, char** argv) {
    int8_t label = 100;

    ros::init(argc, argv, "semantic_depth_node");
    ros::NodeHandle n;

    depth_detect::SemanticDepth semantic_obj(n);

    while(ros::ok()){
      ros::spinOnce();
    }

    return 0;

  }
