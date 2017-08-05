#include "liuh_yuv_viewer/liuh_yuv_viewer.hpp"

#include <ros/ros.h>
#include <ros/rate.h>
#include <image_transport/image_transport.h>

liuh_yuv_viewer::YUVViewer viewer;

void callback() {
  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "liuh_yuv_viewer");
  ros::NodeHandle nh;
  nh.subscribe("/camera/top", &callback);
  ros::spin();
}
