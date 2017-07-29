#include "liuh_camera/camera_driver.hpp"

#include <ros/ros.h>
#include <ros/rate.h>
#include <image_transport/image_transport.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "liuh_camera");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/liuh_camera/video", 1);

  liuh::CameraDriver camera;

  bool holding = false;

  while (ros::ok()) {
    camera.release();
    liuh::SharedImgPtr image = camera.capture();
    pub.publish(image);
  }
}
