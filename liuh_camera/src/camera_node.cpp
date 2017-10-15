#include "liuh_camera/camera_driver.hpp"

#include <ros/ros.h>
#include <ros/rate.h>
#include <image_transport/image_transport.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "liuh_camera");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_top;
  image_transport::Publisher pub_bottom;

  int fps_top = 0;
  nh.getParam("fps_top", fps_top);
  ROS_INFO("fps_top param: %d", fps_top);

  int fps_bottom = 0;
  nh.getParam("fps_bottom", fps_bottom);
  ROS_INFO("fps_bottom param: %d", fps_bottom);

  if (fps_top) {
    pub_top = it.advertise("top", 1);
  }

  if (fps_bottom) {
    pub_bottom = it.advertise("bottom", 1);
  }

  if (fps_top && fps_bottom) {
    ROS_INFO("Using both cameras!");
    liuh_camera::CameraDriver camera_top(liuh_camera::CAMERA_TOP, fps_top);
    liuh_camera::CameraDriver camera_bottom(liuh_camera::CAMERA_BOTTOM, fps_bottom);

    while (ros::ok()) {
      liuh_camera::CameraDevice which;
      liuh_camera::SharedImgPtr image = liuh_camera::CameraDriver::capture(camera_top, camera_bottom, which);
      (which == liuh_camera::CAMERA_TOP ? pub_top : pub_bottom).publish(image);
    }

  } else if (fps_top) {
    ROS_INFO("Only using top camera!");
    liuh_camera::CameraDriver camera_top(liuh_camera::CAMERA_TOP, fps_top);

    while (ros::ok()) {
      pub_top.publish(camera_top.capture());
    }
  } else if (fps_bottom) {
    ROS_INFO("Only using bottom camera!");
    liuh_camera::CameraDriver camera_bottom(liuh_camera::CAMERA_BOTTOM, fps_bottom);

    while (ros::ok()) {
      pub_bottom.publish(camera_bottom.capture());
    }
  } else {
    while (ros::ok()) {
      ROS_WARN_THROTTLE(3, "Neither top nor bottom camera started!");
    }
  }
}
