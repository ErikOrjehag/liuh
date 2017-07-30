#include <pthread.h>
#include <assert.h>
#include <iostream>
#include <string>
#include <sstream>

#include "liuh_camera/camera_driver.hpp"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>

namespace liuh_camera {

  class CameraNodelet : public nodelet::Nodelet {
  private:
    bool running_;
    boost::shared_ptr<boost::thread> thread_;
    ros::NodeHandle nh_;

    void run();

  public:
    CameraNodelet();
    ~CameraNodelet();
    virtual void onInit();
  };

  CameraNodelet::CameraNodelet()
    : running_(false) {

  }

  CameraNodelet::~CameraNodelet() {
    if (running_) {
      running_ = false;
      thread_->join();
    }
  }

  void CameraNodelet::onInit() {
    nh_ = getPrivateNodeHandle();
    running_ = true;
    thread_ = boost::shared_ptr< boost::thread >
      (new boost::thread(boost::bind(&CameraNodelet::run, this)));
  }

  void CameraNodelet::run() {
    image_transport::ImageTransport it(nh_);
    image_transport::Publisher pub = it.advertise("video", 1);
    std::string device_param;
    nh_.getParam("device", device_param);
    NODELET_INFO_STREAM("device param: " << device_param);
    CameraDevice device = device_param == "top" ? CAMERA_TOP : CAMERA_BOTTOM;
    std::string fps_param;
    nh_.getParam("fps", fps_param);
    NODELET_INFO_STREAM("fps param: " << fps_param);
    std::istringstream ss(fps_param);
    unsigned fps;
    ss >> fps;
    CameraDriver camera(device, fps);
    while (running_) {
      camera.release();
      SharedImgPtr image = camera.capture();
      pub.publish(image);
    }
  }

}

PLUGINLIB_EXPORT_CLASS(liuh_camera::CameraNodelet, nodelet::Nodelet)
