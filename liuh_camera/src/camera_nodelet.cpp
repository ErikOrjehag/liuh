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
    
    image_transport::Publisher pub_top;
    image_transport::Publisher pub_bottom;

    int fps_top;
    nh_.getParam("fps_top", fps_top);
    NODELET_INFO("fps_top param: %d", fps_top);

    int fps_bottom;
    nh_.getParam("fps_bottom", fps_bottom);
    NODELET_INFO("fps_bottom param: %d", fps_bottom);

    if (fps_top) {
      pub_top = it.advertise("top", 1);
    }

    if (fps_bottom) {
      pub_bottom = it.advertise("bottom", 1);
    }

    if (fps_top && fps_bottom) {
      NODELET_INFO("Using both cameras!");
      liuh_camera::CameraDriver camera_top(liuh_camera::CAMERA_TOP, fps_top);
      liuh_camera::CameraDriver camera_bottom(liuh_camera::CAMERA_BOTTOM, fps_bottom);

      while (running_) {
        liuh_camera::CameraDevice which;
        liuh_camera::SharedImgPtr image = liuh_camera::CameraDriver::capture(camera_top, camera_bottom, which);
        (which == liuh_camera::CAMERA_TOP ? pub_top : pub_bottom).publish(image);
      }

    } else if (fps_top) {
      NODELET_INFO("Only using top camera!");
      liuh_camera::CameraDriver camera_top(liuh_camera::CAMERA_TOP, fps_top);

      while (running_) {
        pub_top.publish(camera_top.capture());
      }
    } else if (fps_bottom) {
      NODELET_INFO("Only using bottom camera!");
      liuh_camera::CameraDriver camera_bottom(liuh_camera::CAMERA_BOTTOM, fps_bottom);

      while (running_) {
        pub_bottom.publish(camera_bottom.capture());
      }
    } else {
      while (running_) {
        NODELET_WARN_THROTTLE(3, "Neither top nor bottom camera started!");
      }
    }
  }
}

PLUGINLIB_EXPORT_CLASS(liuh_camera::CameraNodelet, nodelet::Nodelet)
