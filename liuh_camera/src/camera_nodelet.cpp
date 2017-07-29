#include <pthread.h>
#include <assert.h>

#include "liuh_camera/camera_driver.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/thread.hpp>

namespace liuh {

  class CameraNodelet : public nodelet::Nodelet {
  private:
    volatile bool running_;
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
    image_transport::Publisher pub = it.advertise("/liuh_camera/video", 1);
    CameraDriver camera;
    while (running_) {
      camera.release();
      liuh::SharedImgPtr image = camera.capture();
      pub.publish(image);
    }
  }

}

PLUGINLIB_EXPORT_CLASS(liuh::CameraNodelet, nodelet::Nodelet)
