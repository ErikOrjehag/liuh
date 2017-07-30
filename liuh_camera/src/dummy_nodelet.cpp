#include <pthread.h>
#include <assert.h>
#include <math.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>

namespace liuh_camera {

  class DummyNodelet : public nodelet::Nodelet {
  private:
    ros::Time last_image_time_;
    image_transport::Subscriber sub;

  public:
    DummyNodelet();
    ~DummyNodelet();
    virtual void onInit();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  };

  DummyNodelet::DummyNodelet() {}

  DummyNodelet::~DummyNodelet() {}

  void DummyNodelet::onInit() {
    ros::NodeHandle nh = getPrivateNodeHandle();
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("/liuh_camera/video", 1, &DummyNodelet::imageCallback, this);
    last_image_time_ = ros::Time::now();
  }

  void DummyNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    float hz = 1.0 / (ros::Time::now() - last_image_time_).toSec();
    last_image_time_ = ros::Time::now();
    float delay = (ros::Time::now() - msg->header.stamp).toSec() * 1000;

    int sum = 0;
    for (int i = 0; i < msg->data.size(); i += msg->data.size() / 10) {
      sum += msg->data[i];
    }

    NODELET_INFO_THROTTLE(1, "Image: %.1f hz, delay: %d ms, sum: %d", hz, (int)round(delay), sum);
  }
}

PLUGINLIB_EXPORT_CLASS(liuh_camera::DummyNodelet, nodelet::Nodelet)
