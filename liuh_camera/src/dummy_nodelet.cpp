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
    ros::Time last_bottom_time_;
    ros::Time last_top_time_;
    image_transport::Subscriber sub_bottom_;
    image_transport::Subscriber sub_top_;

  public:
    DummyNodelet();
    ~DummyNodelet();
    virtual void onInit();
    void topCallback(const sensor_msgs::ImageConstPtr& msg);
    void bottomCallback(const sensor_msgs::ImageConstPtr& msg);
  };

  DummyNodelet::DummyNodelet() {}

  DummyNodelet::~DummyNodelet() {}

  void DummyNodelet::onInit() {
    ros::NodeHandle nh = getPrivateNodeHandle();
    image_transport::ImageTransport it(nh);
    sub_top_ = it.subscribe("/liuh_camera/top", 1, &DummyNodelet::topCallback, this);
    sub_bottom_ = it.subscribe("/liuh_camera/bottom", 1, &DummyNodelet::bottomCallback, this);
    last_bottom_time_ = ros::Time::now();
    last_top_time_ = ros::Time::now();
  }

  void DummyNodelet::bottomCallback(const sensor_msgs::ImageConstPtr& msg) {
    float hz = 1.0 / (ros::Time::now() - last_bottom_time_).toSec();
    last_bottom_time_ = ros::Time::now();
    float delay = (ros::Time::now() - msg->header.stamp).toSec() * 1000;

    int sum = 0;
    for (int i = 0; i < msg->data.size(); i += msg->data.size() / 10) {
      sum += msg->data[i];
    }

    NODELET_INFO_THROTTLE(1, "Bottom: %.1f hz, delay: %d ms, sum: %d", hz, (int)round(delay), sum);
  }

  void DummyNodelet::topCallback(const sensor_msgs::ImageConstPtr& msg) {
    float hz = 1.0 / (ros::Time::now() - last_top_time_).toSec();
    last_top_time_ = ros::Time::now();
    float delay = (ros::Time::now() - msg->header.stamp).toSec() * 1000;

    int sum = 0;
    for (int i = 0; i < msg->data.size(); i += msg->data.size() / 10) {
      sum += msg->data[i];
    }

    NODELET_INFO_THROTTLE(1, "Top: %.1f hz, delay: %d ms, sum: %d", hz, (int)round(delay), sum);
  }
}

PLUGINLIB_EXPORT_CLASS(liuh_camera::DummyNodelet, nodelet::Nodelet)
