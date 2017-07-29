#pragma once

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <boost/shared_ptr.hpp>

namespace liuh
{
  typedef boost::shared_ptr< const sensor_msgs::Image > SharedImgPtr;

  class CameraDriver
  {
  private:
    int fd_;
    bool captured_;
    struct v4l2_buffer* buf_;
    static const unsigned FRAME_BUFFER_COUNT_ = 3;
    void* mem_[FRAME_BUFFER_COUNT_];
    int mem_length_[FRAME_BUFFER_COUNT_];

    void setFramesPerSecond(unsigned fps);

    void initOpenVideoDevice();
    void initSetVideoFormat();
    void initRequestAndMapBuffers();
    void initQueueAllBuffers();
    void initDefaultControlSettings();
    void startCapturing();

  public:
    static const unsigned WIDTH_ = 1280;
    static const unsigned HEIGHT_ = 960;
    static const unsigned LAYERS_ = 2;
    static const unsigned STEP_ = WIDTH_ * LAYERS_;
    static const unsigned SIZE_ = WIDTH_ * HEIGHT_ * LAYERS_;

    CameraDriver();
    ~CameraDriver();
    SharedImgPtr capture();
    void release();
  };
}
