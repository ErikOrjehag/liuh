#include "liuh_camera/camera_driver.hpp"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>

namespace liuh {
  CameraDriver::CameraDriver() {
    captured_ = false;
    initOpenVideoDevice();
    initSetVideoFormat();
    setFramesPerSecond(5);
    initRequestAndMapBuffers();
    initQueueAllBuffers();
    initDefaultControlSettings();
    startCapturing();
  }

  CameraDriver::~CameraDriver() {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int streamoff_res = ioctl(fd_, VIDIOC_STREAMOFF, &type);
    assert(streamoff_res != -1);

    for (unsigned i = 0; i < FRAME_BUFFER_COUNT_; ++i) {
      munmap(mem_[i], mem_length_[i]);
    }

    close(fd_);
    free(buf_);
  }

  SharedImgPtr CameraDriver::capture() {
    struct pollfd pollfd = { fd_, POLLIN | POLLPRI, 0 };
    int timeout = 1000;
    int polled = poll(&pollfd, 1, timeout);

    if (polled < 0) {
      ROS_ERROR("Cannot poll camera image. Reason: %s", strerror(errno));
      assert(false);
    }
    else if (polled == 0)
    {
      ROS_ERROR("Poll camera image timed out after %d ms", timeout);
      assert(false);
    }

    if (!(pollfd.revents & POLLIN)) {
      ROS_ERROR("Unexpected camera poll results: %d", pollfd.revents);
      assert(false);
    }

    int dqbuf_res = ioctl(fd_, VIDIOC_DQBUF, buf_);
    if (dqbuf_res == -1) {
      ROS_ERROR("Could not dequeue camera buffer. Reason: %s", strerror(errno));
      assert(false);
    }

    captured_ = true;

    unsigned long long timestamp = static_cast<unsigned long long>(buf_->timestamp.tv_sec) * 1000000ll + buf_->timestamp.tv_usec;
    unsigned char* image = static_cast<unsigned char*>(mem_[buf_->index]);

    //ROS_INFO("timestamp: %llu", timestamp);

    sensor_msgs::Image* msg = new sensor_msgs::Image();
    msg->header.frame_id = "CameraTop_optical_frame";
    //msg->header.stamp.sec = ;
    //msg->header.stamp.nsec = ;
    msg->width = WIDTH_;
    msg->height = HEIGHT_;
    msg->encoding = sensor_msgs::image_encodings::YUV422;
    msg->is_bigendian = 0;
    msg->step = STEP_;
    msg->data.resize(SIZE_);
    memcpy(msg->data.data(), image, SIZE_);

    return SharedImgPtr(msg);
  }

  void CameraDriver::release() {
    if (captured_) {
      captured_ = false;
      int qbuf_res = ioctl(fd_, VIDIOC_QBUF, buf_);
      if (qbuf_res == -1) {
        ROS_ERROR("Could not queue camera buffer. Reason: %s", strerror(errno));
        assert(false);
      }
    }
  }

  void CameraDriver::setFramesPerSecond(unsigned fps)
  {
    struct v4l2_streamparm param;
    memset(&param, 0, sizeof(struct v4l2_streamparm));
    param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int g_res = ioctl(fd_, VIDIOC_G_PARM, &param);
    assert(g_res != -1);
    param.parm.capture.timeperframe.numerator = 1;
    param.parm.capture.timeperframe.denominator = fps;
    int s_res = ioctl(fd_, VIDIOC_S_PARM, &param);
    assert(s_res != -1);
  }

  void CameraDriver::initOpenVideoDevice()
  {
    const char* device = "/dev/video0";
    fd_ = open(device, O_RDWR | O_NONBLOCK);
    assert(fd_ != -1);
  }

  void CameraDriver::initSetVideoFormat()
  {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH_;
    fmt.fmt.pix.height = HEIGHT_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    int fmt_res = ioctl(fd_, VIDIOC_S_FMT, &fmt);
    if (fmt_res == -1) {
      ROS_ERROR("Cannot set camera video format. Reason: %s", strerror(errno));
      assert(false);
    }

    assert(fmt.fmt.pix.sizeimage == SIZE_);
  }

  void CameraDriver::initRequestAndMapBuffers()
  {
    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof(struct v4l2_requestbuffers));
    rb.count = FRAME_BUFFER_COUNT_;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    int reqbuf_res = ioctl(fd_, VIDIOC_REQBUFS, &rb);
    assert(reqbuf_res != -1);
    assert(rb.count == FRAME_BUFFER_COUNT_);

    buf_ = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
    for (unsigned i = 0; i < FRAME_BUFFER_COUNT_; ++i)
    {
      buf_->index = i;
      buf_->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf_->memory = V4L2_MEMORY_MMAP;
      int query_res = ioctl(fd_, VIDIOC_QUERYBUF, buf_);
      assert(query_res != -1);
      mem_length_[i] = buf_->length;
      mem_[i] = mmap(0, buf_->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf_->m.offset);
      assert(mem_[i] != MAP_FAILED);
    }
  }

  void CameraDriver::initQueueAllBuffers()
  {
    for (unsigned i = 0; i < FRAME_BUFFER_COUNT_; ++i)
    {
      buf_->index = i;
      buf_->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf_->memory = V4L2_MEMORY_MMAP;
      int qbuf_res = ioctl(fd_, VIDIOC_QBUF, buf_);
      assert(qbuf_res != -1);
    }
  }

  void CameraDriver::initDefaultControlSettings()
  {

  }

  void CameraDriver::startCapturing()
  {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int streamon_res = ioctl(fd_, VIDIOC_STREAMON, &type);
    assert(streamon_res != -1);
  }
}