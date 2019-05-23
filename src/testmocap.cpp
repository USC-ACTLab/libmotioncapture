#include "libmotioncapture/testmocap.h"

#include <thread>

#include <pcl/common/io.h>

namespace libmotioncapture {

  class MotionCaptureTestImpl{
  public:
    MotionCaptureTestImpl()
    {
    }

  public:
    float dt;
    std::vector<Object> objects;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
  };

  MotionCaptureTest::MotionCaptureTest(
    float dt,
    const std::vector<Object>& objects,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
  {
    pImpl = new MotionCaptureTestImpl;
    pImpl->dt = dt;
    pImpl->objects = objects;
    pImpl->pointCloud = pointCloud;
  }

  void MotionCaptureTest::waitForNextFrame()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(pImpl->dt * 1000)));
  }

  void MotionCaptureTest::getObjects(
    std::vector<Object> & result) const
  {
    result = pImpl->objects;
  }

  void MotionCaptureTest::getPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr result) const
  {
    pcl::copyPointCloud(*pImpl->pointCloud, *result);
  }

  void MotionCaptureTest::getLatency(
    std::vector<libmotioncapture::LatencyInfo> & result) const
  {
    result.clear();
  }

  MotionCaptureTest::~MotionCaptureTest()
  {
    delete pImpl;
  }

  bool MotionCaptureTest::supportsObjectTracking() const
  {
    return true;
  }

  bool MotionCaptureTest::supportsLatencyEstimate() const
  {
    return false;
  }

  bool MotionCaptureTest::supportsPointCloud() const
  {
    return true;
  }
}

