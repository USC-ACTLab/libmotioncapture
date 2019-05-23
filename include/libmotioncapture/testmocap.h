#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {
  class MotionCaptureTestImpl;

  class MotionCaptureTest : public MotionCapture{
  public:
    MotionCaptureTest(
      float dt,
      const std::vector<Object>& objects,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

    virtual ~MotionCaptureTest();

    virtual void waitForNextFrame();
    virtual void getObjects(
      std::vector<Object>& result) const;

    virtual void getPointCloud(
      pcl::PointCloud<pcl::PointXYZ>::Ptr result) const;
    virtual void getLatency(
      std::vector<LatencyInfo>& result) const;

    virtual bool supportsObjectTracking() const;
    virtual bool supportsLatencyEstimate() const;
    virtual bool supportsPointCloud() const;

  private:
    MotionCaptureTestImpl * pImpl;
  };
}

