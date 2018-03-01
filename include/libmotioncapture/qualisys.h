#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {

  class MotionCaptureQualisysImpl;

  class MotionCaptureQualisys
    : public MotionCapture
  {
  public:
    MotionCaptureQualisys(
      const std::string& hostname,
      int basePort,
      bool enableObjects,
      bool enablePointcloud);

    virtual ~MotionCaptureQualisys();

    const std::string& version() const;

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();
    virtual void getObjects(std::vector<Object>& result) const;
    virtual void getObjectByName(
      const std::string& name,
      Object& result) const;
    virtual void getPointCloud(
      pcl::PointCloud<pcl::PointXYZ>::Ptr result) const;
    virtual void getLatency(
      std::vector<LatencyInfo>& result) const;

    virtual bool supportsObjectTracking() const;
    virtual bool supportsLatencyEstimate() const;
    virtual bool supportsPointCloud() const;

  private:
    MotionCaptureQualisysImpl* pImpl;
  };

} // namespace libobjecttracker


