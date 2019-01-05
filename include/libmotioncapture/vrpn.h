#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {

  class MotionCaptureVrpnImpl;

  class MotionCaptureVrpn
    : public MotionCapture
  {
  public:
    MotionCaptureVrpn(
      const std::string& hostname,
      int updateFrequency = 100);

    virtual ~MotionCaptureVrpn();

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
    MotionCaptureVrpnImpl* pImpl;
  };

} // namespace libobjecttracker


