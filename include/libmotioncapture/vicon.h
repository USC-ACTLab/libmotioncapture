#pragma once
#include "libmotioncapture/motioncapture.h"

// GetSegmentGlobalTranslation
// GetSegmentGlobalRotationQuaternion
// Connect
// IsConnected
// EnableSegmentData
// EnableUnlabeledMarkerData
// EnableMarkerData
// GetVersion
// GetFrame
// GetLatencyTotal,
// GetUnlabeledMarkerCount
// GetUnlabeledMarkerGlobalTranslation

namespace libmotioncapture {

  class MotionCaptureViconImpl;

  class MotionCaptureVicon
    : public MotionCapture
  {
  public:
    MotionCaptureVicon(
      const std::string& hostname,
      bool enableObjects,
      bool enablePointcloud);

    virtual ~MotionCaptureVicon();

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
    MotionCaptureViconImpl* pImpl;
  };

} // namespace libobjecttracker


