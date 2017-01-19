#pragma once
#include "libmotioncapture/motioncapture.h"

#include <map>

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

  class MotionCapturePhasespaceImpl;

  class MotionCapturePhasespace
    : public MotionCapture
  {
  public:
    MotionCapturePhasespace(
      const std::string& hostname,
      size_t numMarkers,
      const std::map<size_t, std::pair<int, int> >& cfs);

    virtual ~MotionCapturePhasespace();

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
    MotionCapturePhasespaceImpl* pImpl;
  };

} // namespace libobjecttracker


