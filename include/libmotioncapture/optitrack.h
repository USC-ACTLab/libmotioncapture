#pragma once
#include "libmotioncapture/motioncapture.h"
#include "NatNetLinux/NatNet.h"

namespace libmotioncapture {
  class MotionCaptureOptitrackImpl;

  class MotionCaptureOptitrack : public MotionCapture{
  public:
    MotionCaptureOptitrack(
      const std::string& localIp,
      const std::string& serverIp);

    virtual ~MotionCaptureOptitrack();

    const std::string& version() const;

    virtual void waitForNextFrame();
    virtual void getObjects(
      std::vector<Object>& result) const;
    virtual void getObjectByRigidbody(
      const RigidBody& rb,
      Object& result) const;

    virtual void getPointCloud(
      pcl::PointCloud<pcl::PointXYZ>::Ptr result) const;
    virtual void getLatency(
      std::vector<LatencyInfo>& result) const;

    virtual bool supportsObjectTracking() const;
    virtual bool supportsLatencyEstimate() const;
    virtual bool supportsPointCloud() const;

  private:
    MotionCaptureOptitrackImpl * pImpl;
  };
}

