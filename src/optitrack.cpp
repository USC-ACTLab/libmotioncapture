#include "libmotioncapture/optitrack.h"

//NatNet
#include <NatNetLinux/NatNetClient.h>
#include "NatNetLinux/NatNet.h"

namespace libmotioncapture {

  class MotionCaptureOptitrackImpl{
  public:
    void getObjectByRigidbody(
      const RigidBody& rb,
      Object& result) const
      {
        std::stringstream sstr;
        sstr << rb.id();
        const std::string name_number = sstr.str();
        std::string name_cf = "cf";
        const std::string name = name_cf + name_number;

        auto const translation = rb.location();
        auto const quaternion = rb.orientation();

        if(rb.trackingValid()) {
            Eigen::Vector3f position(
              -translation.y,     
              translation.x,
              translation.z);

            Eigen::Quaternionf rotation(
              quaternion.qw,
              -quaternion.qy,
              quaternion.qx,
              quaternion.qz
              );

            result = Object(name, position, rotation);

        } else {
            result = Object(name);
        }
      } 

  public:
    NatNetClient client;
    std::string version;
    Point3f axisMultiplier;
    Point3f axisOrder;
  };

  MotionCaptureOptitrack::MotionCaptureOptitrack(
    const std::string& localIp,
    const std::string& serverIp)
  {
    pImpl = new MotionCaptureOptitrackImpl;

    pImpl->client.connect(localIp, serverIp);
    pImpl->version = pImpl->client.getVersionString();
    pImpl->axisMultiplier = Point3f(-1, 1, 1);//0.001,0.001,0.001);
    pImpl->axisOrder = Point3f(1,0,2);
  }

  const std::string & MotionCaptureOptitrack::version() const
  {
    return pImpl->version;
  }

  void MotionCaptureOptitrack::waitForNextFrame()
  {
    pImpl->client.update();
  }

  void MotionCaptureOptitrack::getObjects(
    std::vector<Object> & result) const
  {
    result.clear();
    size_t count = 0;
    
    if(pImpl->client.isNewFrameReady())
    {
      std::vector<RigidBody> const & rBodies = pImpl->client.getLastFrame().rigidBodies();
      count = rBodies.size();
      result.resize(count);
  
      for (size_t i = 0; i < count; ++i) {
        pImpl->getObjectByRigidbody(rBodies[i], result[i]);
      }
    }
  }

  void MotionCaptureOptitrack::getPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr result) const
  {
    result->clear();
    std::vector<Point3f> const & markers = pImpl->client.getLastFrame().unIdMarkers();
    size_t count = markers.size();
    for(size_t i=0;i<count;++i) {
        Point3f p;
        p.x = markers[i][pImpl->axisOrder.x] * pImpl->axisMultiplier.x;
        p.y = markers[i][pImpl->axisOrder.y] * pImpl->axisMultiplier.y;
        p.z = markers[i][pImpl->axisOrder.z] * pImpl->axisMultiplier.z;

        result->push_back(pcl::PointXYZ(p.x,p.y,p.z));
    }
  }

  void MotionCaptureOptitrack::getLatency(
    std::vector<libmotioncapture::LatencyInfo> & result) const
  {
    result.clear();
    std::string nn = "";
    double dd = pImpl->client.getLastFrame().latency();
    result.emplace_back(libmotioncapture::LatencyInfo(nn, dd));
  }

  MotionCaptureOptitrack::~MotionCaptureOptitrack()
  {
    delete pImpl;
  }

  bool MotionCaptureOptitrack::supportsObjectTracking() const
  {
    return false;
  }

  bool MotionCaptureOptitrack::supportsLatencyEstimate() const
  {
    return true;
  }

  bool MotionCaptureOptitrack::supportsPointCloud() const
  {
    return true;
  }
}

