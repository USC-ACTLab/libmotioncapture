#include "libmotioncapture/qualisys.h"

// Qualisys
#include "RTProtocol.h"

#include <string>
#include <sstream>

namespace libmotioncapture {

  class MotionCaptureQualisysImpl
  {
  public:
    CRTProtocol poRTProtocol;
    CRTPacket*  pRTPacket;
    CRTPacket::EComponentType componentType;
    std::string version;
  };

  MotionCaptureQualisys::MotionCaptureQualisys(
    const std::string& hostname,
    int basePort,
    bool enableObjects,
    bool enablePointcloud)
  {
    pImpl = new MotionCaptureQualisysImpl;

    // Connecting ...
    if (!pImpl->poRTProtocol.Connect((char*)hostname.c_str(), basePort, 0, 1, 7)) {
      std::stringstream sstr;
      sstr << "Error connecting QTM on address: " << hostname << ":" << basePort;
      throw std::runtime_error(sstr.str());
    }
    pImpl->pRTPacket = pImpl->poRTProtocol.GetRTPacket();

    // Setting component flag
    pImpl->componentType = static_cast<CRTPacket::EComponentType>(0);
    if (enableObjects) {
      pImpl->componentType = static_cast<CRTPacket::EComponentType>(pImpl->componentType | CRTPacket::Component6dEuler);
    }
    if (enablePointcloud) {
      pImpl->componentType = static_cast<CRTPacket::EComponentType>(pImpl->componentType | CRTPacket::Component3dNoLabels);
    }

    // Get 6DOF settings
    bool dataAvailable;
    pImpl->poRTProtocol.Read6DOFSettings(dataAvailable);

    // TODO: enable UDP streaming of selected component for lower latency?

    // Getting version
    char qtmVersion[255];
    unsigned int major, minor;
    pImpl->poRTProtocol.GetQTMVersion(qtmVersion, 255);
    pImpl->poRTProtocol.GetVersion(major, minor);
    std::stringstream sstr;
    sstr << qtmVersion << " (Protocol: " << major << "." << minor <<")";
    pImpl->version  = sstr.str();
  }

  MotionCaptureQualisys::~MotionCaptureQualisys()
  {
    delete pImpl;
  }

  const std::string& MotionCaptureQualisys::version() const
  {
    return pImpl->version;
  }

  void MotionCaptureQualisys::waitForNextFrame()
  {
    CRTPacket::EPacketType eType;

    pImpl->poRTProtocol.GetCurrentFrame(pImpl->componentType);

    do {
      pImpl->poRTProtocol.ReceiveRTPacket(eType, true);
    } while(eType != CRTPacket::PacketData);
  }

  void MotionCaptureQualisys::getObjects(
    std::vector<Object>& result) const
  {
    float pos[3], rx, ry, rz;

    result.clear();
    size_t count = pImpl->pRTPacket->Get6DOFEulerBodyCount();

    for(size_t i = 0; i < count; ++i) {
      pImpl->pRTPacket->Get6DOFEulerBody(i, pos[0], pos[1], pos[2], rx, ry, rz);
      if (std::isnan(pos[0])) {
        result.push_back(Object(std::string(pImpl->poRTProtocol.Get6DOFBodyName(i))));
      } else {
        std::string name = std::string(pImpl->poRTProtocol.Get6DOFBodyName(i));

        Eigen::Vector3f position = Eigen::Vector3f(pos) / 1000.0;

        Eigen::Matrix3f rotation;
        rotation = Eigen::AngleAxisf((rx/180.0f)*M_PI, Eigen::Vector3f::UnitX())
                 * Eigen::AngleAxisf((ry/180.0f)*M_PI, Eigen::Vector3f::UnitY())
                 * Eigen::AngleAxisf((rz/180.0f)*M_PI, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf quaternion = Eigen::Quaternionf(rotation);

        result.push_back(Object(name, position, quaternion));
      }
    }
  }

  void MotionCaptureQualisys::getObjectByName(
    const std::string& name,
    Object& result) const
  {
    // Find object index
    size_t bodyCount = pImpl->pRTPacket->Get6DOFEulerBodyCount();
    size_t bodyId;
    for (bodyId = 0; bodyId < bodyCount; ++bodyId) {
      if (!strcmp(name.c_str(), pImpl->poRTProtocol.Get6DOFBodyName(bodyId))) {
        break;
      }
    }

    // If found, get object position
    if (bodyId < bodyCount) {
      float pos[3], rx, ry, rz;

      pImpl->pRTPacket->Get6DOFEulerBody(bodyId, pos[0], pos[1], pos[2], rx, ry, rz);

      if (std::isnan(pos[0])) {
        result = Object(name);
      } else {
        Eigen::Vector3f position = Eigen::Vector3f(pos) / 1000.0;

        Eigen::Matrix3f rotation;
        rotation = Eigen::AngleAxisf((rx/180.0f)*M_PI, Eigen::Vector3f::UnitX())
                 * Eigen::AngleAxisf((ry/180.0f)*M_PI, Eigen::Vector3f::UnitY())
                 * Eigen::AngleAxisf((rz/180.0f)*M_PI, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf quaternion = Eigen::Quaternionf(rotation);

        result = Object(name, position, quaternion);
      }
    } else {
      result = Object(name);
    }
  }

  void MotionCaptureQualisys::getPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr result) const
  {
    result->clear();
    size_t count = pImpl->pRTPacket->Get3DNoLabelsMarkerCount();
    for(size_t i = 0; i < count; ++i) {
      float x, y, z;
      uint nId;
      pImpl->pRTPacket->Get3DNoLabelsMarker(i, x, y, z, nId);
      result->push_back(pcl::PointXYZ(x / 1000.0,
                                      y / 1000.0,
                                      z / 1000.0));
    }
  }

  void MotionCaptureQualisys::getLatency(
    std::vector<LatencyInfo>& result) const
  {
    result.clear();
  }

  bool MotionCaptureQualisys::supportsObjectTracking() const
  {
    return true;
  }

  bool MotionCaptureQualisys::supportsLatencyEstimate() const
  {
    return false;
  }

  bool MotionCaptureQualisys::supportsPointCloud() const
  {
    return true;
  }
}
