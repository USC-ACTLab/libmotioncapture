#include "libmotioncapture/vicon.h"

// VICON
#include "vicon_sdk/DataStreamClient.h"

using namespace ViconDataStreamSDK::CPP;

namespace libmotioncapture {

  class MotionCaptureViconImpl
  {
  public:
    Client client;
    std::string version;
  };

  MotionCaptureVicon::MotionCaptureVicon(
    const std::string& hostname,
    bool enableObjects,
    bool enablePointcloud)
  {
    pImpl = new MotionCaptureViconImpl;

    // Try connecting...
    while (!pImpl->client.IsConnected().Connected) {
      pImpl->client.Connect(hostname);
    }

    if (enableObjects) {
      pImpl->client.EnableSegmentData();
    }
    if (enablePointcloud) {
      pImpl->client.EnableUnlabeledMarkerData();
    }

    // This is the lowest latency option
    pImpl->client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

    // Set the global up axis
    pImpl->client.SetAxisMapping(Direction::Forward,
                          Direction::Left,
                          Direction::Up); // Z-up

    // Discover the version number
    Output_GetVersion version = pImpl->client.GetVersion();
    std::stringstream sstr;
    sstr << version.Major << "." << version.Minor << "." << version.Point;
    pImpl->version = sstr.str();
  }

  MotionCaptureVicon::~MotionCaptureVicon()
  {
    delete pImpl;
  }

  const std::string& MotionCaptureVicon::version() const
  {
    return pImpl->version;
  }

  void MotionCaptureVicon::waitForNextFrame()
  {
    while (pImpl->client.GetFrame().Result != Result::Success) {
    }
  }

  void MotionCaptureVicon::getObjects(
    std::vector<Object>& result) const
  {
    result.clear();
    size_t count = pImpl->client.GetSubjectCount().SubjectCount;
    result.resize(count);
    for (size_t i = 0; i < count; ++i) {
      const std::string name = pImpl->client.GetSubjectName(i).SubjectName;
      getObjectByName(name, result[i]);
    }
  }

  void MotionCaptureVicon::getObjectByName(
    const std::string& name,
    Object& result) const
  {
    auto const translation = pImpl->client.GetSegmentGlobalTranslation(name, name);
    auto const quaternion = pImpl->client.GetSegmentGlobalRotationQuaternion(name, name);
    if (   translation.Result == Result::Success
        && quaternion.Result == Result::Success
        && !translation.Occluded
        && !quaternion.Occluded) {

      Eigen::Vector3f position(
        translation.Translation[0] / 1000.0,
        translation.Translation[1] / 1000.0,
        translation.Translation[2] / 1000.0);

      Eigen::Quaternionf rotation(
        quaternion.Rotation[3], // w
        quaternion.Rotation[0], // x
        quaternion.Rotation[1], // y
        quaternion.Rotation[2]  // z
        );

      result = Object(name, position, rotation);
    } else {
      result = Object(name);
    }
  }

  void MotionCaptureVicon::getPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr result) const
  {
    result->clear();
    size_t count = pImpl->client.GetUnlabeledMarkerCount().MarkerCount;
    for(size_t i = 0; i < count; ++i) {
      Output_GetUnlabeledMarkerGlobalTranslation translation =
        pImpl->client.GetUnlabeledMarkerGlobalTranslation(i);
      result->push_back(pcl::PointXYZ(
        translation.Translation[0] / 1000.0,
        translation.Translation[1] / 1000.0,
        translation.Translation[2] / 1000.0));
    }
  }

  void MotionCaptureVicon::getLatency(
    std::vector<LatencyInfo>& result) const
  {
    result.clear();
    size_t latencyCount = pImpl->client.GetLatencySampleCount().Count;
    for(size_t i = 0; i < latencyCount; ++i) {
      std::string sampleName  = pImpl->client.GetLatencySampleName(i).Name;
      double      sampleValue = pImpl->client.GetLatencySampleValue(sampleName).Value;
      result.emplace_back(LatencyInfo(sampleName, sampleValue));
    }
  }

  bool MotionCaptureVicon::supportsObjectTracking() const
  {
    return true;
  }

  bool MotionCaptureVicon::supportsLatencyEstimate() const
  {
    return true;
  }

  bool MotionCaptureVicon::supportsPointCloud() const
  {
    return true;
  }
}
