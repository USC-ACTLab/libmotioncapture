#include "libmotioncapture/phasespace.h"

// Phacespace
#include "phasespace_sdk/owl.h"

#include <thread>

#define INIT_FLAGS 0

namespace libmotioncapture {

  class MotionCapturePhasespaceImpl
  {
  public:
    std::vector<OWLMarker> markers;
    int tracker;
    std::map<size_t, std::pair<int, int> > cfs;
  };

  // cfs maps from cfid->(phasespaceMarkerCenter, phasespaceMarkerFront)
  MotionCapturePhasespace::MotionCapturePhasespace(
    const std::string& hostname,
    size_t numMarkers,
    const std::map<size_t, std::pair<int, int> >& cfs)
  {
    pImpl = new MotionCapturePhasespaceImpl;
    pImpl->cfs = cfs;

    int err = owlInit(hostname.c_str(), INIT_FLAGS);

    if(err < 0) {
      std::stringstream sstr;
      sstr << "Error in phasespace setup: " << err;
      throw std::runtime_error(sstr.str());
    }

    // create tracker 0
    pImpl->tracker = 0;
    owlTrackeri(pImpl->tracker, OWL_CREATE, OWL_POINT_TRACKER);

    // set markers
    pImpl->markers.resize(numMarkers);
    for(int i = 0; i < numMarkers; i++) {
      owlMarkeri(MARKER(pImpl->tracker, i), OWL_SET_LED, i);
    }

    // activate tracker
    owlTracker(pImpl->tracker, OWL_ENABLE);

    // flush requests and check for errors
    if(!owlGetStatus())
    {
      //owl_print_error("error in point tracker setup", owlGetError());
      //throw std::runtime_error("Error in phasespace setup");

	std::stringstream sstr;
      sstr << "Error in phasespace setup: " << owlGetError();
      throw std::runtime_error(sstr.str());
    }

    std::vector<OWLCamera> dummyCameras(100);
    int numCameras = owlGetCameras(dummyCameras.data(), dummyCameras.size());
    std::cout << "numCameras: " << numCameras << std::endl;

    // set default frequency
    owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY);
  
    // start streaming
    owlSetInteger(OWL_STREAMING, OWL_ENABLE);
  }

  MotionCapturePhasespace::~MotionCapturePhasespace()
  {
    owlDone();
    delete pImpl;
  }

  void MotionCapturePhasespace::waitForNextFrame()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    int n = 0;
    int err;
    do {
      // get some markers
      n = owlGetMarkers(pImpl->markers.data(), pImpl->markers.size());
        
      // check for error
      if((err = owlGetError()) != OWL_NO_ERROR)
	    {
	      throw std::runtime_error("Error in phasespace query");
	    }
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    } while (n == 0);
    // now loop until 0
    std::vector<OWLMarker> temp1(pImpl->markers.size());
    //std::vector<OWLMarker> temp2(pImpl->markers.size());
    size_t i = 0;
    do {
      // get some markers
	if (i %2 == 0) {
      n = owlGetMarkers(temp1.data(), pImpl->markers.size());
	} else {
	n = owlGetMarkers(pImpl->markers.data(), pImpl->markers.size());
	}
        
      // check for error
      if((err = owlGetError()) != OWL_NO_ERROR)
	    {
	      throw std::runtime_error("Error in phasespace query");
	    }
      std::this_thread::sleep_for(std::chrono::microseconds(1));
      ++i;
    } while (n != 0);	
	if (i%2 == 1) {
	pImpl->markers = temp1;
	}
  }

  void MotionCapturePhasespace::getObjects(
    std::vector<Object>& result) const
  {
    result.clear();
    for (const auto& cf : pImpl->cfs) {
      size_t id = cf.first;  
      int centerLed = cf.second.first;
      int frontLed = cf.second.second;
      if (   pImpl->markers[centerLed].cond > 0
          && pImpl->markers[frontLed].cond > 0) {
        double opposite = pImpl->markers[frontLed].x - pImpl->markers[centerLed].x;
        double adjacent = pImpl->markers[frontLed].z - pImpl->markers[centerLed].z;
        double theta = atan2(opposite, adjacent);

        Eigen::Vector3f position(
          pImpl->markers[centerLed].z / 1000.0,
          pImpl->markers[centerLed].x / 1000.0,
          pImpl->markers[centerLed].y / 1000.0);

        Eigen::Quaternionf rotation(
          Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
          Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

        result.push_back(Object("cf" + std::to_string(id), position, rotation));
      }
    }
  }

  void MotionCapturePhasespace::getObjectByName(
    const std::string& name,
    Object& result) const
  {
    result = Object(name);
    for (const auto& cf : pImpl->cfs) {
      size_t id = cf.first;
      if (name == "cf" + std::to_string(id)) {      
        int centerLed = cf.second.first;
        int frontLed = cf.second.second;
        if (   pImpl->markers[centerLed].cond > 0
            && pImpl->markers[frontLed].cond > 0) {
          double opposite = pImpl->markers[frontLed].x - pImpl->markers[centerLed].x;
          double adjacent = pImpl->markers[frontLed].z - pImpl->markers[centerLed].z;
          double theta = atan2(opposite, adjacent);

          Eigen::Vector3f position(
            pImpl->markers[centerLed].z / 1000.0,
            pImpl->markers[centerLed].x / 1000.0,
            pImpl->markers[centerLed].y / 1000.0);

          Eigen::Quaternionf rotation(
            Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

          result = Object(name, position, rotation);
          return;
        }
      } 
    }
  }

  void MotionCapturePhasespace::getPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr result) const
  {
    result->clear();
    for(size_t i = 0; i < pImpl->markers.size(); i++) {
	    if(pImpl->markers[i].cond > 0) {
        result->push_back(pcl::PointXYZ(
        pImpl->markers[i].z / 1000.0,
        pImpl->markers[i].x / 1000.0,
        pImpl->markers[i].y / 1000.0));
      }
    }
  }

  void MotionCapturePhasespace::getLatency(
    std::vector<LatencyInfo>& result) const
  {
    result.clear();
  }

  bool MotionCapturePhasespace::supportsObjectTracking() const
  {
    return true;
  }

  bool MotionCapturePhasespace::supportsLatencyEstimate() const
  {
    return true;
  }

  bool MotionCapturePhasespace::supportsPointCloud() const
  {
    return true;
  }
}
