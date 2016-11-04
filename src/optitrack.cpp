#include "libmotioncapture/optitrack.h"

//NatNet
#include <NatNetLinux/NatNetClient.h>

namespace libmotioncapture {
    
    class MotionCaptureOptitrackImpl{
    public:
        NatNetClient client;
        std::string version;
    };
    
    MotionCaptureOptitrack::MotionCaptureOptitrack(std::string localIp,
                                                   std::string serverIp){
        
        axisMultiplier = Point3f(0.001,0.001,0.001);
        axisOrder = Point3f(0,1,2);
        
        pImpl = new MotionCaptureOptitrackImpl;
        
        pImpl->client.connect(localIp, serverIp);
        pImpl->version = pImpl->client.getVersionString();
    }
    
    const std::string & MotionCaptureOptitrack::version() const{
        return pImpl->version;
    }
    
    void MotionCaptureOptitrack::waitForNextFrame(){
        pImpl->client.update();
    }
    
    /*void MotionCaptureOptitrack::getObjects(std::vector<libmotioncapture::Object> & result) const{
        result.clear();
        size_t count = 0;
        if(pImpl->client.isNewFrameReady()){
            std::vector<RigidBody> const & rBodies = pImpl->client.getLastFrame().rigidBodies();
            count = rBodies.size();
            result.resize(count);
            for(size_t i=0;i<count;++i){
                getObjectByRigidbody(rBodies[i],result[i]);
            }
        }
    }*/
    
    

    /*void MotionCaptureOptitrack::getObjectByRigidbody(const RigidBody & rb, libmotioncapture::Object & result) const{
        
        std::stringstream sstr;
        sstr<<rb.id();
        const std::string name = sstr.str();
        
        auto const translation = rb.location();
        auto const quaternion = rb.orientation();
        if(rb.trackingValid()){
            Eigen::Vector3f position(translation.x/1000.0,
                                     translation.y/1000.0,
                                     translation.z/1000.0);
            
            Eigen::Quaternionf rotation(quaternion.qw,
                                        quaternion.qx,
                                        quaternion.qy,
                                        quaternion.qz);
            
            result = Object(name, position, rotation);
        }else{
            result = Object(name);
        }
    }
    
    void MotionCaptureOptitrack::getObjectByName(const std::string & name, libmotioncapture::Object & result) const{
        std::vector<RigidBody> const & rBodies = pImpl->client.getLastFrame().rigidBodies();
        for(size_t i=0;i<rBodies.size();++i){
            std::stringstream sstr;
            sstr<<rBodies[i].id();
            const std::string ni = sstr.str();
            if(ni==name){
                //getObjectByRigidbody(rBodies[i],result);
            }
        }
    }*/
    
    void MotionCaptureOptitrack::getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr result) const{
        result->clear();
        std::vector<Point3f> const & markers = pImpl->client.getLastFrame().unIdMarkers();
        size_t count = markers.size();
        for(size_t i=0;i<count;++i){
            Point3f p;
            p.x = markers[i][axisOrder.x]*axisMultiplier.x;
            p.y = markers[i][axisOrder.y]*axisMultiplier.y;
            p.z = markers[i][axisOrder.z]*axisMultiplier.z;
            
            result->push_back(pcl::PointXYZ(p.x,p.y,p.z));
        }
    }
    
    void MotionCaptureOptitrack::getLatency(std::vector<libmotioncapture::LatencyInfo> & result) const{
        result.clear();
        
        result.clear();
	std::string nn = "";
	double dd = pImpl->client.getLastFrame().latency();
        result.emplace_back(libmotioncapture::LatencyInfo(nn, dd));
    }
    
    MotionCaptureOptitrack::~MotionCaptureOptitrack(){
        delete pImpl;
    }
    
    bool MotionCaptureOptitrack::supportsObjectTracking() const{
        return true;
    }
    
    bool MotionCaptureOptitrack::supportsLatencyEstimate() const{
        return true;
    }
    
    bool MotionCaptureOptitrack::supportsPointCloud() const{
        return true;
    }
}

