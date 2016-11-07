#pragma once
#include "libmotioncapture/motioncapture.h"

//NatNet
#include <NatNetLinux/NatNet.h>

namespace libmotioncapture {
    class MotionCaptureOptitrackImpl;
    
    class MotionCaptureOptitrack : public MotionCapture{
    public:
        MotionCaptureOptitrack(std::string localIp, std::string serverIp);
        ~MotionCaptureOptitrack();
        
        const std::string & version() const;
        
        virtual void waitForNextFrame();
        virtual void getObjects(std::vector<Object>& result) const;
        //virtual void getObjectByName(const std::string & name, Object & result) const;
        
        virtual void getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr result) const;
        
        virtual void getLatency(std::vector<LatencyInfo> & result) const;
        
        virtual bool supportsObjectTracking() const;
        virtual bool supportsLatencyEstimate() const;
        virtual bool supportsPointCloud() const;
        
        Point3f axisMultiplier;
        Point3f axisOrder;
    private:
        MotionCaptureOptitrackImpl * pImpl;
    };
}

