#include "sensor.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudI::Ptr Sensor::CamStream(char* ipAddress, unsigned short port){
// var
    CloudVector                       pointCloud;
    boost::shared_ptr<VisionaryTData> pDataHandler;
    PointCloudI::Ptr                  cloud_raw (new PointCloudI);
    pcl::PassThrough<PointI>          passz;
    std::vector<uint16_t>             confidenceMap;
    std::vector<uint16_t>             intensityMap;
////
    // Generate Visionary instance
    pDataHandler = boost::make_shared<VisionaryTData>();
    VisionaryDataStream dataStream(pDataHandler, inet_addr(ipAddress), htons(port));
    VisionaryControl control(inet_addr(ipAddress), htons(2112));

    // Connect to devices data stream
    if (!dataStream.openConnection())
    {
        printf("Failed to open data stream connection to device.\n");
        return cloud_raw;
    }
    //-----------------------------------------------
    // Connect to devices control channel
    if (!control.openConnection())
    {
        printf("Failed to open control connection to device.\n");
        return cloud_raw;
    }
    control.stopAcquisition();
    control.startAcquisition();

    if (dataStream.getNextFrame())
    {
        // Convert data to a point cloud
        pDataHandler->generatePointCloud(pointCloud);
        confidenceMap = pDataHandler->getConfidenceMap();
        intensityMap = pDataHandler->getIntensityMap();
    }

    cloud_raw->points.resize(pointCloud.size());
    for(size_t i=0; i<cloud_raw->points.size(); ++i)
    {
        cloud_raw->points[i].x = pointCloud[i].x;
        cloud_raw->points[i].y = pointCloud[i].y;
        cloud_raw->points[i].z = pointCloud[i].z;
        cloud_raw->points[i].intensity = 20*log10(intensityMap[i]);
    }

    passz.setInputCloud(cloud_raw);
    passz.setFilterFieldName("z");
    passz.setFilterLimits(0, 5);
    passz.filter(*cloud_raw);

    control.stopAcquisition();
    control.closeConnection();
    dataStream.closeConnection();

    return cloud_raw;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Sensor::TestConnection(char* ipAddress, unsigned short port){
// var
    boost::shared_ptr<VisionaryTData> pDataHandler;
    VisionaryDataStream               dataStream (pDataHandler, inet_addr(ipAddress), htons(port));
    VisionaryControl                  control    (inet_addr(ipAddress), htons(2112));
////
    if (dataStream.openConnection() && control.openConnection())
    {
        control.closeConnection();
        dataStream.closeConnection();
        return true;
    }
    else
    {
        return false;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
