#ifndef SENSOR_H
#define SENSOR_H

#include "PointXYZ.h"
#include "VisionaryTData.h"
#include "VisionaryDataStream.h"
#include "VisionaryControl.h"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <config.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>

typedef pcl::PointXYZ           PointT;
typedef pcl::PointXYZI          PointI;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef std::vector<PointXYZ>   CloudVector;

class Sensor
{
    public:
        PointCloudI::Ptr CamStream        (char* ipAddress, unsigned short port);
        bool             TestConnection   (char* ipAddress, unsigned short port);
};

#endif // SENSOR_H
