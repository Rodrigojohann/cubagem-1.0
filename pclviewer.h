#pragma once
#include <sensor.h>
#include <controller.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <thread>
#define PORT 2114
#include <sstream>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <objectsdata.h>


using namespace std;
typedef std::vector<PointXYZ> CloudVector;

class PCLViewer
{
  public:

    ObjectsData Run(char* ipaddr);
    std::vector<short> ConvertCloudtoVector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  protected:

    pcl::PointCloud<pcl::PointXYZ>::Ptr     inputcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     cloudnew;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     filteredcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     segmented_cloud;
    std::vector<pcl::PointIndices>          unsortedclusters;
    std::vector<pcl::PointIndices>          notorientedclusters;
    std::vector<pcl::PointIndices>          clusters;
    int                                     clustersize;
    float                                  dimensionX, dimensionY, dimensionZ;
    float                                  x1, x2, x3, x4, x5;
    float                                  y1, y2, y3, y4, y5;
    float                                  z1, z2, z3, z4, z5;
    int                                     limitcluster;
    int                                     clusternumber;
    std::vector<short>                      outputcloud1, outputcloud2, outputcloud3, outputcloud4, outputcloud5;
    int                                    Nsamples = 5;
};
