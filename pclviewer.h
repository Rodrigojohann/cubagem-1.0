#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <sensor.h>
#include <controller.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <thread>
#define IP (char*)"192.168.140.2"
#define PORT 2114
#include <sstream>

using namespace std;
typedef std::vector<PointXYZ> CloudVector;

class PCLViewer
{
  public:

    string Run(char* ipaddr);

  protected:

    pcl::PointCloud<pcl::PointXYZ>::Ptr     inputcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     cloudnew;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coloredinput;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     filteredcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     segmented_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coloredcloud;
    std::vector<pcl::PointIndices>          unsortedclusters;
    std::vector<pcl::PointIndices>          notorientedclusters;
    std::vector<pcl::PointIndices>          clusters;
    int                                     cloudcolor[5][3] = {{0, 0, 255}, {0, 220, 0}, {255, 0, 0}, {200, 200, 0}, {255, 0, 255}};
    int                                     clustersize;
    double                                  dimensionX, dimensionY, dimensionZ;
    double                                  objvolume, totalvolume;
    double                                  volumemean;
    double                                  mean1, mean2, mean3, mean4, mean5;
    double                                  x1, x2, x3, x4, x5;
    double                                  y1, y2, y3, y4, y5;
    double                                  z1, z2, z3, z4, z5;

    string                                  X1, X2, X3, X4, X5;
    string                                  Y1, Y2, Y3, Y4, Y5;
    string                                  Z1, Z2, Z3, Z4, Z5;

    int                                     limitcluster;
    int                                     clusternumber;
    CloudVector                             camera_cloud;
    double                                  hullarea;
    std::vector<std::string>                outputarray;
    string                                  outputstring;
};
