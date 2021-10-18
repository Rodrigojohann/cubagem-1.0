#pragma once
#include <QMainWindow>
#include <QFileDialog>
#include <QColor>
#include <QTimer>
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

struct ObjectsData {

    pcl::PointCloud<pcl::PointXYZ>::Ptr input;
    std::vector<double> dimensions1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr box1;
    std::vector<double> dimensions2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr box2;
    std::vector<double> dimensions3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr box3;
    std::vector<double> dimensions4;
    pcl::PointCloud<pcl::PointXYZ>::Ptr box4;
    std::vector<double> dimensions5;
    pcl::PointCloud<pcl::PointXYZ>::Ptr box5;

};

class PCLViewer
{
  public:

//    string Run(char* ipaddr);
    ObjectsData Run(char* ipaddr);

  protected:

    pcl::PointCloud<pcl::PointXYZ>::Ptr     inputcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     cloudnew;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     filteredcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     segmented_cloud;
    std::vector<pcl::PointIndices>          unsortedclusters;
    std::vector<pcl::PointIndices>          notorientedclusters;
    std::vector<pcl::PointIndices>          clusters;
    int                                     clustersize;
    double                                  dimensionX, dimensionY, dimensionZ;
    double                                  x1, x2, x3, x4, x5;
    double                                  y1, y2, y3, y4, y5;
    double                                  z1, z2, z3, z4, z5;
    string                                  X1, X2, X3, X4, X5;
    string                                  Y1, Y2, Y3, Y4, Y5;
    string                                  Z1, Z2, Z3, Z4, Z5;
    int                                     limitcluster;
    int                                     clusternumber;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     outputcloud1, outputcloud2, outputcloud3, outputcloud4, outputcloud5;
};
