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

using namespace std;
typedef std::vector<PointXYZ> CloudVector;

class PCLViewer
{
  public:

    void MainFrame();
    void ConnectDevice();
    bool firstCall;

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
    string                                  Vol1, Vol2, Vol3, Vol4, Vol5;
    string                                  VolStr, TotalStr;
    int                                     limitcluster;
    int                                     clusternumber;
    CloudVector                             camera_cloud;
    double                                  hullarea;
};
