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
#include <boost/serialization/serialization.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


using namespace std;
typedef std::vector<PointXYZ> CloudVector;

struct ObjectsData {

    std::vector<short> input;
    std::vector<double> dimensions1;
    std::vector<short> box1;
    std::vector<double> dimensions2;
    std::vector<short> box2;
    std::vector<double> dimensions3;
    std::vector<short> box3;
    std::vector<double> dimensions4;
    std::vector<short> box4;
    std::vector<double> dimensions5;
    std::vector<short> box5;

    template<typename archive> void serialize(archive& ar, const unsigned /*version*/) {
        ar & input;
        ar & dimensions1;
        ar & box1;
        ar & dimensions2;
        ar & box2;
        ar & dimensions3;
        ar & box3;
        ar & dimensions4;
        ar & box4;
        ar & dimensions5;
        ar & box5;
    }

};

class PCLViewer
{
  public:

//    string Run(char* ipaddr);
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
    double                                  dimensionX, dimensionY, dimensionZ;
    double                                  x1, x2, x3, x4, x5;
    double                                  y1, y2, y3, y4, y5;
    double                                  z1, z2, z3, z4, z5;
    string                                  X1, X2, X3, X4, X5;
    string                                  Y1, Y2, Y3, Y4, Y5;
    string                                  Z1, Z2, Z3, Z4, Z5;
    int                                     limitcluster;
    int                                     clusternumber;
    std::vector<short>                      outputcloud1, outputcloud2, outputcloud3, outputcloud4, outputcloud5;
};
