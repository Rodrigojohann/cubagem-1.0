#pragma once
#include <QMainWindow>
#include <QFileDialog>
#include <QColor>
#include <QTimer>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <sensor.h>
#include <processor.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <thread>
#include <math.h>
#include <iomanip>
#include <sstream>
#include <dlib/dlib/svm.h>
#include <pcl/io/pcd_io.h>
#include <curl/curl.h>

using namespace std;
using namespace dlib;
typedef pcl::PointXYZ                      PointT;
typedef pcl::PointXYZI                     PointI;
typedef pcl::PointCloud<PointT>            PointCloudT;
typedef pcl::PointCloud<PointI>            PointCloudI;
typedef std::vector<PointXYZ>              CloudVector;
typedef pcl::PointCloud<pcl::PointXYZRGBA> ColoredCloudT;
typedef matrix<double, 4, 1>               sample_type;
typedef radial_basis_kernel<sample_type>   kernel_type;

class Executer
{
  public:

    string        Run(char* ipaddr);
    void          Clean();
    bool          firstCall;
    void          ConnectTDC(char *inputurl);
    void          SendJSON(int count, double volume, bool connection, double ymin, double ymax, double xmin, double xmax, double height);
    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);

  protected:

    PointCloudI::Ptr                                     cloudnew;
    PointCloudI::Ptr                                     cloudundistorted;
    PointCloudI::Ptr                                     cloud_preprocessed;
    PointCloudI::Ptr                                     filteredcloud;
    PointCloudT::Ptr                                     cloud_hull;
    PointCloudI::Ptr                                     cloud_template;
    std::vector<pcl::Vertices>                           polygons;
    ColoredCloudT::Ptr                                   coloredinput;
    ColoredCloudT::Ptr                                   coloredcloud;
    std::vector<pcl::Vertices>                           hullpolygons;
    std::vector<pcl::PointIndices>                       notorientedclusters;
    std::vector<PointCloudT::Ptr>                        clusters;
    int                                                  cloudcolor[3] = {0, 0, 255};
    int                                                  clustersize;
    int                                                  limitcluster;
    int                                                  stringprecision;
    int                                                  numberofboxes;
    string                                               numberofboxes_str;
    double                                               dimensionX, dimensionY, dimensionZ;
    double                                               minZ;
    double                                               objvolume, totalvolume;
    double                                               volumemean;
    double                                               hullarea;
    double                                               surfacearea;
    double                                               sumZ, meanZ;
    double                                               sumsizes;
    double                                               palletarea;
    double                                               minpalletarea;
    string                                               TotalStr;
    double                                               min_intensity, max_intensity;
    std::vector<double>                                  featuresvector;
    std::vector<double>                                  sumfeaturesvector;
    std::vector<std::vector<double>>                     featuresvectorvector;
    sample_type                                          featuresmatrix;
    decision_function<kernel_type>                       df;
    vector_normalizer<sample_type>                       normalizer;
    double                                               x_min, x_max, y_min, y_max, camheight, zoom;
    bool                                                 connection;
    std::vector<std::string>                             outputarray;
    string                                               outputstring;
};
