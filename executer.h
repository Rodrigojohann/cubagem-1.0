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
#include <pcl/io/pcd_io.h>
#include <connector.h>
#include <regressor.h>
#include <dlib/dlib/svm.h>

typedef pcl::PointXYZ                          PointT;
typedef pcl::PointXYZI                         PointI;
typedef pcl::PointCloud<PointT>                PointCloudT;
typedef pcl::PointCloud<PointI>                PointCloudI;
typedef std::vector<PointXYZ>                  CloudVector;
typedef dlib::matrix<double, 4, 1>             sample_type;
typedef dlib::radial_basis_kernel<sample_type> kernel_type;

class Executer
{
  public:

    std::string   Run();

  protected:

    PointCloudI::Ptr                                     cloudnew;
    PointCloudI::Ptr                                     cloudundistorted;
    PointCloudI::Ptr                                     cloud_preprocessed;
    PointCloudI::Ptr                                     filteredcloud;
    std::vector<pcl::PointIndices>                       clusters_indices;
    std::vector<PointCloudT::Ptr>                        clusters;
    int                                                  numberofboxes;
    std::string                                          numberofboxes_str;
    double                                               dimensionX, dimensionY, dimensionZ;
    double                                               volumemean;
    double                                               hullarea;
    std::vector<double>                                  featuresvector;
    std::vector<double>                                  sumfeaturesvector;
    std::vector<std::vector<double>>                     featuresvectorvector;
    sample_type                                          featuresmatrix;
    dlib::decision_function<kernel_type>                 df;
    dlib::vector_normalizer<sample_type>                 normalizer;
    double                                               x_min, x_max, y_min, y_max, camheight, zoom;
    bool                                                 connection;
    std::vector<std::string>                             outputarray;
    std::string                                          outputstring;
};
