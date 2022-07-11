#ifndef EXECUTER_H
#define EXECUTER_H

#include <sensor.h>
#include <processor.h>
#include <thread>
#include <math.h>
#include <iomanip>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <connector.h>
#include <regressor.h>
#include <dlib/dlib/svm.h>
#include <pcl/io/point_cloud_image_extractors.h>

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

    void Run();

  protected:

    PointCloudI::Ptr                                     cloudnew;
    PointCloudI::Ptr                                     cloudundistorted;
    PointCloudI::Ptr                                     cloud_preprocessed;
    PointCloudI::Ptr                                     filteredcloud;
    PointCloudT::Ptr                                     concatclusters;
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

#endif // EXECUTER_H
