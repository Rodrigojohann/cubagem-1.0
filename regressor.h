#ifndef REGRESSOR_H
#define REGRESSOR_H

#include <dlib/dlib/svm.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <processor.h>

typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Regressor
{
  public:
    std::vector<double> ExtractFeatures(PointCloudT::Ptr inputcloud);
    void                SaveFeatures(std::vector<double> inputvector);
    std::vector<double> ConcatFeatures(std::vector<std::vector<double>>);

};

#endif // REGRESSOR_H
