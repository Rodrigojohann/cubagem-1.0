#include <dlib/dlib/svm.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

typedef pcl::PointCloud<PointT> PointCloudT;

class Regressor
{
  public:
    std::vector<double> ExtractFeatures(PointCloudT::Ptr inputcloud);
    void                SaveFeatures(std::vector<double> inputvector);
    std::vector<double> ConcatFeatures(std::vector<std::vector<double>>);

};
