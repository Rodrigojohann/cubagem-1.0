#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>
#define  CAMHEIGHT 2.21

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

class Controller
{
public:
    std::vector <pcl::PointIndices>                 SortClusters(std::vector <pcl::PointIndices> inputclusters, int size);
    PointCloudT::Ptr                                FilterCloud(PointCloudT::Ptr inputcloud);
    std::tuple<std::vector<pcl::PointIndices>, int> CloudSegmentation(PointCloudT::Ptr inputcloud);
    std::tuple<float, float, float>              CalculateDimensions(PointCloudT::Ptr inputcloud);
    bool                                            NormalOrientation (PointCloudT::Ptr inputcloud, pcl::PointIndices inputcluster);
    std::vector <pcl::PointIndices>                 RemoveInclined(PointCloudT::Ptr inputcloud, std::vector<pcl::PointIndices> inputclusters);
};
