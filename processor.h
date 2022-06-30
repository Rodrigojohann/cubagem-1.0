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
#include <pcl/surface/impl/mls.hpp>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <config.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <pcl/registration/icp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/grsd.h>

typedef pcl::PointXYZ           PointT;
typedef pcl::PointXYZI          PointI;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointI> PointCloudI;

class Processor
{
public:
    PointCloudI::Ptr                PreProcessingCloud(PointCloudI::Ptr inputcloud);
    PointCloudI::Ptr                FilterROI(PointCloudI::Ptr inputcloud, double x_min, double x_max, double y_min, double y_max, double camheight);
    PointCloudT::Ptr                RemovePallet(PointCloudT::Ptr inputcloud);
    std::vector<pcl::PointIndices>  CloudSegmentation(PointCloudI::Ptr inputcloud);
    bool                            ClusterCondition(const PointI& seedPoint, const PointI& candidatePoint, float squaredDistance);
    std::tuple<float, float, float> CalculateDimensions(PointCloudT::Ptr inputcloud);
    std::vector <PointCloudT::Ptr>  ExtractTopPlaneBox(PointCloudT::Ptr inputcloud, std::vector<pcl::PointIndices> inputclusters);
    std::vector <PointCloudT::Ptr>  IndicestoClouds(PointCloudI::Ptr inputcloud, std::vector<pcl::PointIndices> inputindices);
    double                          ConcaveHullArea(PointCloudT::Ptr inputcloud);
    double                          ConvexHullArea(PointCloudT::Ptr inputcloud);
    double                          SurfaceArea(double hullarea, double dimensionX, double dimensionY);
    PointCloudT::Ptr                ProjectCloud(PointCloudT::Ptr inputcloud);
    std::vector<double>             ExtractFeatures(PointCloudT::Ptr inputcloud);
    void                            SaveFeatures(std::vector<double> inputvector);
    std::vector<double>             ConcatFeatures(std::vector<std::vector<double>>);
    bool                            CheckPosition(PointCloudI::Ptr inputcloud, PointCloudI::Ptr templatecloud);
};
