#include "controller.h"

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector <pcl::PointIndices> Controller::SortClusters(std::vector <pcl::PointIndices> inputclusters, int size)
{
    std::vector <pcl::PointIndices> sortedclusters;
    pcl::PointIndices temp;

    sortedclusters = inputclusters;

    for (size_t i=1 ; i < size; ++i)
    {
        temp = inputclusters[i];
        int j = i - 1;
        while (j >= 0 && temp.indices.size() > sortedclusters[j].indices.size())
        {
            sortedclusters[j+1] = inputclusters[j];
            --j;
        }
        sortedclusters[j+1] = temp;
    }
    return sortedclusters;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudT::Ptr Controller::FilterCloud(PointCloudT::Ptr inputcloud)
{
// var
    pcl::PassThrough<pcl::PointXYZ>                          pass_x;
    pcl::PassThrough<pcl::PointXYZ>                          pass_y;
    pcl::PassThrough<pcl::PointXYZ>                          pass_z;
    PointCloudT::Ptr                                         outputcloud (new PointCloudT);
    PointCloudT::Ptr                                         outputcloud1 (new PointCloudT);
    PointCloudT::Ptr                                         outputcloud2 (new PointCloudT);
    pcl::PointIndicesPtr                                     ground (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr                              coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr                                   inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ>                      seg;
    pcl::SegmentDifferences<pcl::PointXYZ>                   p;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr                  tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>                        mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
////
    pass_x.setInputCloud(inputcloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-X_MIN, X_MAX);;
    pass_x.filter(*inputcloud);

    pass_y.setInputCloud(inputcloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-Y_MIN, Y_MAX);
    pass_y.filter(*inputcloud);

    pass_z.setInputCloud(inputcloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits((CAMHEIGHT-0.12), (CAMHEIGHT+0.12));
    pass_z.filter(*outputcloud);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.2);

    if (outputcloud->points.size() > 10){
        seg.setInputCloud (outputcloud);
        seg.segment (*inliers, *coefficients);

        outputcloud1.reset(new pcl::PointCloud<pcl::PointXYZ>);
        outputcloud1->points.resize(inliers->indices.size());

        for(size_t i=0; i < inliers->indices.size(); ++i)
        {
            outputcloud1->points[i].x = (*outputcloud)[inliers->indices[i]].x;
            outputcloud1->points[i].y = (*outputcloud)[inliers->indices[i]].y;
            outputcloud1->points[i].z = (*outputcloud)[inliers->indices[i]].z;
        }

        p.setInputCloud (inputcloud);
        p.setTargetCloud (outputcloud1);
        p.setDistanceThreshold (0.001);
        p.segment(*outputcloud1);

        mls.setInputCloud (outputcloud1);
        mls.setPolynomialFit (false)
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.05);
        mls.process (mls_points);

        outputcloud2->points.resize(mls_points.size());

        for(size_t i=0; i<outputcloud2->points.size(); ++i)
        {
            outputcloud2->points[i].x = (mls_points)[i].x;
            outputcloud2->points[i].y = (mls_points)[i].y;
            outputcloud2->points[i].z = (mls_points)[i].z;
        }
    }
    return outputcloud2;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<std::vector<pcl::PointIndices>, int> Controller::CloudSegmentation(PointCloudT::Ptr inputcloud)
{
// var
    pcl::search::Search<pcl::PointXYZ>::Ptr        tree (new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector <pcl::PointIndices>                clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
////
    if (inputcloud->points.size() > 10){
        tree->setInputCloud (inputcloud);
        ec.setClusterTolerance (0.015);
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (inputcloud);
        ec.extract (clusters);
    }

    return std::make_tuple(clusters, clusters.size());
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<float, float, float> Controller::CalculateDimensions(PointCloudT::Ptr inputcloud)
{
// var
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    pcl::PointXYZ                                  minPt;
    pcl::PointXYZ                                  maxPt;
    pcl::PointXYZ                                  min_point_OBB;
    pcl::PointXYZ                                  max_point_OBB;
    pcl::PointXYZ                                  position_OBB;
    Eigen::Matrix3f                                rotational_matrix_OBB;
    float                                          dimensionX, dimensionY, dimensionZ;
    pcl::PassThrough<pcl::PointXYZ>                passz;
////
    pcl::getMinMax3D(*inputcloud, minPt, maxPt);

    passz.setInputCloud(inputcloud);
    passz.setFilterFieldName ("z");
    passz.setFilterLimits ((minPt.z-0.1), (minPt.z+0.1));
    passz.filter(*inputcloud);

    feature_extractor.setInputCloud(inputcloud);
    feature_extractor.compute();
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    dimensionX = (max_point_OBB.x - min_point_OBB.x);
    dimensionY = (max_point_OBB.y - min_point_OBB.y);
    dimensionZ = (CAMHEIGHT - minPt.z);

    return std::make_tuple(dimensionX, dimensionY, dimensionZ);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Controller::NormalOrientation(PointCloudT::Ptr inputcloud, pcl::PointIndices inputcluster)
{
// var
    pcl::PointCloud<pcl::Normal>::Ptr                 normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr           tree (new pcl::search::KdTree<pcl::PointXYZ>());
    PointCloudT::Ptr                                  segmented_cloud (new PointCloudT);
    float                                             normal_x_mean;
    float                                             normal_y_mean;
    float                                             tolerance = 0.25;
////
    segmented_cloud->points.resize(inputcluster.indices.size());

    for(size_t i=0; i<inputcluster.indices.size(); ++i)
    {
        segmented_cloud->points[i].x = (*inputcloud)[inputcluster.indices[i]].x;
        segmented_cloud->points[i].y = (*inputcloud)[inputcluster.indices[i]].y;
        segmented_cloud->points[i].z = (*inputcloud)[inputcluster.indices[i]].z;
    }

    ne.setInputCloud(segmented_cloud);
    ne.setSearchMethod (tree);
    ne.setKSearch (5);
    ne.compute(*normals);

    normal_x_mean = 0.0;
    normal_y_mean = 0.0;

    for (size_t i=0; i < normals->size(); ++i)
    {
        normal_x_mean += (*normals)[i].normal_x;                                                                
        normal_y_mean += (*normals)[i].normal_y;
    }

    normal_x_mean = normal_x_mean/normals->size();
    normal_y_mean = normal_y_mean/normals->size();

    if (normal_x_mean < tolerance && normal_x_mean > (-tolerance) && normal_y_mean < tolerance && normal_y_mean > (-tolerance))
    {
        return true;
    }
    else
    {
        return false;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector <pcl::PointIndices> Controller::RemoveInclined(PointCloudT::Ptr inputcloud, std::vector <pcl::PointIndices> inputclusters)
{
// var
    std::vector <pcl::PointIndices> selectedclusters;
    bool IsNormal;
////
    for (int i=0; i<inputclusters.size(); ++i)
    {
        IsNormal = NormalOrientation(inputcloud, inputclusters[i]);
        if (IsNormal == true)
        {
            selectedclusters.push_back(inputclusters[i]);
        }
    }
    return selectedclusters;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
