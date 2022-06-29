#include "controller.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudI::Ptr Controller::PreProcessingCloud(PointCloudI::Ptr inputcloud){
// var
    PointCloudI::Ptr                        mls_points     (new PointCloudI);
    PointCloudI::Ptr                        filtered_cloud (new PointCloudI);
    pcl::search::KdTree<PointI>::Ptr        tree           (new pcl::search::KdTree<PointI>);
    pcl::RadiusOutlierRemoval<PointI>       outrem;
    pcl::MovingLeastSquares<PointI, PointI> mls;
////
    if (inputcloud->points.size() > 10){

        mls.setInputCloud(inputcloud);
        mls.setPolynomialOrder(3);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(0.03);
        mls.process(*mls_points);
    }

    outrem.setInputCloud(mls_points);
    outrem.setRadiusSearch(0.05);
    outrem.setMinNeighborsInRadius(3);
    outrem.setKeepOrganized(false);
    outrem.filter(*filtered_cloud);

    return filtered_cloud;
//    return inputcloud;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudI::Ptr Controller::FilterROI(PointCloudI::Ptr inputcloud, double x_min, double x_max, double y_min, double y_max, double camheight)
{
// var
    pcl::PassThrough<PointI>        pass_x;
    pcl::PassThrough<PointI>        pass_y;
    pcl::PassThrough<PointI>        pass_z;
    PointCloudI::Ptr                outputcloud  (new PointCloudI);
    PointCloudI::Ptr                outputcloud1 (new PointCloudI);
    pcl::PointIndicesPtr            ground       (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr     coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr          inliers      (new pcl::PointIndices);
    pcl::SACSegmentation<PointI>    seg;
    pcl::SegmentDifferences<PointI> p;
////
    pass_x.setInputCloud(inputcloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-x_min, x_max);;
    pass_x.filter(*inputcloud);

    pass_y.setInputCloud(inputcloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-y_min, y_max);
    pass_y.filter(*inputcloud);

    pass_z.setInputCloud(inputcloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.0, (camheight+0.08));
    pass_z.filter(*inputcloud);

    pass_z.setInputCloud(inputcloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits((camheight - 0.08), (camheight + 0.08));
    pass_z.filter(*outputcloud);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2);

    if (outputcloud->points.size() > 10){
        seg.setInputCloud (outputcloud);
        seg.segment(*inliers, *coefficients);

        outputcloud1->points.resize(inliers->indices.size());

        for (size_t i = 0; i < inliers->indices.size(); ++i)
        {
            outputcloud1->points[i].x = (*outputcloud)[inliers->indices[i]].x;
            outputcloud1->points[i].y = (*outputcloud)[inliers->indices[i]].y;
            outputcloud1->points[i].z = (*outputcloud)[inliers->indices[i]].z;
            outputcloud1->points[i].intensity = (*outputcloud)[i].intensity;
        }

        p.setInputCloud(inputcloud);
        p.setTargetCloud(outputcloud1);
        p.setDistanceThreshold(0.001);
        p.segment(*outputcloud1);
    }
    else
    {
        outputcloud1->points.resize(inputcloud->points.size());
        for (size_t i = 0; i < inputcloud->points.size(); ++i)
        {
            outputcloud1->points[i].x = (*inputcloud)[i].x;
            outputcloud1->points[i].y = (*inputcloud)[i].y;
            outputcloud1->points[i].z = (*inputcloud)[i].z;
            outputcloud1->points[i].intensity = (*inputcloud)[i].intensity;
        }
    }

    return outputcloud1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudT::Ptr Controller::RemovePallet(PointCloudT::Ptr inputcloud)
{
// var
    PointCloudT::Ptr                filtered_cloud (new PointCloudT);
    PointT                   minPt, maxPt;
    pcl::PassThrough<PointT> pass_z;
////
    pcl::getMinMax3D(*inputcloud, minPt, maxPt);

    pass_z.setInputCloud(inputcloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0, (CAMHEIGHT - PALLETHEIGHT));
    pass_z.filter(*filtered_cloud);

    return filtered_cloud;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<pcl::PointIndices> Controller::CloudSegmentation(PointCloudI::Ptr inputcloud)
{
// var
    std::vector<pcl::PointIndices>                      clusters;
    pcl::ConditionalEuclideanClustering<PointI>         clustering  (true);
    pcl::search::KdTree<PointI>::Ptr                    search_tree (new pcl::search::KdTree<PointI>);
    pcl::EuclideanClusterExtraction<PointI>             ec;
////
    clustering.setInputCloud(inputcloud);
    clustering.setClusterTolerance(0.05);
    clustering.setMinClusterSize(25);
    clustering.setMaxClusterSize(250000);
    clustering.setConditionFunction(boost::bind(&Controller::ClusterCondition, this, _1, _2, _3));
    clustering.segment(clusters);

    return clusters;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Controller::ClusterCondition(const PointI& seedPoint, const PointI& candidatePoint, float squaredDistance)
{
// var
    float intensitythreshold = 0.8;
    float heightthreshold = 0.03;
////
    if (std::abs(seedPoint.z) - std::abs(candidatePoint.z) < heightthreshold)
    {
        if (std::abs(seedPoint.intensity - candidatePoint.intensity) < intensitythreshold)
         {
             return (true);
         }
    }
    else
    {
        return (false);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<float, float, float> Controller::CalculateDimensions(PointCloudT::Ptr inputcloud)
{
// var
    pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
    PointT                                  min_point_OBB;
    PointT                                  max_point_OBB;
    PointT                                  position_OBB;
    Eigen::Matrix3f                         rotational_matrix_OBB;
    float                                   dimensionX, dimensionY, dimensionZ;
    PointT                                  centroid;
////
    pcl::computeCentroid(*inputcloud, centroid);

    feature_extractor.setInputCloud(inputcloud);
    feature_extractor.compute();
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    dimensionX = (max_point_OBB.x - min_point_OBB.x)*100;
    dimensionY = (max_point_OBB.y - min_point_OBB.y)*100;
    dimensionZ = (CAMHEIGHT - centroid.z)*100;

    return std::make_tuple(dimensionX, dimensionY, dimensionZ);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<PointCloudT::Ptr> Controller::ExtractTopPlaneBox(PointCloudT::Ptr inputcloud, std::vector <pcl::PointIndices> inputclusters)
{
// var
    std::vector<PointCloudT::Ptr>     selectedclusters;
    PointCloudT::Ptr                  cloud_plane     (new PointCloudT);
    PointCloudT::Ptr                  segmented_cloud (new PointCloudT);
    pcl::PointIndices::Ptr            inliers         (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr       coefficients    (new pcl::ModelCoefficients());
    pcl::SACSegmentation<PointT>      seg;
    pcl::ExtractIndices<PointT>       extract;
    pcl::RadiusOutlierRemoval<PointT> outrem;
////
    for (int i = 0; i < inputclusters.size(); ++i)
    {
        segmented_cloud.reset(new PointCloudT);
        cloud_plane.reset(new PointCloudT);
        segmented_cloud->points.resize(inputclusters[i].indices.size());

        for (size_t j = 0; j < inputclusters[i].indices.size(); ++j)
        {
            segmented_cloud->points[j].x = (*inputcloud)[inputclusters[i].indices[j]].x;
            segmented_cloud->points[j].y = (*inputcloud)[inputclusters[i].indices[j]].y;
            segmented_cloud->points[j].z = (*inputcloud)[inputclusters[i].indices[j]].z;
        }

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setAxis(Eigen::Vector3f::UnitZ());
        seg.setEpsAngle(5.0f*(M_PI/180.0f));
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.035);

        seg.setInputCloud(segmented_cloud);
        seg.segment(*inliers, *coefficients);

        extract.setInputCloud(segmented_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        outrem.setInputCloud(cloud_plane);
        outrem.setRadiusSearch(0.04);
        outrem.setMinNeighborsInRadius(2);
        outrem.setKeepOrganized(false);
        outrem.filter(*cloud_plane);

        if (cloud_plane->points.size() > 10)
        {
            selectedclusters.push_back(cloud_plane);
        }
    }

    return selectedclusters;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<PointCloudT::Ptr> Controller::IndicestoClouds(PointCloudI::Ptr inputcloud, std::vector<pcl::PointIndices> inputindices)
{
// var
    PointCloudT::Ptr              segmented_cloud (new PointCloudT);
    std::vector<PointCloudT::Ptr> selectedclusters;
////
    for (int i = 0; i < inputindices.size(); ++i)
    {
        segmented_cloud.reset(new PointCloudT);
        segmented_cloud->points.resize(inputindices[i].indices.size());

        for(size_t j = 0; j<inputindices[i].indices.size(); ++j)
        {
            segmented_cloud->points[j].x = (*inputcloud)[inputindices[i].indices[j]].x;
            segmented_cloud->points[j].y = (*inputcloud)[inputindices[i].indices[j]].y;
            segmented_cloud->points[j].z = (*inputcloud)[inputindices[i].indices[j]].z;
        }

        selectedclusters.push_back(segmented_cloud);
    }

    return selectedclusters;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Controller::ConcaveHullArea(PointCloudT::Ptr inputcloud)
{
// var
    PointCloudT::Ptr         cloud_hull (new PointCloudT);
    double                   hullarea;
    pcl::ConcaveHull<PointT> chull;
////
    if (inputcloud->points.size() > 10)
    {
        chull.setInputCloud(inputcloud);
        chull.setDimension(2);
        chull.setAlpha(0.1);
        chull.reconstruct(*cloud_hull);
    }

    hullarea = 10000*(pcl::calculatePolygonArea(*cloud_hull));

    return hullarea;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Controller::ConvexHullArea(PointCloudT::Ptr inputcloud)
{
// var
    PointCloudT::Ptr        cloud_hull (new PointCloudT);
    pcl::ConvexHull<PointT> chull;
    double                  hullarea;
////
    if (inputcloud->points.size() > 10)
    {
        chull.setInputCloud(inputcloud);
        chull.setDimension(2);
        chull.reconstruct(*cloud_hull);
    }

    hullarea = 10000*(pcl::calculatePolygonArea(*cloud_hull));

    return hullarea;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Controller::SurfaceArea(double hullarea, double dimensionX, double dimensionY)
{
// var
    double OBBarea;
    double surfacearea;
////
    OBBarea = dimensionX*dimensionY;

    if (hullarea/OBBarea < 0.8)
    {
        surfacearea = hullarea;
    }
    else
    {
        surfacearea = OBBarea;
    }

    return surfacearea;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudT::Ptr Controller::ProjectCloud(PointCloudT::Ptr inputcloud)
{
// var
    PointCloudT::Ptr outputcloud (new PointCloudT);
////
    outputcloud->points.resize(inputcloud->points.size());

    for(size_t i = 0; i < inputcloud->points.size(); ++i)
    {
        outputcloud->points[i].x = (*inputcloud)[i].x;
        outputcloud->points[i].y = (*inputcloud)[i].y;
        outputcloud->points[i].z = CAMHEIGHT;
    }

    return outputcloud;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> Controller::ExtractFeatures(PointCloudT::Ptr inputcloud)
{
// var
    std::vector<double>                     outputvector;
    double                                  centroidz, hullarea;
    PointT                                  centroid;
////
    pcl::computeCentroid(*inputcloud, centroid);
    centroidz = centroid.z;

    hullarea = ConcaveHullArea(ProjectCloud(inputcloud));

    outputvector.push_back(centroidz);
    outputvector.push_back(hullarea);

    return outputvector;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Controller::SaveFeatures(std::vector<double> inputvector)
{
// var
    std::ofstream outdata;
////
    outdata.open("./svr_train/sickboxes_face2_new_cleaned.csv", std::ios_base::app);
    if( !outdata ) { // file couldn't be opened
       cerr << "Error: file could not be opened";
       exit(1);
    }
    outdata << "\n";
    for (int i = 0; i < inputvector.size(); ++i)
          outdata << inputvector[i] << ", ";
       outdata.close();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> Controller::ConcatFeatures(std::vector<std::vector<double>> featuresvectorvector)
{
// var
    std::vector<double> outputvector;
    std::vector<double> majorvector;
    std::vector<double> minorvector;
    std::vector<double> featuresvector;
////
    outputvector.resize(featuresvectorvector[0].size());
    majorvector.resize(featuresvectorvector[0].size());
    minorvector.resize(featuresvectorvector[0].size());
//cout << "\n\nclusters: " << featuresvectorvector.size();

    majorvector = featuresvectorvector[0];

//cout << "\n\nCentroid: "  << majorvector[0];
//cout << "\nArea: " << majorvector[1];

    for (int i = 1; i < featuresvectorvector.size(); ++i)
    {
        featuresvector = featuresvectorvector[i];

        minorvector[0] += featuresvector[1]*featuresvector[0];
        minorvector[1] += featuresvector[1];

//cout << "\n\nCentroid: "  << featuresvector[0];
//cout << "\nArea: " << featuresvector[1];

    }

    minorvector[0] = minorvector[0]/minorvector[1];

    if (featuresvectorvector.size() == 1)
    {
        outputvector = majorvector;
    }

    else
    {
        outputvector[0] = (majorvector[1]*majorvector[0] + 5*minorvector[1]*minorvector[0])/(majorvector[1]+5*minorvector[1]);
        outputvector[1] = majorvector[1] + minorvector[1];
    }

    outputvector.push_back(majorvector[0]);
    outputvector.push_back(majorvector[1]);

//cout << "\n\nTotal Centroid: " << outputvector[0];
//cout << "\nTotal Area: " << outputvector[1];

    return outputvector;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Controller::CheckPosition(PointCloudI::Ptr inputcloud, PointCloudI::Ptr templatecloud)
{
// var
    PointCloudI::Ptr outputcloud (new PointCloudI);
    pcl::IterativeClosestPoint<PointI, PointI> icp;
    double fitness_score;
////
    icp.setInputSource(inputcloud);
    icp.setInputTarget(templatecloud);

    icp.align(*outputcloud);
    fitness_score = icp.getFitnessScore();
    //icp.getFinalTransformation();

    if (fitness_score < 0.1)
    {
        return true;
    }
    else
    {
        return false;
    }

//// var
//    pcl::PassThrough<PointI> pass_x;
//    pcl::PassThrough<PointI> pass_y;
//    PointCloudI::Ptr         outputcloud  (new PointCloudI);
//////
//    pass_x.setInputCloud(inputcloud);
//    pass_x.setFilterFieldName("x");
//    pass_x.setFilterLimits((X_CENTER-PALLETLENGTH/2), (X_CENTER+PALLETLENGTH/2));;
//    pass_x.filter(*outputcloud);

//    pass_y.setInputCloud(outputcloud);
//    pass_y.setFilterFieldName("y");
//    pass_y.setFilterLimits((Y_CENTER-PALLETLENGTH/2), (Y_CENTER+PALLETLENGTH/2));;
//    pass_y.filter(*outputcloud);

//    if ((inputcloud->points.size() - outputcloud->points.size()/inputcloud->points.size()) < 0.02)
//    {
//        return true;
//    }
//    else
//    {
//        return false;
//    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

