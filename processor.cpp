#include "processor.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudI::Ptr Processor::RemoveDistortion(PointCloudI::Ptr inputcloud){
// var
    PointCloudI::Ptr outputcloud (new PointCloudI);
    double           xi, yi, zi, xi_0, yi_0, zi_0, i_0, Ri, M, x2, y2;
////
    if (inputcloud->points.size() > 0)
    {
        outputcloud->points.resize(inputcloud->points.size());

        for(size_t i=0; i<outputcloud->points.size(); ++i)
        {
            xi_0 = (*inputcloud)[i].x;
            yi_0 = (*inputcloud)[i].y;
            zi_0 = (*inputcloud)[i].z;
            i_0 = (*inputcloud)[i].intensity;

            x2 = xi_0*xi_0;
            y2 = yi_0*yi_0;

            xi = xi_0;
            yi = yi_0;
            zi = zi_0 - zi_0*(7.77e-6*exp(-((4*x2)+y2)/8192) + 4.83e-6*exp(-(x2+y2)/4608) + 6.99e-6*exp(-(x2+y2)/8192));

            outputcloud->points[i].x = (xi);
            outputcloud->points[i].y = (yi);
            outputcloud->points[i].z = (zi);
            outputcloud->points[i].intensity = (i_0);
        }
    }

    return outputcloud;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudI::Ptr Processor::PreProcessingCloud(PointCloudI::Ptr inputcloud)
{
// var
    PointCloudI::Ptr                        mls_points     (new PointCloudI);
    PointCloudI::Ptr                        filtered_cloud (new PointCloudI);
    pcl::search::KdTree<PointI>::Ptr        tree           (new pcl::search::KdTree<PointI>);
    pcl::RadiusOutlierRemoval<PointI>       outrem;
    pcl::MovingLeastSquares<PointI, PointI> mls;
////
    if (inputcloud->points.size() > 0)
    {
        mls.setInputCloud(inputcloud);
        mls.setPolynomialOrder(3);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(0.03);
        mls.process(*mls_points);

        outrem.setInputCloud(mls_points);
        outrem.setRadiusSearch(0.05);
        outrem.setMinNeighborsInRadius(3);
        outrem.setKeepOrganized(false);
        outrem.filter(*filtered_cloud);
    }

    return filtered_cloud;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudI::Ptr Processor::FilterROI(PointCloudI::Ptr inputcloud, double x_min, double x_max, double y_min, double y_max, double camheight)
{
// var
    pcl::PassThrough<PointI>        pass_x;
    pcl::PassThrough<PointI>        pass_y;
    pcl::PassThrough<PointI>        pass_z;
    PointCloudI::Ptr                unfilteredcloud    (new PointCloudI);
    PointCloudI::Ptr                cloud_passthrough  (new PointCloudI);
    PointCloudI::Ptr                outputcloud        (new PointCloudI);
    pcl::PointIndicesPtr            ground             (new pcl::PointIndices);
    pcl::PointIndices::Ptr          inliers            (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr     coefficients       (new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointI>    seg;
    pcl::SegmentDifferences<PointI> p;
////
    if (inputcloud->points.size() > 100)
    {
        *unfilteredcloud = *inputcloud;

        pass_x.setInputCloud(unfilteredcloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-x_min, x_max);;
        pass_x.filter(*unfilteredcloud);

        pass_y.setInputCloud(unfilteredcloud);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-y_min, y_max);
        pass_y.filter(*unfilteredcloud);

        pass_z.setInputCloud(unfilteredcloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.0, (camheight+0.08));
        pass_z.filter(*unfilteredcloud);

        pass_z.setInputCloud(unfilteredcloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits((camheight - 0.08), (camheight + 0.08));
        pass_z.filter(*cloud_passthrough);
    }

    if (cloud_passthrough->points.size() > 100)
    {
        seg.setInputCloud(cloud_passthrough);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.2);
        seg.segment(*inliers, *coefficients);

        outputcloud->points.resize(inliers->indices.size());

        for (size_t i = 0; i < inliers->indices.size(); ++i)
        {
            outputcloud->points[i].x = (*cloud_passthrough)[inliers->indices[i]].x;
            outputcloud->points[i].y = (*cloud_passthrough)[inliers->indices[i]].y;
            outputcloud->points[i].z = (*cloud_passthrough)[inliers->indices[i]].z;
            outputcloud->points[i].intensity = (*cloud_passthrough)[i].intensity;
        }

        p.setInputCloud(unfilteredcloud);
        p.setTargetCloud(outputcloud);
        p.setDistanceThreshold(0.001);
        p.segment(*outputcloud);
    }
    else
    {
        outputcloud->points.resize(unfilteredcloud->points.size());
        for (size_t i = 0; i < inputcloud->points.size(); ++i)
        {
            outputcloud->points[i].x = (*unfilteredcloud)[i].x;
            outputcloud->points[i].y = (*unfilteredcloud)[i].y;
            outputcloud->points[i].z = (*unfilteredcloud)[i].z;
            outputcloud->points[i].intensity = (*unfilteredcloud)[i].intensity;
        }
    }

    return outputcloud;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<pcl::PointIndices> Processor::CloudSegmentation(PointCloudI::Ptr inputcloud)
{
// var
    std::vector<pcl::PointIndices>              clusters;
    pcl::ConditionalEuclideanClustering<PointI> clustering  (true);
    pcl::search::KdTree<PointI>::Ptr            search_tree (new pcl::search::KdTree<PointI>);
    pcl::EuclideanClusterExtraction<PointI>     ec;
////
    if (inputcloud->points.size() > 0)
    {
        clustering.setInputCloud(inputcloud);
        clustering.setClusterTolerance(0.05);
        clustering.setMinClusterSize(25);
        clustering.setMaxClusterSize(250000);
        clustering.setConditionFunction(boost::bind(&Processor::ClusterCondition, this, _1, _2, _3));
        clustering.segment(clusters);
    }

    return clusters;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Processor::ClusterCondition(const PointI& seedPoint, const PointI& candidatePoint, float squaredDistance)
{
// var
    float intensitythreshold = 0.8;
    float heightthreshold = 0.03;
////
    if (std::abs(seedPoint.z) - std::abs(candidatePoint.z) < heightthreshold)
    {
        if (std::abs(seedPoint.intensity - candidatePoint.intensity) < intensitythreshold)
        {
           return true;
        }
        else
        {
           return false;
        }
    }
    else
    {
        return false;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<float, float, float> Processor::CalculateDimensions(PointCloudT::Ptr inputcloud)
{
// var
    pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
    PointT                                  min_point_OBB;
    PointT                                  max_point_OBB;
    PointT                                  position_OBB;
    Eigen::Matrix3f                         rotational_matrix_OBB;
    float                                   dimensionX = 0.0;
    float                                   dimensionY = 0.0;
    float                                   dimensionZ = 0.0;
    PointT                                  centroid;
////
    if (inputcloud->points.size() > 0)
    {
        pcl::computeCentroid(*inputcloud, centroid);

        feature_extractor.setInputCloud(inputcloud);
        feature_extractor.compute();
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

        dimensionX = (max_point_OBB.x - min_point_OBB.x)*100;
        dimensionY = (max_point_OBB.y - min_point_OBB.y)*100;
        dimensionZ = (CAMHEIGHT - centroid.z)*100;
    }

    if ((dimensionX > 5) and (dimensionY > 5) and (dimensionZ > 5))
    {
        return std::make_tuple(dimensionX, dimensionY, dimensionZ);
    }
    else
    {
        return std::make_tuple(0.0, 0.0, 0.0);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<PointCloudT::Ptr> Processor::ExtractTopPlaneBox(PointCloudT::Ptr inputcloud, std::vector <pcl::PointIndices> inputclusters)
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
std::vector<PointCloudT::Ptr> Processor::IndicestoClouds(PointCloudI::Ptr inputcloud, std::vector<pcl::PointIndices> inputindices)
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
double Processor::ConcaveHullArea(PointCloudT::Ptr inputcloud)
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
double Processor::ConvexHullArea(PointCloudT::Ptr inputcloud)
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
double Processor::SurfaceArea(double hullarea, double dimensionX, double dimensionY)
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
PointCloudT::Ptr Processor::ProjectCloud(PointCloudT::Ptr inputcloud)
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
bool Processor::CheckPosition(PointCloudI::Ptr inputcloud, PointCloudI::Ptr templatecloud)
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
pcl::PCLImage Processor::GenerateColoredCloud(PointCloudI::Ptr inputcloud, PointCloudT::Ptr clusters)
{
//var
    PointCloudT::Ptr inputcloudxyz (new PointCloudT);
    PointCloudT::Ptr clusterswithmaxmin (new PointCloudT);
    pcl::PCLImage    image;
    float            angularResolution = (float)(0.1f*(M_PI/180.0f));
    float            maxAngleWidth = (float)(360.0f*(M_PI/180.0f));
    float            maxAngleHeight = (float)(180.0f*(M_PI/180.0f));
    Eigen::Affine3f  sensorPose = (Eigen::Affine3f) Eigen::Translation3f(0.0f, 0.0f, 0.0f); //The position of the sensor defines the virtual sensor The 6 DOF position, its origin is roll=pitch=yaw=0.
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; //x is facing right, y is down, z-axis is forward, another option is laser frame, x is facing forward, y is to left, z up.
    float noiseLevel = 0.00;
    float minRange = 0.0f;
    int borderSize = 1;
    pcl::RangeImage::Ptr rangeImage_input (new pcl::RangeImage);
    pcl::RangeImage::Ptr rangeImage_clusters (new pcl::RangeImage);
    pcl::RangeImage::Ptr rangeImage (new pcl::RangeImage);
    PointT minPtinput, maxPtinput;
////
    inputcloudxyz->points.resize(inputcloud->points.size());

    for (int i = 0; i < inputcloudxyz->points.size(); ++i)
    {
        inputcloudxyz->points[i].x = (*inputcloud)[i].x;
        inputcloudxyz->points[i].y = (*inputcloud)[i].y;
        inputcloudxyz->points[i].z = (*inputcloud)[i].z;
    }

    pcl::getMinMax3D(*inputcloudxyz, minPtinput, maxPtinput);

    inputcloudxyz->points.resize((inputcloud->points.size() + 8));

    inputcloudxyz->points[inputcloud->points.size()].x = minPtinput.x;
    inputcloudxyz->points[inputcloud->points.size()].y = minPtinput.y;
    inputcloudxyz->points[inputcloud->points.size()].z = minPtinput.z;

    inputcloudxyz->points[inputcloud->points.size()+1].x = minPtinput.x;
    inputcloudxyz->points[inputcloud->points.size()+1].y = minPtinput.y;
    inputcloudxyz->points[inputcloud->points.size()+1].z = maxPtinput.z;

    inputcloudxyz->points[inputcloud->points.size()+2].x = minPtinput.x;
    inputcloudxyz->points[inputcloud->points.size()+2].y = maxPtinput.y;
    inputcloudxyz->points[inputcloud->points.size()+2].z = minPtinput.z;

    inputcloudxyz->points[inputcloud->points.size()+3].x = minPtinput.x;
    inputcloudxyz->points[inputcloud->points.size()+3].y = maxPtinput.y;
    inputcloudxyz->points[inputcloud->points.size()+3].z = maxPtinput.z;

    inputcloudxyz->points[inputcloud->points.size()+4].x = maxPtinput.x;
    inputcloudxyz->points[inputcloud->points.size()+4].y = minPtinput.y;
    inputcloudxyz->points[inputcloud->points.size()+4].z = minPtinput.z;

    inputcloudxyz->points[inputcloud->points.size()+5].x = maxPtinput.x;
    inputcloudxyz->points[inputcloud->points.size()+5].y = minPtinput.y;
    inputcloudxyz->points[inputcloud->points.size()+5].z = maxPtinput.z;

    inputcloudxyz->points[inputcloud->points.size()+6].x = maxPtinput.x;
    inputcloudxyz->points[inputcloud->points.size()+6].y = maxPtinput.y;
    inputcloudxyz->points[inputcloud->points.size()+6].z = minPtinput.z;

    inputcloudxyz->points[inputcloud->points.size()+7].x = maxPtinput.x;
    inputcloudxyz->points[inputcloud->points.size()+7].y = maxPtinput.y;
    inputcloudxyz->points[inputcloud->points.size()+7].z = maxPtinput.z;

    clusterswithmaxmin->points.resize((clusters->points.size() + 8));

    for (int i = 0; i < clusters->points.size(); ++i)
    {
        clusterswithmaxmin->points[i].x = (*clusters)[i].x;
        clusterswithmaxmin->points[i].y = (*clusters)[i].y;
        clusterswithmaxmin->points[i].z = (*clusters)[i].z;
    }

    clusterswithmaxmin->points[clusters->points.size()].x = minPtinput.x;
    clusterswithmaxmin->points[clusters->points.size()].y = minPtinput.y;
    clusterswithmaxmin->points[clusters->points.size()].z = minPtinput.z;

    clusterswithmaxmin->points[clusters->points.size()+1].x = minPtinput.x;
    clusterswithmaxmin->points[clusters->points.size()+1].y = minPtinput.y;
    clusterswithmaxmin->points[clusters->points.size()+1].z = maxPtinput.z;

    clusterswithmaxmin->points[clusters->points.size()+2].x = minPtinput.x;
    clusterswithmaxmin->points[clusters->points.size()+2].y = maxPtinput.y;
    clusterswithmaxmin->points[clusters->points.size()+2].z = minPtinput.z;

    clusterswithmaxmin->points[clusters->points.size()+3].x = minPtinput.x;
    clusterswithmaxmin->points[clusters->points.size()+3].y = maxPtinput.y;
    clusterswithmaxmin->points[clusters->points.size()+3].z = maxPtinput.z;

    clusterswithmaxmin->points[clusters->points.size()+4].x = maxPtinput.x;
    clusterswithmaxmin->points[clusters->points.size()+4].y = minPtinput.y;
    clusterswithmaxmin->points[clusters->points.size()+4].z = minPtinput.z;

    clusterswithmaxmin->points[clusters->points.size()+5].x = maxPtinput.x;
    clusterswithmaxmin->points[clusters->points.size()+5].y = minPtinput.y;
    clusterswithmaxmin->points[clusters->points.size()+5].z = maxPtinput.z;

    clusterswithmaxmin->points[clusters->points.size()+6].x = maxPtinput.x;
    clusterswithmaxmin->points[clusters->points.size()+6].y = maxPtinput.y;
    clusterswithmaxmin->points[clusters->points.size()+6].z = minPtinput.z;

    clusterswithmaxmin->points[clusters->points.size()+7].x = maxPtinput.x;
    clusterswithmaxmin->points[clusters->points.size()+7].y = maxPtinput.y;
    clusterswithmaxmin->points[clusters->points.size()+7].z = maxPtinput.z;


    rangeImage_input->createFromPointCloud(*inputcloudxyz, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    float *ranges_input = rangeImage_input->getRangesArray();
    unsigned char *rgb_image_input = pcl::visualization::FloatImageUtils::getVisualImage(ranges_input, rangeImage_input->width, rangeImage_input->height, 0.0, 1.0, true);

    pcl::io::saveRgbPNGFile("saveRangeImageRGB_input.png", rgb_image_input, rangeImage_input->width, rangeImage_input->height);


    rangeImage_clusters->createFromPointCloud(*clusterswithmaxmin, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    float *ranges_clusters = rangeImage_clusters->getRangesArray();
    unsigned char *rgb_image_clusters = pcl::visualization::FloatImageUtils::getVisualImage(ranges_clusters, rangeImage_clusters->width, rangeImage_clusters->height, 0.0, 1.0, true);

    pcl::io::saveRgbPNGFile("saveRangeImageRGB_clusters.png", rgb_image_clusters, rangeImage_clusters->width, rangeImage_clusters->height);

    return image;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
