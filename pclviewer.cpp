#include "pclviewer.h"

using namespace std;

//string PCLViewer::Run(char* ipaddr){
ObjectsData PCLViewer::Run(char* ipaddr){
////////
    Controller c;
    Sensor s;
    ObjectsData outputdata;

    x1 = x2 = x3 = x4 = x5 = 0.0;
    y1 = y2 = y3 = y4 = y5 = 0.0;
    z1 = z2 = z3 = z4 = z5 = 0.0;

    if (s.TestConnection(ipaddr, PORT) == false)
    {
        return outputdata;
    }
    else
    {
        for (size_t counter = 0; counter < 10; ++counter)
        {
            cloudnew.reset(new pcl::PointCloud<pcl::PointXYZ>);
            cloudnew = s.CamStream(ipaddr, PORT);

            filteredcloud = c.FilterCloud(cloudnew);
            std::tie(unsortedclusters, clustersize) = c.CloudSegmentation(filteredcloud);

            notorientedclusters = c.SortClusters(unsortedclusters, clustersize);
            clusters = c.RemoveInclined(filteredcloud, notorientedclusters);

            if (clusters.size() > 5)
            {
                limitcluster = 5;
            }
            else
            {
                limitcluster = clusters.size();
            }


            for (int number=0; number<limitcluster; ++number)
            {
                segmented_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
                segmented_cloud->points.resize(clusters[number].indices.size());

                for(size_t i=0; i<clusters[number].indices.size(); ++i)
                {
                    segmented_cloud->points[i].x = (*filteredcloud)[clusters[number].indices[i]].x;
                    segmented_cloud->points[i].y = (*filteredcloud)[clusters[number].indices[i]].y;
                    segmented_cloud->points[i].z = (*filteredcloud)[clusters[number].indices[i]].z;
                }

                std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensions(segmented_cloud);

                if (number == 0)
                {
                    x1 += dimensionX;
                    y1 += dimensionY;
                    z1 += dimensionZ;
                    outputcloud1 = ConvertCloudtoVector(segmented_cloud);
                }
                else if (number == 1)
                {
                    x2 += dimensionX;
                    y2 += dimensionY;
                    z2 += dimensionZ;
                    outputcloud2 = ConvertCloudtoVector(segmented_cloud);
                }
                else if (number == 2)
                {
                    x3 += dimensionX;
                    y3 += dimensionY;
                    z3 += dimensionZ;
                    outputcloud3 = ConvertCloudtoVector(segmented_cloud);
                }
                else if (number == 3)
                {
                    x4 += dimensionX;
                    y4 += dimensionY;
                    z4 += dimensionZ;
                    outputcloud4 = ConvertCloudtoVector(segmented_cloud);
                }
                else if (number == 4)
                {
                    x5 += dimensionX;
                    y5 += dimensionY;
                    z5 += dimensionZ;
                    outputcloud5 = ConvertCloudtoVector(segmented_cloud);
                }
            }
        }

        outputdata.input = ConvertCloudtoVector(cloudnew);
        outputdata.dimensions1.push_back(x1/10);
        outputdata.dimensions1.push_back(y1/10);
        outputdata.dimensions1.push_back(z1/10);

        outputdata.dimensions2.push_back(x2/10);
        outputdata.dimensions2.push_back(y2/10);
        outputdata.dimensions2.push_back(z2/10);

        outputdata.dimensions3.push_back(x3/10);
        outputdata.dimensions3.push_back(y3/10);
        outputdata.dimensions3.push_back(z3/10);

        outputdata.dimensions4.push_back(x4/10);
        outputdata.dimensions4.push_back(y4/10);
        outputdata.dimensions4.push_back(z4/10);

        outputdata.dimensions5.push_back(x5/10);
        outputdata.dimensions5.push_back(y5/10);
        outputdata.dimensions5.push_back(z5/10);

        outputdata.box1 = outputcloud1;
        outputdata.box2 = outputcloud2;
        outputdata.box3 = outputcloud3;
        outputdata.box4 = outputcloud4;
        outputdata.box5 = outputcloud5;

        return outputdata;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<short> ConvertCloudtoVector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
// var
     std::vector<short> points;
     std::size_t nr_points = cloud->points.size();
     std::size_t j = 0;
     const int conversion_factor = 500;
////
     for (std::size_t i = 0; i < nr_points; ++i)
     {
         const pcl::PointXYZ& point = (*cloud)[i]
         points[j*3 + 0] = static_cast<short>(point.x * conversion_factor);
         points[j*3 + 1] = static_cast<short>(point.y * conversion_factor);
         points[j*3 + 2] = static_cast<short>(point.z * conversion_factor);

         ++j;
     }
     points.resize(j*3);
     return points;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
