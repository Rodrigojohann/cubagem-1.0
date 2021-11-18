#include "pclviewer.h"

using namespace std;

ObjectsData PCLViewer::Run(char* ipaddr){
// var
    Controller c;
    Sensor s;
    ObjectsData outputdata;
///
    x1 = x2 = x3 = x4 = x5 = 0.0;
    y1 = y2 = y3 = y4 = y5 = 0.0;
    z1 = z2 = z3 = z4 = z5 = 0.0;

    if (s.TestConnection(ipaddr, PORT) == false)
    {
        outputdata.connection = "connection failed";
        outputdata.dimensions1.push_back(0);
        outputdata.dimensions1.push_back(0);
        outputdata.dimensions1.push_back(0);

        outputdata.dimensions2.push_back(0);
        outputdata.dimensions2.push_back(0);
        outputdata.dimensions2.push_back(0);

        outputdata.dimensions3.push_back(0);
        outputdata.dimensions3.push_back(0);
        outputdata.dimensions3.push_back(0);

        outputdata.dimensions4.push_back(0);
        outputdata.dimensions4.push_back(0);
        outputdata.dimensions4.push_back(0);

        outputdata.dimensions5.push_back(0);
        outputdata.dimensions5.push_back(0);
        outputdata.dimensions5.push_back(0);

        return outputdata;
    }
    else
    {
        for (size_t counter = 0; counter < Nsamples; ++counter)
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

                for(size_t i=0; i < clusters[number].indices.size(); ++i)
                {
                    segmented_cloud->points[i].x = (*filteredcloud)[clusters[number].indices[i]].x;
                    segmented_cloud->points[i].y = (*filteredcloud)[clusters[number].indices[i]].y;
                    segmented_cloud->points[i].z = (*filteredcloud)[clusters[number].indices[i]].z;
                }

                std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensions(segmented_cloud);

                switch (number){
                  case 0:
                    x1 += dimensionX;
                    y1 += dimensionY;
                    z1 += dimensionZ;
                    if (counter == (Nsamples-1)){
                        outputdata.box1 = ConvertCloudtoVector(segmented_cloud);
                    }
                    break;
                  case 1:
                    x2 += dimensionX;
                    y2 += dimensionY;
                    z2 += dimensionZ;
                    if (counter == (Nsamples-1)){
                        outputdata.box2 = ConvertCloudtoVector(segmented_cloud);
                    }
                    break;
                  case 2:
                    x3 += dimensionX;
                    y3 += dimensionY;
                    z3 += dimensionZ;
                    if (counter == (Nsamples-1)){
                        outputdata.box3 = ConvertCloudtoVector(segmented_cloud);
                    }
                    break;
                  case 3:
                    x4 += dimensionX;
                    y4 += dimensionY;
                    z4 += dimensionZ;
                    if (counter == (Nsamples-1)){
                        outputdata.box4 = ConvertCloudtoVector(segmented_cloud);
                    }
                    break;
                  case 4:
                    x5 += dimensionX;
                    y5 += dimensionY;
                    z5 += dimensionZ;
                    if (counter == (Nsamples-1)){
                        outputdata.box5 = ConvertCloudtoVector(segmented_cloud);
                    }
                    break;
                  }
            }
        }

        outputdata.connection = "connection succeeded";
        outputdata.input = ConvertCloudtoVector(cloudnew);
        outputdata.dimensions1.push_back(x1/Nsamples);
        outputdata.dimensions1.push_back(y1/Nsamples);
        outputdata.dimensions1.push_back(z1/Nsamples);

        outputdata.dimensions2.push_back(x2/Nsamples);
        outputdata.dimensions2.push_back(y2/Nsamples);
        outputdata.dimensions2.push_back(z2/Nsamples);

        outputdata.dimensions3.push_back(x3/Nsamples);
        outputdata.dimensions3.push_back(y3/Nsamples);
        outputdata.dimensions3.push_back(z3/Nsamples);

        outputdata.dimensions4.push_back(x4/Nsamples);
        outputdata.dimensions4.push_back(y4/Nsamples);
        outputdata.dimensions4.push_back(z4/Nsamples);

        outputdata.dimensions5.push_back(x5/Nsamples);
        outputdata.dimensions5.push_back(y5/Nsamples);
        outputdata.dimensions5.push_back(z5/Nsamples);

        return outputdata;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<short> PCLViewer::ConvertCloudtoVector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
// var
     std::vector<short> points;
     std::size_t nr_points = cloud->points.size();
     std::size_t j = 0;
     const int conversion_factor = 500;
////
     points.resize(nr_points*3);
     for (std::size_t i = 0; i < nr_points; ++i)
     {
         points[j*3 + 0] = static_cast<short>((*cloud)[i].x * conversion_factor);
         points[j*3 + 1] = static_cast<short>((*cloud)[i].y * conversion_factor);
         points[j*3 + 2] = static_cast<short>((*cloud)[i].z * conversion_factor);

         ++j;
     }
     return points;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
