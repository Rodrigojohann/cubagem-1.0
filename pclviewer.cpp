#include "pclviewer.h"

using namespace std;

void PCLViewer::Run(char* ipaddr){
  ////////
    Controller c;
    Sensor s;

    coloredinput.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    volumemean = 0.0;
    mean1 = 0.0;
    mean2 = 0.0;
    mean3 = 0.0;
    mean4 = 0.0;
    mean5 = 0.0;

    if (s.TestConnection(ipaddr, PORT) == True)
    {
        cout << "\n True \n\n";
    }

    for (size_t counter = 0; counter < 10; ++counter)
    {
        cloudnew.reset(new pcl::PointCloud<pcl::PointXYZ>);
        coloredinput.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

        cloudnew = s.CamStream(ipaddr, PORT);
        coloredinput->points.resize(cloudnew->points.size());

        if (cloudnew->points.size() > 100)
        {
            for (size_t i = 0; i < coloredinput->points.size(); i++)
            {
                coloredinput->points[i].x = (*cloudnew)[i].x;
                coloredinput->points[i].y = (*cloudnew)[i].y;
                coloredinput->points[i].z = (*cloudnew)[i].z;
                coloredinput->points[i].r = 255;
                coloredinput->points[i].g = 255;
                coloredinput->points[i].b = 255;
                coloredinput->points[i].a = 200;
            }

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

            totalvolume = 0;
            objvolume = 0;

            coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

            for (int number=0; number<limitcluster; ++number)
            {
                segmented_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
                coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
                coloredcloud->points.resize(clusters[number].indices.size());
                segmented_cloud->points.resize(clusters[number].indices.size());

                for(size_t i=0; i<clusters[number].indices.size(); ++i)
                {
                    segmented_cloud->points[i].x = (*filteredcloud)[clusters[number].indices[i]].x;
                    segmented_cloud->points[i].y = (*filteredcloud)[clusters[number].indices[i]].y;
                    segmented_cloud->points[i].z = (*filteredcloud)[clusters[number].indices[i]].z;

                    coloredcloud->points[i].x = (*filteredcloud)[clusters[number].indices[i]].x;
                    coloredcloud->points[i].y = (*filteredcloud)[clusters[number].indices[i]].y;
                    coloredcloud->points[i].z = (*filteredcloud)[clusters[number].indices[i]].z;

                    coloredcloud->points[i].r = cloudcolor[number][0];
                    coloredcloud->points[i].g = cloudcolor[number][1];
                    coloredcloud->points[i].b = cloudcolor[number][2];
                    coloredcloud->points[i].a = 255;
                }

                hullarea = c.SurfaceArea(segmented_cloud);
                std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensions(segmented_cloud);

                objvolume = hullarea*dimensionZ/100;
                totalvolume += objvolume;

                VolStr = to_string(objvolume).substr(0,5)+" m³";

                if (number == 0)
                {
                    mean1 += objvolume;
                }
                else if (number == 1)
                {
                    mean2 += objvolume;
                }
                else if (number == 2)
                {
                    mean3 += objvolume;
                }
                else if (number == 3)
                {
                    mean4 += objvolume;
                }
                else if (number == 4)
                {
                    mean5 += objvolume;
                }
            }
        }
        volumemean += totalvolume;
    }

    Vol1 = to_string(mean1/10).substr(0,5)+" m³";
    Vol2 = to_string(mean2/10).substr(0,5)+" m³";
    Vol3 = to_string(mean3/10).substr(0,5)+" m³";
    Vol4 = to_string(mean4/10).substr(0,5)+" m³";
    Vol5 = to_string(mean5/10).substr(0,5)+" m³";
    TotalStr = to_string(volumemean/10).substr(0,5)+" m³";

    cout << "Volume: " << TotalStr;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::ConnectDevice()
{
    cout << "test";
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
