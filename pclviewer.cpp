#include "pclviewer.h"

using namespace std;

double PCLViewer::Run(char* ipaddr){
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

    x1 = x2 = x3 = x4 = x5 = 0.0;
    y1 = y2 = y3 = y4 = y5 = 0.0;
    z1 = z2 = z3 = z4 = z5 = 0.0;


    if (s.TestConnection(ipaddr, PORT) == true)
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

                //objvolume = hullarea*dimensionZ/100;
                objvolume = dimensionX*dimensionY*dimensionZ;
                totalvolume += objvolume;

                if (number == 0)
                {
                    mean1 += objvolume;
                    x1 += dimensionX;
                    y1 += dimensionY;
                    z1 += dimensionZ;
                }
                else if (number == 1)
                {
                    mean2 += objvolume;
                    x2 += dimensionX;
                    y2 += dimensionY;
                    z2 += dimensionZ;
                }
                else if (number == 2)
                {
                    mean3 += objvolume;
                    x3 += dimensionX;
                    y3 += dimensionY;
                    z3 += dimensionZ;
                }
                else if (number == 3)
                {
                    mean4 += objvolume;
                    x4 += dimensionX;
                    y4 += dimensionY;
                    z4 += dimensionZ;
                }
                else if (number == 4)
                {
                    mean5 += objvolume;
                    x5 += dimensionX;
                    y5 += dimensionY;
                    z5 += dimensionZ;
                }
            }
        }
        volumemean += totalvolume;
    }

    mean1 = mean1/10;
    mean2 = mean2/10;
    mean3 = mean3/10;
    mean4 = mean4/10;
    mean5 = mean5/10;
    volumemean = volumemean/10;

//    x1 = x1/10;
//    x2 = x2/10;
//    x3 = x3/10;
//    x4 = x4/10;
//    x5 = x5/10;

//    y1 = y1/10;
//    y2 = y2/10;
//    y3 = y3/10;
//    y4 = y4/10;
//    y5 = y5/10;

//    z1 = z1/10;
//    z2 = z2/10;
//    z3 = z3/10;
//    z4 = z4/10;
//    z5 = z5/10;

    outputarray[0][0] = x1/10;
    outputarray[0][1] = x2/10;
    outputarray[0][2] = x3/10;
    outputarray[0][3] = x4/10;
    outputarray[0][4] = x5/10;

    outputarray[1][0] = y1/10;
    outputarray[1][1] = y2/10;
    outputarray[1][2] = y3/10;
    outputarray[1][3] = y4/10;
    outputarray[1][4] = y5/10;

    outputarray[2][0] = z1/10;
    outputarray[2][1] = z2/10;
    outputarray[2][2] = z3/10;
    outputarray[2][3] = z4/10;
    outputarray[2][4] = z5/10;

    return outputarray;

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
