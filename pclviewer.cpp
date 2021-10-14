#include "pclviewer.h"

using namespace std;

string PCLViewer::Run(char* ipaddr){
  ////////
    Controller c;
    Sensor s;

    coloredinput.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    outputarray.clear();

    volumemean = 0.0;
    mean1 = 0.0;
    mean2 = 0.0;
    mean3 = 0.0;
    mean4 = 0.0;
    mean5 = 0.0;

    x1 = x2 = x3 = x4 = x5 = 0.0;
    y1 = y2 = y3 = y4 = y5 = 0.0;
    z1 = z2 = z3 = z4 = z5 = 0.0;


    if (s.TestConnection(ipaddr, PORT) == false)
    {
        outputarray.push_back("connection failed");

        std::stringstream ss;

        for(size_t i = 0; i < outputarray.size(); ++i)
        {
          if(i != 0)
            ss << ",";
          ss << outputarray[i];
        }

        outputstring = ss.str();
        return outputstring;
    }
    else
    {
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

        X1 = to_string(x1/10);
        X2 = to_string(x2/10);
        X3 = to_string(x3/10);
        X4 = to_string(x4/10);
        X5 = to_string(x5/10);

        Y1 = to_string(y1/10);
        Y2 = to_string(y2/10);
        Y3 = to_string(y3/10);
        Y4 = to_string(y4/10);
        Y5 = to_string(y5/10);

        Z1 = to_string(z1/10);
        Z2 = to_string(z2/10);
        Z3 = to_string(z3/10);
        Z4 = to_string(z4/10);
        Z5 = to_string(z5/10);

        outputarray.push_back("{");
        outputarray.push_back("\"1\": {\"X\": \""+X1+"\", \"Y\": \""+Y1+"\", \"Z\": \""+Z1+"\"},");
        outputarray.push_back("\"2\": {\"X\": \""+X2+"\", \"Y\": \""+Y2+"\", \"Z\": \""+Z2+"\"},");
        outputarray.push_back("\"3\": {\"X\": \""+X3+"\", \"Y\": \""+Y3+"\", \"Z\": \""+Z3+"\"},");
        outputarray.push_back("\"4\": {\"X\": \""+X4+"\", \"Y\": \""+Y4+"\", \"Z\": \""+Z4+"\"},");
        outputarray.push_back("\"5\": {\"X\": \""+X5+"\", \"Y\": \""+Y5+"\", \"Z\": \""+Z5+"\"}");
        outputarray.push_back("}");
//        outputarray.push_back({x2,y2,z2});
//        outputarray.push_back({x3,y3,z3});
//        outputarray.push_back({x4,y4,z4});
//        outputarray.push_back({x5,y5,z5});

        std::stringstream ss;

        for(size_t i = 0; i < outputarray.size(); ++i)
        {
//          if(i != 0)
//            ss << ",";
          ss << outputarray[i];
        }

        outputstring = ss.str();

        return outputstring;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
