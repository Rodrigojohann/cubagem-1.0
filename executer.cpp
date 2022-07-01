﻿#include "executer.h"

using namespace std;

string Executer::Run(){
// var
      Processor  processor;
      Sensor     sensor;
////
      Clean();
      connection = sensor.TestConnection(IP2, PORT);

      if (connection == false)
      {
          outputstring = "connection failed";

          return outputstring;
      }
      else
      {
          cloudnew = sensor.CamStream(IP2, PORT);

          cloudundistorted = processor.RemoveDistortion(cloudnew);
          cloud_preprocessed = processor.PreProcessingCloud(cloudundistorted);
          filteredcloud = processor.FilterROI(cloud_preprocessed, x_min, x_max, y_min, y_max, camheight);

          clusters_indices = processor.CloudSegmentation(filteredcloud);
          std::sort(clusters_indices.begin(), clusters_indices.end(), [](pcl::PointIndices & a, pcl::PointIndices & b){ return a.indices.size() > b.indices.size();});
          clusters = processor.IndicestoClouds(filteredcloud, clusters_indices);

          for (int number=0; number < clusters.size(); ++number)
          {
              hullarea = processor.ConcaveHullArea(processor.ProjectCloud(clusters[number]));
              std::tie(dimensionX, dimensionY, dimensionZ) = processor.CalculateDimensions(clusters[number]);

              featuresvector = processor.ExtractFeatures(clusters[number]);
              featuresvector[0] = camheight - featuresvector[0];
              featuresvectorvector.push_back(featuresvector);
          }

          sumfeaturesvector = processor.ConcatFeatures(featuresvectorvector);
//                processor.SaveFeatures(sumfeaturesvector);

          for (int i=0; i < (sumfeaturesvector.size()); ++i)
          {
              featuresmatrix(i) = sumfeaturesvector[i];
          }

          numberofboxes = round(df(normalizer(featuresmatrix)));
          volumemean = df(normalizer(featuresmatrix))*STANDARDBOXVOLUME;

          numberofboxes_str = to_string(numberofboxes);

//          if (numberofboxes == NUMBEROFBOXES)
//          {
//  //            ConnectTDC((char*)"192.168.136.200:1880/sinaleiro/on");
//          }
//          else
//          {
//  //            ConnectTDC((char*)"192.168.136.200:1880/sinaleiro/off");
//          }

  //        SendJSON(numberofboxes, volumemean, connection, y_min, y_max, x_min, x_max, camheight);

//          outputarray.push_back(numberofboxes_str);

          outputstring = numberofboxes_str;

          return outputstring;
      }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Executer::Clean()
{
    numberofboxes = 0;
    volumemean = 0.0;
    featuresvectorvector.clear();
}