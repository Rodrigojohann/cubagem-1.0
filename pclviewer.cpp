#include "pclviewer.h"

using namespace std;

string PCLViewer::Run(char* ipaddr){
    ////////
      Controller c;
      Sensor     s;

      CleanWindow();
      connection = s.TestConnection(IP2, PORT);

      if (connection == false)
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
          for (size_t counter = 0; counter < Nsamples; ++counter)
          {
              cloudnew = s.CamStream(IP2, PORT);
  //            cloudnew.reset(new PointCloudI);
  //            pcl::io::loadPCDFile<PointI> ("1651606523.pcd", *cloudnew);

              cloudundistorted = s.RemoveDistortion(cloudnew);
              cloud_preprocessed = c.PreProcessingCloud(cloudundistorted);
              if (cloud_preprocessed->points.size() > 100)
              {
                  filteredcloud = c.FilterROI(cloud_preprocessed, x_min, x_max, y_min, y_max, camheight);

                  notorientedclusters = c.CloudSegmentation(filteredcloud);
                  std::sort(notorientedclusters.begin(), notorientedclusters.end(), [](pcl::PointIndices & a, pcl::PointIndices & b){ return a.indices.size() > b.indices.size();});
                  clusters = c.IndicestoClouds(filteredcloud, notorientedclusters);

                  if (clusters.size() > 10)
                  {
                      limitcluster = 10;
                  }
                  else
                  {
                      limitcluster = clusters.size();
                  }

                  totalvolume = 0.0;
                  objvolume = 0.0;

                  for (int number=0; number < limitcluster; ++number)
                  {
                      hullarea = c.ConcaveHullArea(c.ProjectCloud(clusters[number]));
                      std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensions(clusters[number]);

                      featuresvector = c.ExtractFeatures(clusters[number]);
                      ////
                      featuresvector[0] = camheight - featuresvector[0];
                      ////
                      featuresvectorvector.push_back(featuresvector);

                      if (dimensionZ > 5)
                      {
                          sumsizes += clusters[number]->points.size();
                          sumZ += dimensionZ*(clusters[number]->points.size());
                      }
                  }
                  sumfeaturesvector = c.ConcatFeatures(featuresvectorvector);
  //                c.SaveFeatures(sumfeaturesvector);

                  for (int i=0; i < (sumfeaturesvector.size()); ++i)
                  {
                      featuresmatrix(i) = sumfeaturesvector[i];
                  }

                  numberofboxes = round(df(normalizer(featuresmatrix)));
                  volumemean = df(normalizer(featuresmatrix))*STANDARDBOXVOLUME;
              }
          }

          numberofboxes_str = to_string(numberofboxes);

          if (numberofboxes == NUMBEROFBOXES)
          {
  //            ConnectTDC((char*)"192.168.136.200:1880/sinaleiro/on");
          }
          else
          {
  //            ConnectTDC((char*)"192.168.136.200:1880/sinaleiro/off");
          }

  //        SendJSON(numberofboxes, volumemean, connection, y_min, y_max, x_min, x_max, camheight);

          outputarray.push_back("{");

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
void PCLViewer::Clean()
{
    cloud_hull.reset(new PointCloudT);

    if (firstCall == true)
    {
        firstCall = false;
    }

    numberofboxes = 0;
    volumemean = 0.0;
    sumsizes = 0;
    sumZ = 0;
    featuresvectorvector.clear();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::ConnectTDC(char *inputurl)
{
// var
    CURL * curl;
    CURLcode res;
    std::string readBuffer;
////
    curl = curl_easy_init();

    if(curl)
    {
      curl_easy_setopt(curl, CURLOPT_URL, inputurl);
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
      res = curl_easy_perform(curl);
      curl_easy_cleanup(curl);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::SendJSON(int count, double volume, bool connection, double ymin, double ymax, double xmin, double xmax, double height)
{
// var
    CURL *hnd;
    struct curl_slist *slist1;
    string jsonstr;
////
    slist1 = NULL;
    slist1 = curl_slist_append(slist1, "Content-Type: application/json");

    jsonstr = "{\"count\": "+to_string(count)+", \"volume\": "+to_string(volume)+", \"status\": {\"connection\": "+to_string(connection)+", \"limiteSuperior\": "+to_string(ymin)+", \"limiteInferior\": "+to_string(ymax)+ ", \"limiteEsquerda\": "+to_string(xmin)+", \"limiteDireita\": "+to_string(xmax)+", \"Altura\": "+to_string(height)+"}}";

    hnd = curl_easy_init();
    curl_easy_setopt(hnd, CURLOPT_URL, "192.168.136.200:1880/status");
    curl_easy_setopt(hnd, CURLOPT_POSTFIELDS, jsonstr.c_str());
    curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, slist1);
    curl_easy_perform(hnd);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t PCLViewer::WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);

    return size * nmemb;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
