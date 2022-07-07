#include "regressor.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> Regressor::ExtractFeatures(PointCloudT::Ptr inputcloud)
{
// var
    std::vector<double> outputvector;
    double              centroidz, hullarea;
    PointT              centroid;
    Processor           processor;
////
    pcl::computeCentroid(*inputcloud, centroid);
    centroidz = centroid.z;

    hullarea = processor.ConcaveHullArea(processor.ProjectCloud(inputcloud));

    outputvector.push_back(centroidz);
    outputvector.push_back(hullarea);

    return outputvector;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Regressor::SaveFeatures(std::vector<double> inputvector)
{
// var
    std::ofstream outdata;
////
    outdata.open("./sickboxes_face2_new_cleaned.csv", std::ios_base::app);
    if( !outdata ) { // file couldn't be opened
       std::cerr << "Error: file could not be opened";
       exit(1);
    }
    outdata << "\n";
    for (int i = 0; i < inputvector.size(); ++i)
          outdata << inputvector[i] << ", ";
       outdata.close();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> Regressor::ConcatFeatures(std::vector<std::vector<double>> featuresvectorvector)
{
// var
    std::vector<double> outputvector;
    std::vector<double> majorvector;
    std::vector<double> minorvector;
    std::vector<double> featuresvector;
////
    if (featuresvectorvector.size() > 0)
    {
        outputvector.resize(featuresvectorvector[0].size());
        majorvector.resize(featuresvectorvector[0].size());
        minorvector.resize(featuresvectorvector[0].size());
    //std::cout << "\n\nclusters: " << featuresvectorvector.size();

        majorvector = featuresvectorvector[0];

    //std::cout << "\n\nCentroid: "  << majorvector[0];
    //std::cout << "\nArea: " << majorvector[1];

        for (int i = 1; i < featuresvectorvector.size(); ++i)
        {
            featuresvector = featuresvectorvector[i];

            minorvector[0] += featuresvector[1]*featuresvector[0];
            minorvector[1] += featuresvector[1];

    //std::cout << "\n\nCentroid: "  << featuresvector[0];
    //std::cout << "\nArea: " << featuresvector[1];

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

    //std::cout << "\n\nTotal Centroid: " << outputvector[0];
    //std::cout << "\nTotal Area: " << outputvector[1];
    }

    return outputvector;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

