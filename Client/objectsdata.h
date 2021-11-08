#ifndef OBJECTSDATA_H
#define OBJECTSDATA_H

#include <string>

using namespace std;

struct ObjectsData {

    std::string connection;
    std::vector<short> input;
    std::vector<double> dimensions1;
    std::vector<short> box1;
    std::vector<double> dimensions2;
    std::vector<short> box2;
    std::vector<double> dimensions3;
    std::vector<short> box3;
    std::vector<double> dimensions4;
    std::vector<short> box4;
    std::vector<double> dimensions5;
    std::vector<short> box5;

    template<typename archive> void serialize(archive& ar, const unsigned /*version*/) {
        ar & connection;
        ar & input;
        ar & dimensions1;
        ar & box1;
        ar & dimensions2;
        ar & box2;
        ar & dimensions3;
        ar & box3;
        ar & dimensions4;
        ar & box4;
        ar & dimensions5;
        ar & box5;
    }

};

#endif // OBJECTSDATA_H
