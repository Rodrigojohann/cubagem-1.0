#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <objectsdata.h>
#include "PointXYZ.h"

using boost::asio::ip::tcp;

std::vector<PointXYZ> ConvertVectortoCloud(std::vector<short> inputvector)
{
// var
     std::vector<PointXYZ> points;
     std::size_t nr_points = inputvector.size()/3;
     std::size_t j = 0;
     const int conversion_factor = 500;
////
     points.resize(nr_points);

     for (std::size_t i = 0; i < nr_points; ++i)
     {
         points[i].x = inputvector[i*3 + 0]/conversion_factor;
         points[i].y = inputvector[i*3 + 1]/conversion_factor;
         points[i].z = inputvector[i*3 + 2]/conversion_factor;
     }
     return points;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage: client <host>" << std::endl;
      return 1;
    }

    boost::asio::io_service io_service;

    tcp::resolver resolver(io_service);
    tcp::resolver::query query(argv[1], "13");
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    tcp::socket socket(io_service);
    boost::asio::connect(socket, endpoint_iterator);

    for (;;)
    {
        ObjectsData readdata;
        boost::asio::streambuf buf;
        boost::system::error_code error;

        std::size_t length = boost::asio::read(socket, buf, error);
        std::cout << "read " << length << " bytes" << std::endl;

        std::istream is(&buf);
        boost::archive::text_iarchive ia(is);

        ia >> readdata;

        std::cout << "  Connection: " << readdata.connection << "\n";
        std::cout << "  Input cloud size: " << ConvertVectortoCloud(readdata.input).size() << "\n";
        std::cout << "  Object 1 cloud size: " << ConvertVectortoCloud(readdata.box1).size() << "\n";
        std::cout << "  Object 1 dimensions: X: " << readdata.dimensions1[0] << "; Y: " << readdata.dimensions1[1] << "; Z: " << readdata.dimensions1[2] << "\n";
        std::cout << "  Object 2 cloud size: " << ConvertVectortoCloud(readdata.box2).size() << "\n";
        std::cout << "  Object 2 dimensions: X: " << readdata.dimensions2[0] << "; Y: " << readdata.dimensions2[1] << "; Z: " << readdata.dimensions2[2] << "\n";
        std::cout << "  Object 3 cloud size: " << ConvertVectortoCloud(readdata.box3).size() << "\n";
        std::cout << "  Object 3 dimensions: X: " << readdata.dimensions3[0] << "; Y: " << readdata.dimensions3[1] << "; Z: " << readdata.dimensions3[2] << "\n";
        std::cout << "  Object 4 cloud size: " << ConvertVectortoCloud(readdata.box4).size() << "\n";
        std::cout << "  Object 4 dimensions: X: " << readdata.dimensions4[0] << "; Y: " << readdata.dimensions4[1] << "; Z: " << readdata.dimensions4[2] << "\n";
        std::cout << "  Object 5 cloud size: " << ConvertVectortoCloud(readdata.box5).size() << "\n";
        std::cout << "  Object 5 dimensions: X: " << readdata.dimensions5[0] << "; Y: " << readdata.dimensions5[1] << "; Z: " << readdata.dimensions5[2] << "\n";

        if (error == boost::asio::error::eof)
            break; // Connection closed cleanly by peer.
        else if (error)
            throw boost::system::system_error(error); // Some other error.
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
