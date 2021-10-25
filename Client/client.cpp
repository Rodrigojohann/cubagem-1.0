#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <../objectsdata.h>

using boost::asio::ip::tcp;

//struct ObjectsData {

//    std::string connection;
//    std::vector<short> input;
//    std::vector<double> dimensions1;
//    std::vector<short> box1;
//    std::vector<double> dimensions2;
//    std::vector<short> box2;
//    std::vector<double> dimensions3;
//    std::vector<short> box3;
//    std::vector<double> dimensions4;
//    std::vector<short> box4;
//    std::vector<double> dimensions5;
//    std::vector<short> box5;

//    template<typename archive> void serialize(archive& ar, const unsigned /*version*/) {
//        ar & connection;
//        ar & input;
//        ar & dimensions1;
//        ar & box1;
//        ar & dimensions2;
//        ar & box2;
//        ar & dimensions3;
//        ar & box3;
//        ar & dimensions4;
//        ar & box4;
//        ar & dimensions5;
//        ar & box5;
//    }

//};


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

        std::istream is(&buf);
        boost::archive::binary_iarchive ia(is);

        ia >> readdata;

        std::cout << "  Connection: " << readdata.connection << "\n";
        std::cout << "  Input cloud size: " << readdata.input.size() << "\n";
        std::cout << "  Object 1 cloud size: " << readdata.box1.size() << "\n";
        std::cout << "  Object 1 dimensions: X: " << readdata.dimensions1[0] << "; Y: " << readdata.dimensions1[1] << "; Z: " << readdata.dimensions1[2] << "\n";
        std::cout << "  Object 2 cloud size: " << readdata.box2.size() << "\n";
        std::cout << "  Object 2 dimensions: X: " << readdata.dimensions2[0] << "; Y: " << readdata.dimensions2[1] << "; Z: " << readdata.dimensions2[2] << "\n";
        std::cout << "  Object 3 cloud size: " << readdata.box3.size() << "\n";
        std::cout << "  Object 3 dimensions: X: " << readdata.dimensions3[0] << "; Y: " << readdata.dimensions3[1] << "; Z: " << readdata.dimensions3[2] << "\n";
        std::cout << "  Object 4 cloud size: " << readdata.box4.size() << "\n";
        std::cout << "  Object 4 dimensions: X: " << readdata.dimensions4[0] << "; Y: " << readdata.dimensions4[1] << "; Z: " << readdata.dimensions4[2] << "\n";
        std::cout << "  Object 5 cloud size: " << readdata.box5.size() << "\n";
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
