#include "pclviewer.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include "connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>

using namespace boost;
using boost::asio::ip::tcp;

int main (int argc, char *argv[])
{
/////
    char* ip = argv[1];
    PCLViewer w;
//    ObjectsData outputdata;
/////
    try
    {
      boost::asio::io_service io_service;
      boost::system::error_code ec;
      boost::asio::ip::address ip_address = boost::asio::ip::address::from_string("0.0.0.0", ec);

      tcp::acceptor acceptor(io_service, tcp::endpoint(ip_address, 13));

      for (;;)
      {
        tcp::socket socket(io_service);
        unsigned int nr_points = 0;


        boost::system::error_code ignored_error;
        acceptor.accept(socket);

        ObjectsData outputdata = w.Run(ip);

        boost::asio::write (socket, boost::asio::buffer(&outputdata, sizeof(outputdata)), ignored_error);

//        boost::asio::write (socket, boost::asio::buffer(outputdata.box1.front(), sizeof(outputdata.box1)));
//        boost::asio::write (socket, boost::asio::buffer(outputdata.input.front(), sizeof(outputdata.input)));
//        boost::asio::write (socket, boost::asio::buffer(outputdata.input.front(), sizeof(outputdata.input)));

//        std::string data = w.Run(ip);
//        std::string data = "test";
//        outputdata = w.Run(ip);


//        boost::asio::write(socket, boost::asio::buffer(data), ignored_error);
      }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
