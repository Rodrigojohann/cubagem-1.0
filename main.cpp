#include "pclviewer.h"
#include <boost/asio.hpp>


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
      tcp::socket socket(io_service);

      for (;;)
      {
        unsigned int nr_points = 0;
        ObjectsData outputdata = w.Run(ip);

        nr_points = static_cast<unsigned int> (11/3);

        boost::asio::write (socket, boost::asio::buffer (&nr_points, sizeof (nr_points)));

        acceptor.accept(socket);

        boost::asio::write (socket, boost::asio::buffer(&outputdata.input.front(), nr_points * 3 * sizeof (short)));

//        std::string data = w.Run(ip);
        std::string data = "test";
//        outputdata = w.Run(ip);

        boost::system::error_code ignored_error;
        boost::asio::write(socket, boost::asio::buffer(data), ignored_error);
      }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
