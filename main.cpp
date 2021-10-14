#include "pclviewer.h"
#include <boost/asio.hpp>
#include <boost/serialization/vector.hpp>

using namespace boost;
using boost::asio::ip::tcp;


int main ()
{
/////
    char* ip = "192.168.140.2";
    PCLViewer w;
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
        acceptor.accept(socket);

//        std::string message = make_daytime_string();

        std::string data = w.Run(ip);

        boost::system::error_code ignored_error;
        boost::asio::write(socket, boost::asio::buffer(data), ignored_error);
      }
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }

    return 0;



//  w.Run(ip);

}


