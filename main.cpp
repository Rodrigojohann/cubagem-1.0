#include "pclviewer.h"
#include <boost/asio.hpp>

using namespace boost;
using boost::asio::ip::tcp;


int main ()
{
/////
    char* ip = "192.168.140.2";
    PCLViewer w;
    double outputarray[5][3];
/////
    try
    {
      boost::asio::io_service io_service;
      boost::system::error_code ec;
      boost::asio::ip::address ip_address = boost::asio::ip::address::from_string("127.0.0.1", ec);

      tcp::acceptor acceptor(io_service, tcp::endpoint(ip_address, 13));

      for (;;)
      {
        tcp::socket socket(io_service);
        acceptor.accept(socket);

//        std::string message = make_daytime_string();

        boost::system::error_code ignored_error;
        boost::asio::write(socket, boost::asio::buffer(w.Run(ip)), ignored_error);
      }
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }

    return 0;



//  w.Run(ip);

}


