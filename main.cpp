#include "pclviewer.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread/thread.hpp>


using namespace boost;
using boost::asio::ip::tcp;
typedef boost::shared_ptr<tcp::socket> socket_ptr;


void session(socket_ptr sock, char* ip)
{
  PCLViewer w;

  try
  {
    for (;;)
    {
      ObjectsData outputdata = w.Run(ip);

      boost::system::error_code error;
      size_t length = sock->read_some(boost::asio::buffer(outputdata), error);
      if (error == boost::asio::error::eof)
        break; // Connection closed cleanly by peer.
      else if (error)
        throw boost::system::system_error(error); // Some other error.

      boost::asio::write(*sock, boost::asio::buffer(outputdata, length));
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception in thread: " << e.what() << "\n";
  }
}


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
        socket_ptr sock(new tcp::socket (io_service));
        unsigned int nr_points = 0;

        boost::system::error_code ignored_error;
        acceptor.accept(*sock);
        boost::thread t(boost::bind(session, sock, ip));

//        ObjectsData outputdata = w.Run(ip);

//        boost::asio::write (socket, boost::asio::buffer(&outputdata, sizeof(outputdata)), ignored_error);

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
