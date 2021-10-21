#include "pclviewer.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include "connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>

using namespace boost;
using boost::asio::ip::tcp;

namespace s11n_example {

/// Serves stock quote information to any client that connects to it.
class server
{
public:
  /// Constructor opens the acceptor and starts waiting for the first incoming
  /// connection.
  server(boost::asio::io_service& io_service, unsigned short port) : acceptor_(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string("0.0.0.0", ec), port))
  {
    PCLViewer w;
    char* ip = "192.168.140.2";

    ObjectsData outputdata = w.Run(ip);
    sentdata.push_back(outputdata);

    // Start an accept operation for a new connection.
    connection_ptr new_conn(new connection(acceptor_.get_io_service()));
    acceptor_.async_accept(new_conn->socket(), boost::bind(&server::handle_accept, this, boost::asio::placeholders::error, new_conn));
  }

  /// Handle completion of a accept operation.
  void handle_accept(const boost::system::error_code& e, connection_ptr conn)
  {
    if (!e)
    {
      // Successfully accepted a new connection. Send the list of stocks to the
      // client. The connection::async_write() function will automatically
      // serialize the data structure for us.
      conn->async_write(sentdata, boost::bind(&server::handle_write, this, boost::asio::placeholders::error, conn));
    }

    // Start an accept operation for a new connection.
    connection_ptr new_conn(new connection(acceptor_.get_io_service()));
    acceptor_.async_accept(new_conn->socket(),
        boost::bind(&server::handle_accept, this,
          boost::asio::placeholders::error, new_conn));
  }

  /// Handle completion of a write operation.
  void handle_write(const boost::system::error_code& e, connection_ptr conn)
  {
    // Nothing to do. The socket will be closed automatically when the last
    // reference to the connection object goes away.
  }

private:
  /// The acceptor object used to accept incoming socket connections.
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::system::error_code ec;

  /// The data to be sent to each client.
  std::vector<ObjectsData> sentdata;
};

} // namespace s11n_example

int main(int argc, char* argv[])
{
/////
    char* ip = argv[1];
    PCLViewer w;
/////

  try
  {
    // Check command line arguments.
    if (argc != 2)
    {
      std::cerr << "Usage: camera ip as input" << std::endl;
      return 1;
    }
    unsigned short port = boost::lexical_cast<unsigned short>(13);

    boost::asio::io_service io_service;
    s11n_example::server server(io_service, port, ip);
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}



//int main (int argc, char *argv[])
//{
///////
//    char* ip = argv[1];
//    PCLViewer w;
////    ObjectsData outputdata;
///////
//    try
//    {
//      boost::asio::io_service io_service;
//      boost::system::error_code ec;
//      boost::asio::ip::address ip_address = boost::asio::ip::address::from_string("0.0.0.0", ec);

//      tcp::acceptor acceptor(io_service, tcp::endpoint(ip_address, 13));

//      for (;;)
//      {
//        tcp::socket socket(io_service);
//        unsigned int nr_points = 0;


//        boost::system::error_code ignored_error;
//        acceptor.accept(socket);

//        ObjectsData outputdata = w.Run(ip);

//        boost::asio::write (socket, boost::asio::buffer(&outputdata, sizeof(outputdata)), ignored_error);

////        boost::asio::write (socket, boost::asio::buffer(outputdata.box1.front(), sizeof(outputdata.box1)));
////        boost::asio::write (socket, boost::asio::buffer(outputdata.input.front(), sizeof(outputdata.input)));
////        boost::asio::write (socket, boost::asio::buffer(outputdata.input.front(), sizeof(outputdata.input)));

////        std::string data = w.Run(ip);
////        std::string data = "test";
////        outputdata = w.Run(ip);


////        boost::asio::write(socket, boost::asio::buffer(data), ignored_error);
//      }
//    }
//    catch (std::exception& e)
//    {
//        std::cerr << e.what() << std::endl;
//    }

//    return 0;
//}
