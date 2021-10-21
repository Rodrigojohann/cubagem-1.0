//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2016 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <boost/bind.hpp>
#include <iostream>
#include <vector>
#include "../connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>


using boost::asio::ip::tcp;


struct ObjectsData {

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


namespace s11n_example {

/// Downloads stock quote information from a server.
class client
{
public:
  /// Constructor starts the asynchronous connect operation.
  client(boost::asio::io_service& io_service, const std::string& host, const std::string& service) : connection_(io_service)
  {
    // Resolve the host name into an IP address.
    boost::asio::ip::tcp::resolver resolver(io_service);
    boost::asio::ip::tcp::resolver::query query(host, service);
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    // Start an asynchronous connect operation.
    boost::asio::async_connect(connection_.socket(), endpoint_iterator, boost::bind(&client::handle_connect, this, boost::asio::placeholders::error));
  }

  /// Handle completion of a connect operation.
  void handle_connect(const boost::system::error_code& e)
  {
    if (!e)
    {
      // Successfully established connection. Start operation to read the list
      // of stocks. The connection::async_read() function will automatically
      // decode the data that is read from the underlying socket.
      connection_.async_read(readdata, boost::bind(&client::handle_read, this, boost::asio::placeholders::error));
    }
    else
    {
      // An error occurred. Log it and return. Since we are not starting a new
      // operation the io_service will run out of work to do and the client will
      // exit.
      std::cerr << e.message() << std::endl;
    }
  }

  /// Handle completion of a read operation.
  void handle_read(const boost::system::error_code& e)
  {
    if (!e)
    {
      // Print out the data that was received.
      for (std::size_t i = 0; i < readdata.size(); ++i)
      {
        std::cout << "Data number " << i+1 << "\n";
        std::cout << "  Input cloud size: " << readdata[i].input.size() << "\n";
        std::cout << "  Object 1 cloud size: " << readdata[i].box1.size() << "\n";
        std::cout << "  Object 1 dimensions: X: " << readdata[i].dimensions1[0] << "; Y: " << readdata[i].dimensions1[1] << "; Z: " << readdata[i].dimensions1[2] << "\n";
        std::cout << "  Object 2 cloud size: " << readdata[i].box2.size() << "\n";
        std::cout << "  Object 2 dimensions: X: " << readdata[i].dimensions2[0] << "; Y: " << readdata[i].dimensions2[1] << "; Z: " << readdata[i].dimensions2[2] << "\n";
        std::cout << "  Object 3 cloud size: " << readdata[i].box3.size() << "\n";
        std::cout << "  Object 3 dimensions: X: " << readdata[i].dimensions3[0] << "; Y: " << readdata[i].dimensions3[1] << "; Z: " << readdata[i].dimensions3[2] << "\n";
        std::cout << "  Object 4 cloud size: " << readdata[i].box4.size() << "\n";
        std::cout << "  Object 4 dimensions: X: " << readdata[i].dimensions4[0] << "; Y: " << readdata[i].dimensions4[1] << "; Z: " << readdata[i].dimensions4[2] << "\n";
        std::cout << "  Object 5 cloud size: " << readdata[i].box5.size() << "\n";
        std::cout << "  Object 5 dimensions: X: " << readdata[i].dimensions5[0] << "; Y: " << readdata[i].dimensions5[1] << "; Z: " << readdata[i].dimensions5[2] << "\n";
      }
    }
    else
    {
      // An error occurred.
      std::cerr << e.message() << std::endl;
    }

    // Since we are not starting a new operation the io_service will run out of
    // work to do and the client will exit.
  }

private:
  /// The connection to the server.
  connection connection_;

  /// The data received from the server.
  std::vector<ObjectsData> readdata;
};

} // namespace s11n_example

int main(int argc, char* argv[])
{
  try
  {
    // Check command line arguments.
    if (argc != 2)
    {
      std::cerr << "Usage: client <host> " << std::endl;
      return 1;
    }

    boost::asio::io_service io_service;

    s11n_example::client client(io_service, argv[1], "13");
    io_service.run();

  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}





//int main(int argc, char* argv[])
//{
//  try
//  {
//    if (argc != 2)
//    {
//      std::cerr << "Usage: client <host>" << std::endl;
//      return 1;
//    }

//    boost::asio::io_service io_service;

//    tcp::resolver resolver(io_service);
//    tcp::resolver::query query(argv[1], "13");
//    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

//    tcp::socket socket(io_service);
//    boost::asio::connect(socket, endpoint_iterator);

//    for (;;)
//    {
//      std::vector<ObjectsData> readdata;
//      boost::system::error_code error;

//      size_t len = socket.read_some(boost::asio::buffer(readdata), error);

//      if (error == boost::asio::error::eof)
//        break; // Connection closed cleanly by peer.
//      else if (error)
//        throw boost::system::system_error(error); // Some other error.

//          if (!error)
//          {
//            // Print out the data that was received.
//            for (std::size_t i = 0; i < readdata.size(); ++i)
//            {
//              std::cout << "Data number " << i+1 << "\n";
//              std::cout << "  Input cloud size: " << readdata[i].input.size() << "\n";
//              std::cout << "  Object 1 cloud size: " << readdata[i].box1.size() << "\n";
//              std::cout << "  Object 1 dimensions: X: " << readdata[i].dimensions1[0] << "; Y: " << readdata[i].dimensions1[1] << "; Z: " << readdata[i].dimensions1[2] << "\n";
//              std::cout << "  Object 2 cloud size: " << readdata[i].box2.size() << "\n";
//              std::cout << "  Object 2 dimensions: X: " << readdata[i].dimensions2[0] << "; Y: " << readdata[i].dimensions2[1] << "; Z: " << readdata[i].dimensions2[2] << "\n";
//              std::cout << "  Object 3 cloud size: " << readdata[i].box3.size() << "\n";
//              std::cout << "  Object 3 dimensions: X: " << readdata[i].dimensions3[0] << "; Y: " << readdata[i].dimensions3[1] << "; Z: " << readdata[i].dimensions3[2] << "\n";
//              std::cout << "  Object 4 cloud size: " << readdata[i].box4.size() << "\n";
//              std::cout << "  Object 4 dimensions: X: " << readdata[i].dimensions4[0] << "; Y: " << readdata[i].dimensions4[1] << "; Z: " << readdata[i].dimensions4[2] << "\n";
//              std::cout << "  Object 5 cloud size: " << readdata[i].box5.size() << "\n";
//              std::cout << "  Object 5 dimensions: X: " << readdata[i].dimensions5[0] << "; Y: " << readdata[i].dimensions5[1] << "; Z: " << readdata[i].dimensions5[2] << "\n";
//            }
//          }
//          else
//          {
//            // An error occurred.
//            std::cerr << error.message() << std::endl;
//          }
//    }
//  }
//  catch (std::exception& e)
//  {
//    std::cerr << e.what() << std::endl;
//  }

//  return 0;
//}
