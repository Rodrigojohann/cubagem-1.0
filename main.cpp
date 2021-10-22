#include "pclviewer.h"
#include <boost/asio.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include "sensor.h"

using namespace boost;
using boost::asio::ip::tcp;

int main (int argc, char *argv[])
{
/////
    char* ip = argv[1];
    PCLViewer w;
    Sensor s;
/////

    if (argc != 2)
    {
      std::cerr << "Usage: insert camera ip" << std::endl;
      return 1;
    }

    if (s.TestConnection(ipaddr, PORT) == false)
    {
        std::cerr << "Could not connect to camera" << std::endl;
        return 1;
    }

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
        std::vector<ObjectsData> sentdata;

        boost::asio::streambuf buf;
        std::ostream os(&buf);
        boost::archive::binary_oarchive oa(os);

        ObjectsData outputdata = w.Run(ip);
        sentdata.push_back(outputdata);

        oa << outputdata;

        boost::system::error_code ignored_error;
        boost::asio::write (socket, buf, ignored_error);
      }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
