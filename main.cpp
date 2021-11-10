#include "pclviewer.h"
#include <boost/asio.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <time.h>

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

    try
    {
      boost::asio::io_service io_service;
      boost::system::error_code ec;
      boost::asio::ip::address ip_address = boost::asio::ip::address::from_string("0.0.0.0", ec);

      tcp::acceptor acceptor(io_service, tcp::endpoint(ip_address, 13));

      for (;;)
      {
;
          tcp::socket socket(io_service);
          acceptor.accept(socket);

          ObjectsData outputdata = w.Run(ip);

          boost::asio::streambuf buf;
          std::ostream os(&buf);
          boost::archive::text_oarchive oa(os);
          oa & outputdata;

          const size_t header = buf.size();
          std::vector<boost::asio::const_buffer> buffers;
          buffers.push_back(buf.data());

          const size_t rc = boost::asio::write(socket, buffers);
          std::cout << "wrote " << rc << " bytes" << std::endl;
      }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
