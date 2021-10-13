#include "pclviewer.h"
#include <boost/asio.hpp>


int main ()
{
  char* ip = "192.168.140.2";

  PCLViewer w;
  w.Run(ip);

}

