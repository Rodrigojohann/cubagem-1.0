#ifndef CONNECTOR_H
#define CONNECTOR_H

#include <curl/curl.h>
#include <iostream>
#include <config.h>
#include <sstream>

class Connector
{
  public:
    void          ConnectTDC(char *inputurl);
    void          SendJSON(int count, double volume, bool connection, double ymin, double ymax, double xmin, double xmax, double height);
    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);
};

#endif // CONNECTOR_H
