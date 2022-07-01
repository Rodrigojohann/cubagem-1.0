#include "connector.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Connector::ConnectTDC(char *inputurl)
{
// var
    CURL * curl;
    CURLcode res;
    std::string readBuffer;
////
    curl = curl_easy_init();

    if(curl)
    {
      curl_easy_setopt(curl, CURLOPT_URL, inputurl);
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
      res = curl_easy_perform(curl);
      curl_easy_cleanup(curl);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Connector::SendJSON(int count, double volume, bool connection, double ymin, double ymax, double xmin, double xmax, double height)
{
// var
    CURL *hnd;
    struct curl_slist *slist1;
    string jsonstr;
////
    slist1 = NULL;
    slist1 = curl_slist_append(slist1, "Content-Type: application/json");

    jsonstr = "{\"count\": "+to_string(count)+", \"volume\": "+to_string(volume)+", \"status\": {\"connection\": "+to_string(connection)+", \"limiteSuperior\": "+to_string(ymin)+", \"limiteInferior\": "+to_string(ymax)+ ", \"limiteEsquerda\": "+to_string(xmin)+", \"limiteDireita\": "+to_string(xmax)+", \"Altura\": "+to_string(height)+"}}";

    hnd = curl_easy_init();
    curl_easy_setopt(hnd, CURLOPT_URL, "192.168.136.200:1880/status");
    curl_easy_setopt(hnd, CURLOPT_POSTFIELDS, jsonstr.c_str());
    curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, slist1);
    curl_easy_perform(hnd);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t Connector::WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);

    return size * nmemb;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
