#include "executer.h"

int main()
{
/////
    Executer executer;
/////
    try
    {
      for (;;)
      {
        std::string data = executer.Run();
        std::cout << data;
      }
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }

    return 0;
}
