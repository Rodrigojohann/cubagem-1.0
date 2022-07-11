#include "executer.h"

int main()
{
/////
    Executer executer;
/////
    try
    {
        executer.Run();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
