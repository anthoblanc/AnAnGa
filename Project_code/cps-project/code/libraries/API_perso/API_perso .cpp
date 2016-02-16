#include <fstream>
#include <string>

//Arduino
#include <AP_HAL.h>

// **** Function **** //
void print_help() 
{ 
    std::ifstream file("HELP_API.txt");
    std::string str; 
    while (std::getline(file, str))
    {
        hal.console->printf("%s",std);
    }
}

void API_interpretate_chain(char * stringAPI, int length_stringAPI) 
{

}
