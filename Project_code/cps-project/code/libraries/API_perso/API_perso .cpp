#include <fstream>
#include <string>

//Arduino
#include <AP_HAL.h>

// **** Function **** //
void API_print_help() 
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
	hal.console->printf("The given string is: %s",stringAPI);
	if(stringAPI::compare("help")=0)
	{
		print_help();
	}
	else
	hal.console->printf("The function does not exist or is not operational yet!");

}
