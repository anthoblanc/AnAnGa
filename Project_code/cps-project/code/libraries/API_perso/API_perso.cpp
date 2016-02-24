//***************************************************
//                     Define
//***************************************************

//Arduino
#include <AP_HAL.h>
//Own library
#include "../libraries/StandardController/StandardController.h"
#include "../libraries/PID/PID.h"

//String intepretation
#define first_position_num 	3
#define separation_char 	'-'

//PID reference
#define PIDcontroller_Heading		'h'
#define PIDcontroller_Roll		'r'
#define PIDcontroller_Altitude		'a'
#define PIDcontroller_ClimbRate		'c'
#define PIDcontroller_Pitch		'p'
#define PIDcontroller_Speed 		's'

//***************************************************
//                    Function
//***************************************************
void API_print_help() 
{ 
    char str[]=\
"\
e-12-0-1-2: edit PID12 with   \n\
h: help \n\
\0";
    int i=0;
    while (str[i]!='\0')
    {
        hal.console->printf("%c",str[i]);
    }
}

//**********************//
void API_interpretate_chain(char * stringAPI, int length_stringAPI) 
{
	int i=0; //local counter
	char buffer_conv[10]; //buffer for atoi
	int attribut[4]={0};
	int second_position_num=0;
	int third_position_num=0;

	hal.console->printf("The given string is: %s",stringAPI); //test
	
	switch(stringAPI[0]) {
	//*** edit ***//	
	case 'e': 

		if(stringAPI[2]!=separation_char) hal.console->printf("Wrong usage of the function"); break; //if the standart is not respected

		//Kp
		for(i=first_position_num;stringAPI[i]!='-';i++) buffer_conv[i-first_position_num]=stringAPI[i]; //copy the number to convert
		buffer_conv[i-first_position_num]='\0'; //end char
		attribut[1]=atoi(buffer_conv); //convertion to inter

		if(stringAPI[i+1]!=separation_char) hal.console->printf("Wrong usage of the function"); break; //if the standart is not respected

		//Ki
		second_position_num=i+2;
		for(i=second_position_num;stringAPI[i]!='-';i++) buffer_conv[i-second_position_num]=stringAPI[i]; //copy the number to convert
		buffer_conv[i-second_position_num]='\0'; //end char
		attribut[2]=atoi(buffer_conv); //convertion to inter

		if(stringAPI[i+1]!=separation_char) hal.console->printf("Wrong usage of the function"); break; //if the standart is not respected
		
		//Kd
		third_position_num=i+2;
		for(i=third_position_num;stringAPI[i]!='-';i++) buffer_conv[i-third_position_num]=stringAPI[i]; //copy the number to convert
		buffer_conv[i-third_position_num]='\0'; //end char
		attribut[3]=atoi(buffer_conv); //convertion to inter
		
		//Which PID?
		switch(stringAPI[1]) {
		case PIDcontroller_Heading  	: struct PIDs.Heading.setControllerGains(attribut[1],attribut[2],attribut[3]); 		break;
		case PIDcontroller_Roll     	: struct PIDs.Roll.setControllerGains(attribut[1],attribut[2],attribut[3]); 		break;
		case PIDcontroller_Altitude 	: struct PIDs.Altitude.setControllerGains(attribut[1],attribut[2],attribut[3]);		break;
		case PIDcontroller_ClimbRate	: struct PIDs.ClimbRate.setControllerGains(attribut[1],attribut[2],attribut[3]); 	break;
		case PIDcontroller_Pitch    	: struct PIDs.Pitch.setControllerGains(attribut[1],attribut[2],attribut[3]);		break;
		case PIDcontroller_Speed    	: struct PIDs.Speed.setControllerGains(attribut[1],attribut[2],attribut[3]); 		break;
		default:hal.console->printf("Wrong usage of the function"); break; //if the standart is not respected
		}
		break;	
	
	//*** help ***//
	case 'h': 
		API_print_help(); 
		break;
	default: 
		hal.console->printf("The function does not exist or is not operational yet!");
		break;
	}	
}

//**********************//
int atoi(char *str)
{
    int res = 0; // Initialize result
  
    // Iterate through all characters of input string and
    // update result
    for (int i = 0; str[i] != '\0'; ++i)
        res = res*10 + str[i] - '0';
  
    // return result.
    return res;
}
