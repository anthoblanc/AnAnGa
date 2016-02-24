//***************************************************
//                     Define
//***************************************************

#define FALSE         0
#define TRUE          1

//Arduino
#include <AP_HAL.h>
//Own library
#include "../libraries/StandardController/StandardController.h"
#include "../libraries/PID/PID.h"

//String intepretation
#define first_position_num 	3
#define separation_char 	'-'

//PID reference
#define PIDcontroller_Throttle		't'
#define PIDcontroller_Aileron		'a'
#define PIDcontroller_Rudder 		'r'
#define PIDcontroller_Elevator		'h'

//fly mode
#define takeoff_mod 		0
#define circle_mod 		1
#define looping_mod		2 
#define go_streight_mod		3
#define roll_mod		4


//***************************************************
//                    Function
//***************************************************
void API_print_help() 
{ 
    char str[]=\
"h help \
et-#VAL_P#-#VAL_I#-#VAL_D# edit throttle \n\
ea-#VAL_P#-#VAL_I#-#VAL_D# edit aileron \n \
er-#VAL_P#-#VAL_I#-#VAL_D# edit rudder \n \
ee-#VAL_P#-#VAL_I#-#VAL_D# edit elevator \
#VAL_P# sould be remplace by the value of the PROPORTIONAL gain \n \
#VAL_I# sould be remplace by the value of the INTEGRALE gain \n \
#VAL_D# sould be remplace by the value of the DERIVATE gain \n \
\0";
    int i=0;
    while (str[i]!='\0')
    {
        hal.console->printf("%c",str[i]);
	i++
    }
}

//**********************//
void API_interpretate_chain(char * stringAPI, int length_stringAPI, TrajectoryController& trCTRL, int& Plane_flying_current_state) 
{
	int i=0; //local counter
	char buffer_conv[10]; //buffer for atoi
	float attribut[4]={0};
	int second_position_num=0;
	int third_position_num=0;

	//hal.console->printf("The given string is: %s",stringAPI); //test
	
	switch(stringAPI[0]) {
	//*** edit ***//	
	case 'e': 

		if(stringAPI[2]!=separation_char) hal.console->printf("Wrong usage of the function \n"); break; //if the standart is not respected

		//Kp
		for(i=first_position_num;stringAPI[i]!='-';i++) buffer_conv[i-first_position_num]=stringAPI[i]; //copy the number to convert
		buffer_conv[i-first_position_num]='\0'; //end char
		attribut[1]=atoi(buffer_conv); //convertion to inter

		if(stringAPI[i+1]!=separation_char) hal.console->printf("Wrong usage of the function \n"); break; //if the standart is not respected

		//Ki
		second_position_num=i+2;
		for(i=second_position_num;stringAPI[i]!='-';i++) buffer_conv[i-second_position_num]=stringAPI[i]; //copy the number to convert
		buffer_conv[i-second_position_num]='\0'; //end char
		attribut[2]=atoi(buffer_conv); //convertion to inter

		if(stringAPI[i+1]!=separation_char) hal.console->printf("Wrong usage of the function \n"); break; //if the standart is not respected
		
		//Kd
		third_position_num=i+2;
		for(i=third_position_num;stringAPI[i]!='-';i++) buffer_conv[i-third_position_num]=stringAPI[i]; //copy the number to convert
		buffer_conv[i-third_position_num]='\0'; //end char
		attribut[3]=atoi(buffer_conv); //convertion to inter
		
		//Which PID?
		switch(stringAPI[1]) {
		case PIDcontroller_Throttle 	: trCTRL.editPID(Throttle,attribut[1],attribut[2],attribut[3]);		hal.console->printf("PID modified\n");break;
		case PIDcontroller_Aileron     	: trCTRL.editPID(Aileron,attribut[1],attribut[2],attribut[3]); 		hal.console->printf("PID modified\n");break;
		case PIDcontroller_Rudder 	: trCTRL.editPID(Rudder,attribut[1],attribut[2],attribut[3]);		hal.console->printf("PID modified\n");break;
		case PIDcontroller_Elevator	: trCTRL.editPID(Elevator,attribut[1],attribut[2],attribut[3]); 	hal.console->printf("PID modified\n");break;
		default:hal.console->printf("Wrong usage of the function \n"); break; //if the standart is not respected
		}
		hal.console->printf("PID modified\n");
		break;	
	
	//*** help ***//
	case 'h': 
		API_print_help(); 
		break;
	//*** fly ***//
	case 'f': 
		switch(stringAPI[1]) {
			case 't': Plane_flying_current_state=takeoff_mod; 	hal.console->printf("akeoff_mod actived \n"); 	break;
			case 'c': Plane_flying_current_state=circle_mod; 	hal.console->printf("circle_mod actived \n"); 	break;
			case 'l': Plane_flying_current_state=looping_mod; 	hal.console->printf("looping_mod actived \n"); 	break;
			case 's': Plane_flying_current_state=go_streight_mod; 	hal.console->printf("go_streight_mod actived \n"); break;
			case 'r': Plane_flying_current_state=roll_mod; 		hal.console->printf("roll_mod actived \n"); 	break;
			default: hal.console->printf("Wrong usage of the function \n"); break; //if the standart is not respected
		}
		break;
	default: 
		hal.console->printf("The function does not exist or is not operational yet! \n");
		break;
	}	
}

//**********************//
float atoi(char *str)
{
	float res = 0; // Initialize result
	float store=0; //temp variable
	int nb_after_coma=0;
	// Iterate through all characters of input string and
	// update result
	for (int i = 0; str[i] != '\0'; ++i)
	{
		if(str[i] =='.') nb_after_coma++; //coma detected
		if(nb_after_coma==0) res = res*10 + str[i] - '0'; //before coma
		else 
		{
		store=(str[i] - '0');
		for(int j=0;j<nb_after_coma;j++) store*=0.1; //calculate the digit value in decimal
		res = res + store; //concatain with the value
		nb_after_coma++;
		}
	}
	

	// return result.
	return res;
}
