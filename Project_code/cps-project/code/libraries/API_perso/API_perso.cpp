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
#define circle_mod 			1
#define looping_mod			2 
#define go_streight_mod		3
#define roll_mod			4
#define back_glide_mod		5
#define half_circle_mod		6
#define upclimb_mod			7
#define snake_mod 			8


//***************************************************
//                    Function
//***************************************************
void API_print_help() 
{ 
    char str[]=\
"h help \n\
et-#VAL_P#-#VAL_I#-#VAL_D# edit throttle \n\
ea-#VAL_P#-#VAL_I#-#VAL_D# edit aileron \n\
er-#VAL_P#-#VAL_I#-#VAL_D# edit rudder \n\
ee-#VAL_P#-#VAL_I#-#VAL_D# edit elevator \n\
  #VAL_P# sould be remplace by the value of the PROPORTIONAL gain \n\
  #VAL_I# sould be remplace by the value of the INTEGRALE gain \n\
  #VAL_D# sould be remplace by the value of the DERIVATE gain \n\
ft takeoff_mod \n\
fc circle_mod \n\
fl looping_mod \n\
fs go_streight_mod \n\
fr roll_mod \n\
fb back_glide_mod \n\
fh half_circle_mod \n\
\0";
    int i=0;
    while (str[i]!='\0')
    {
        hal.console->printf("%c",str[i]);
	i++;
    }
}

//**********************//
void API_interpretate_chain(char * stringAPI, int length_stringAPI, TrajectoryController& trCTRL, int& Plane_flying_current_state, float& desiredL) 
{
	int i=0; //local counter
	char buffer_conv[10]; //buffer for atof
        float attribut[4];
	int second_position_num=0;
	int third_position_num=0;

	//hal.console->printf("The given string is: %s",stringAPI); //test
	
	switch(stringAPI[0]) {
	//*** edit ***//	
	case 'e': 
		/*
		Note: the 3 tests for the correct syntax may be desactiveted for a better performance!
		*/		

		//if(stringAPI[2]!=separation_char) { hal.console->printf("Wrong usage of the function \n"); break;}//if the standart is not respected

		//Kp
		for(i=first_position_num;stringAPI[i]!=separation_char;i++) buffer_conv[i-first_position_num]=stringAPI[i]; //copy the number to convert
		buffer_conv[i-first_position_num]='\0'; //end char
                attribut[1]=atof_own(buffer_conv); //convertion to inter

		//if(stringAPI[i]!=separation_char) { hal.console->printf("Wrong usage of the function \n"); break;} //if the standart is not respected

		//Ki
		second_position_num=i+1;
		for(i=second_position_num;stringAPI[i]!=separation_char;i++) buffer_conv[i-second_position_num]=stringAPI[i]; //copy the number to convert
                buffer_conv[i-second_position_num]='\0'; //end char
                attribut[2]=atof_own(buffer_conv); //convertion to inter

		//if(stringAPI[i]!=separation_char) { hal.console->printf("Wrong usage of the function \n"); break;}//if the standart is not respected
		
		//Kd
		third_position_num=i+1;
                for(i=third_position_num;stringAPI[i]!='\0';i++) buffer_conv[i-third_position_num]=stringAPI[i]; //copy the number to convert
		buffer_conv[i-third_position_num]='\0'; //end char
                attribut[3]=atof_own(buffer_conv); //convertion to inter
		
		//Which PID?
		switch(stringAPI[1]) {
		case PIDcontroller_Throttle 	: trCTRL.editPID(Throttle,attribut[1],attribut[2],attribut[3]);		break; //hal.console->printf("PID modified\n");break;
		case PIDcontroller_Aileron     	: trCTRL.editPID(Aileron,attribut[1],attribut[2],attribut[3]); 		break; //hal.console->printf("PID modified\n");break;
		case PIDcontroller_Rudder 	: trCTRL.editPID(Rudder,attribut[1],attribut[2],attribut[3]);		break; //hal.console->printf("PID modified\n");break;
		case PIDcontroller_Elevator	: trCTRL.editPID(Elevator,attribut[1],attribut[2],attribut[3]); 	break; //hal.console->printf("PID modified\n");break;
                default:hal.console->printf("Wrong usage of the function \n"); break; //if the standart is not respected
		}
                break;
	
	//*** help ***//
	case 'h': 
		API_print_help(); 
		break;
	//*** fly ***//
	case 'f': 
		switch(stringAPI[1]) {
			case 't': Plane_flying_current_state=takeoff_mod; 	hal.console->printf("takeoff_mode actived \n"); 	break;
			case 'c': Plane_flying_current_state=circle_mod; 	hal.console->printf("circle_mode actived \n"); 		break;
			case 'l': Plane_flying_current_state=looping_mod; 	hal.console->printf("looping_mode actived \n"); 	break;
			case 'g': Plane_flying_current_state=go_streight_mod; 	hal.console->printf("go_streight_mode actived \n"); 	break;
			case 'r': Plane_flying_current_state=roll_mod; 		hal.console->printf("roll_mode actived \n"); 		break;
			case 'b': Plane_flying_current_state=back_glide_mod; 	hal.console->printf("back_glide_mode actived \n"); 	break;
			case 'h': Plane_flying_current_state=half_circle_mod; 	hal.console->printf("half_circle_mode actived \n"); 	break;
			case 'u': Plane_flying_current_state=upclimb_mod; 	hal.console->printf("climb_up_mode actived \n"); 	break;
			case 's': Plane_flying_current_state=snake_mod; 	hal.console->printf("snake_mode actived \n"); 		break;
			
			default: hal.console->printf("Wrong usage of the function \n"); break; //if the standart is not respected
		}
		break;
	//*** look ahead distance ***//
        case 'l': //not tested
                if(stringAPI[1]!=separation_char) {
                    hal.console->printf("Wrong usage of the function \n");
                    break; //if the standart is not respected
                }

                for(i=2;stringAPI[i]!='\0';i++) {
                    buffer_conv[i-2]=stringAPI[i];
                }
                buffer_conv[i-2]='\0'; //end char
                desiredL=atof_own(buffer_conv);
                hal.console->printf("Look ahead distance changed to: %f\n",desiredL);
                break;
		
	default: 
		hal.console->printf("The function does not exist or is not operational yet! \n");
		break;
	}	
}

//**********************//
/*float atof(char *str)
{	
	//Optimisation: conversion in float only at the end
	int res = 0; // Initialize // int to reduce the complexity of the operation
	// Iterate through all characters of input string and
	// update result
	for (int i = 0; str[i] != '\0'; ++i)
	{
		res = res*10 + str[i] - '0'; //before coma
	}
	//hal.console->printf("atoi %i\n",res);
	//hal.console->printf("atof %f\n",(float) res/1000);
	// return result.
	return (float) res/1000;
}*/

float atof_own(char* in){

        float out;
        int i;
        int point=-1;
        int length;
        float multiplier;
        float sign=1.0;

        if(in[0]=='-'){
                sign=-1.0;
                in[0] = '0';
        }
        if(in[0]=='+'){
                in[0] = '0';
        }


        for(i=0;in[i]!='\0';i++){
                if (in[i]=='.'){
                        point = i;
                }
        }
        length=i-1;

        if(point==-1){
                point = length+1;
        }

        multiplier = 1;
        for(i=point-1;i>=0;i--){
                out += static_cast<float>(static_cast<int>(in[i])-48)*multiplier;
                multiplier *= 10;
        }

        multiplier = 0.1;
        for(i=point+1;i<=length;i++){
                out += static_cast<float>(static_cast<int>(in[i])-48)*multiplier;
                multiplier /= 10;
        }

        out = sign * out;

        return(out);

}

