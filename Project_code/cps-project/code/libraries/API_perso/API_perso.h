#ifndef API_PERSO_H
#define API_PERSO_H


/* ***********************************************
Print_help

Print the explaination how to use the API
*********************************************** */
void API_print_help() ;

/* ***********************************************
API_interpretate_chain(char * stringAPI, int length_stringAPI)

char * stringAPI : string obtain from the USB interface
int length_stringAPI : length of the string

interpretate in term of action a given string such as:

-------------------------------- HELP --------------------------------
'h' 					help

-------------------------------- EDIT --------------------------------

### PID ###

'et-#VAL_P#-#VAL_I#-#VAL_D#' 		edit throttle
'ea-#VAL_P#-#VAL_I#-#VAL_D#' 		edit aileron		
'er-#VAL_P#-#VAL_I#-#VAL_D#' 		edit rudder 		
'ee-#VAL_P#-#VAL_I#-#VAL_D#' 		edit elevator

/!\ gain divided by 1000 !!!

#VAL_P# sould be remplace by the value of the PROPORTIONAL 	gain divided by 1000
#VAL_I# sould be remplace by the value of the INTEGRALE 	gain divided by 1000
#VAL_D# sould be remplace by the value of the DERIVATE 		gain divided by 1000

---------------------------
### look ahead distance ###

l-#VAL_lookahead# 	edit look_ahead_distance

Note: This function has not been tested


------------------------------ FLY MODE -------------------------------
'ft' take off or go higher
'fc' fly a circle 
'fl' do a looping 
'fs' fly streight East
'fr' do a roll
'fb' fly streight West // back glide
'fh' fly half circle 

*********************************************** */
void API_interpretate_chain(char * stringAPI, int length_stringAPI,TrajectoryController& trCTRL, int& Plane_flying_current_state,float& desiredL) ;

/* ***********************************************
int atoi(char *str)

char * str string to convert

convert a char into an interger
		
*********************************************** */
//float atof(char *str);
float atof_own(char *in);


#include "API_perso.cpp"

#endif
