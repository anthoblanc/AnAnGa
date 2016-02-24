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
'et-#VAL_P#-#VAL_I#-#VAL_D#' 		edit throttle
'ea-#VAL_P#-#VAL_I#-#VAL_D#' 		edit aileron		
'er-#VAL_P#-#VAL_I#-#VAL_D#' 		edit rudder 		
'ee-#VAL_P#-#VAL_I#-#VAL_D#' 		edit elevator

#VAL_P# sould be remplace by the value of the PROPORTIONAL 	gain
#VAL_I# sould be remplace by the value of the INTEGRALE 	gain
#VAL_D# sould be remplace by the value of the DERIVATE 		gain

*********************************************** */
void API_interpretate_chain(char * stringAPI, int length_stringAPI,TrajectoryController& trCTRL) ;

/* ***********************************************
int atoi(char *str)

char * str string to convert

convert a char into an interger
		
*********************************************** */
float atoi(char *str);


#include "API_perso.cpp"

#endif
