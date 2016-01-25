#include <math.h>  //mathematical operator such as sqrt
#include "../TrajectoryControl/TrajectoryControl.h"

struct vector Provide_nice_curve_acc()
{
	struct vector acc_cmd;
	return( acc_cmd );
}

struct vector Get_Acc_straigth(const AP_HAL::HAL& hal,struct vector velocity_refgnd, struct vector trajectory_refgnd)
{
	struct vector acc_cmd;
        float norm_L_squared;

        norm_L_squared = 2/ScalarProduct(trajectory_refgnd,trajectory_refgnd);
	
        acc_cmd = CrossProduct(velocity_refgnd,trajectory_refgnd);
    //hal.console->printf("%f,%f,%f,%f,",acc_cmd.x,acc_cmd.y,acc_cmd.z,norm_L_squared);
        acc_cmd=CrossProduct(acc_cmd,velocity_refgnd); // V*L*V
	
	//product with a scalar
        acc_cmd.x *= norm_L_squared; // 2/L²
        acc_cmd.y *= norm_L_squared; // 2/L²
        acc_cmd.z *= norm_L_squared; // 2/L²


	return(acc_cmd);
}
