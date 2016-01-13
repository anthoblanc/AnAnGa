#include <math.h>  //mathematical operator such as sqrt
#include "../TrajectoryControl/TrajectoryControl.h"

struct vector Provide_nice_curve_acc()
{
	struct vector acc_cmd;
	return vector acc_cmd;
}

struct vector Get_Acc_straigth(struct vector velocity_refgnd, struct vector trajectory_refgnd)
{
	struct vector acc_cmd;
	
	acc_cmd=CrossProduct(CrossProduct(velocity_refgnd,trajectory_refgnd),velocity_refgnd); // V*L*V
	
	//product with a scalar
	acc_cmd.x*=2/NormVector(trajectory_refgnd)/NormVector(trajectory_refgnd); // 2/L²
	acc_cmd.y*=2/NormVector(trajectory_refgnd)/NormVector(trajectory_refgnd); // 2/L²
	acc_cmd.z*=2/NormVector(trajectory_refgnd)/NormVector(trajectory_refgnd); // 2/L²

	/*struct StateVariable {
    		struct vector uvw, uvwDot ,pqr, phiThetaPsi, phiThetaPsiDot, pnPePd, pnPePdDot;
    		float groundSpeed, groundSpeedDot;
	};*/

	return vector acc_cmd;
}
