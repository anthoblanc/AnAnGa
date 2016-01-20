#ifndef __PATH_H__
#define __PATH_H__

#include "../PID/formulasForStateVariables.h"

struct vector pathloop ( struct vector pnPePdtmp0, struct vector pnPePdtmp, uint32_t time, uint32_t tstart, uint32_t tend ){

    struct vector allola;
	int r = 300;
    // 在pathloop call的时候计算下面20个target点 =D
    allola.x = pnPePdtmp.x + 2*(pnPePdtmp0.x - pnPePdtmp.x)*sin(360.0*(M_PI/180.0)*(time+2000000-tstart)/(tend-tstart));
    allola.y = pnPePdtmp.y + 2*(pnPePdtmp0.y - pnPePdtmp.y)*sin(360.0*(M_PI/180.0)*(time+2000000-tstart)/(tend-tstart));
    allola.z = pnPePdtmp.z - abs(r*sqrt(1-cos(360.0*(M_PI/180.0)*(time+2000000-tstart)/(tend-tstart)))); // give altitude coordinate after 2 sec
    
    return(allola);
    }

struct vector pathned;
struct vector pnPePdtmp;
struct vector pnPePdtmp0;

#endif
