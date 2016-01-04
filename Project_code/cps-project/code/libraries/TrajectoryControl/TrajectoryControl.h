#ifndef TRAJECTORYCONTROL_H
#define TRAJECTORYCONTROL_H

// General Math Operations
// Cross Product
struct vector CrossProduct (const struct vector x, const struct vector y){
	
	struct vector z;
	
	z.x = x.y*y.z - x.z*y.y;
	z.y = x.z*y.x - x.x*y.z;
	z.z = x.x*y.y - x.y*y.x;
	
	return(z);
}





#endif