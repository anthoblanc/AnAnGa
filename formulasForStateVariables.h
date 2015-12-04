#ifndef FORMULASFORSTATEVARIABLES_H
#define FORMULASFORSTATEVARIABLES_H


// Declaration of a 3-dimensional vector
struct vector {
    float x,y,z;
};


// Formulas for calculating derivatives of the state variables
struct vector derivativeAngularRate( struct vector pqr, struct vector phiThetaPsi ){
    
    struct vector a_out;
	float sinPhi = sin(phiThetaPsi.x);
	float sinTheta = sin(phiThetaPsi.y);
	float cosPhi = cos(phiThetaPsi.x);
	float cosTheta = cos(phiThetaPsi.y);
    float tanTheta = sinTheta/cosTheta;
    
    a_out.x = pqr.x + 
            sinPhi*tanTheta*pqr.y + 
            cosPhi*tanTheta*pqr.z;
    a_out.y = cosPhi*pqr.y - sinPhi*pqr.z;
    a_out.z = (sinPhi/cosTheta)*pqr.y + (cosPhi/cosTheta)*pqr.z;
    
    return(a_out);
    }

struct vector derivativeVelocity( struct vector uvw, struct vector phiThetaPsi ){

	struct vector out;
	float sinPhi = sin(phiThetaPsi.x);
	float sinTheta = sin(phiThetaPsi.y);
	float sinPsi = sin(phiThetaPsi.z);
	float cosPhi = cos(phiThetaPsi.x);
	float cosTheta = cos(phiThetaPsi.y);
	float cosPsi = cos(phiThetaPsi.z);

	float sinPhiSinTheta = sinPhi * sinTheta;
	float cosPhiSinTheta = cosPhi * sinTheta;
	
	out.x = cosTheta*cosPsi*uvw.x + 
		(sinPhiSinTheta*cosPsi - cosPhi*sinPsi)*uvw.y +
		(cosPhiSinTheta*cosPsi + sinPhi*sinPsi)*uvw.z;
	out.y = cosTheta*sinPsi*uvw.x +
		(sinPhiSinTheta*sinPsi + cosPhi*cosPsi)*uvw.y +
		(cosPhiSinTheta*sinPsi - sinPhi*cosPsi)*uvw.z;
	out.z = -sinTheta*uvw.x + sinPhi*cosTheta*uvw.y + cosPhi*cosTheta*uvw.z;

	return(out);
}


// Formulas for calculating the vehicle-velocity plus the vehicle-acceleration
float velocity( struct vector uvw ){
    return sqrt( pow(uvw.x,2) + pow(uvw.y,2) + pow(uvw.z,2));
    }

float acceleration ( struct vector uvw, struct vector uvwDot ){
    return (uvw.x*uvwDot.x + uvw.y*uvwDot.y + uvw.z*uvwDot.z)/(velocity(uvw));
    }


// Declaration of all state variables for controlling the vehicle
struct vector uvw;
struct vector uvwDot;
struct vector pqr;
struct vector phiThetaPsi;
struct vector phiThetaPsiDot;
struct vector pnPePd;
struct vector pnPePdDot;
float groundSpeed;
float groundSpeedDot;


#endif
