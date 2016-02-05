
void vector::importVector (vector refVector){

    x = refVector.x;
    y = refVector.y;
    z = refVector.z;

}

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
	

// Transformation from NED-frame to Body-frame
struct vector NEDtoBODY ( const struct vector ned, const struct vector phiThetaPsi ) {

	struct vector body;
	
	float sinPhi = sin(phiThetaPsi.x);
	float sinTheta = sin(phiThetaPsi.y);
	float sinPsi = sin(phiThetaPsi.z);
	float cosPhi = cos(phiThetaPsi.x);
	float cosTheta = cos(phiThetaPsi.y);
	float cosPsi = cos(phiThetaPsi.z);

	float sinPhiSinTheta = sinPhi * sinTheta;
	float cosPhiSinTheta = cosPhi * sinTheta;
	
	//source: http://www.es.ele.tue.nl/education/5HC99/wiki/images/4/42/RigidBodyDynamics.pdf
	body.x = (cosTheta*cosPsi) * ned.x + 
		(cosTheta*sinPsi)  * ned.y +
		-sinTheta          * ned.z;
	body.y = (sinPhiSinTheta*cosPsi - cosPhi*sinPsi) * ned.x +
		(sinPhiSinTheta*sinPsi + cosPhi*cosPsi)  * ned.y +
		sinPhi*cosTheta                          * ned.z;
	body.z = (cosPhiSinTheta*cosPsi + sinPhi*sinPsi) * ned.x + 
		(cosPhiSinTheta*sinPsi - sinPhi*cosPsi)  * ned.y +
		cosPhi*cosTheta                          * ned.z;	

	return(body);

}

// Transformation from Body-frame to NED-frame
// STILL HAVE TO VERIFY IF INVERSE OF BODYTONED IS JUST THE TRANSPOSE
struct vector BODYtoNED ( const struct vector body, const struct vector phiThetaPsi ) {

        struct vector ned;

        float sinPhi = sin(phiThetaPsi.x);
        float sinTheta = sin(phiThetaPsi.y);
        float sinPsi = sin(phiThetaPsi.z);
        float cosPhi = cos(phiThetaPsi.x);
        float cosTheta = cos(phiThetaPsi.y);
        float cosPsi = cos(phiThetaPsi.z);

        float sinPhiSinTheta = sinPhi * sinTheta;
        float cosPhiSinTheta = cosPhi * sinTheta;

        ned.x = (cosTheta*cosPsi) * body.x +
                (sinPhiSinTheta*cosPsi - cosPhi*sinPsi) * body.y +
                (cosPhiSinTheta*cosPsi + sinPhi*sinPsi) * body.z;

        ned.y = (cosTheta*sinPsi)  * body.x +
                (sinPhiSinTheta*sinPsi + cosPhi*cosPsi)  * body.y +
                (cosPhiSinTheta*sinPsi - sinPhi*cosPsi)  * body.z;

        ned.z = -sinTheta * body.x +
                sinPhi*cosTheta * body.y +
                cosPhi*cosTheta * body.z;

        return(ned);

}


void StateVariables::importData (StateVariables refStateVars){

    uvw.importVector(refStateVars.uvw);
    uvwDot.importVector(refStateVars.uvwDot);
    pqr.importVector(refStateVars.pqr);
    phiThetaPsi.importVector(refStateVars.phiThetaPsi);
    phiThetaPsiDot.importVector(refStateVars.phiThetaPsiDot);
    pnPePd.importVector(refStateVars.pnPePd);
    pnPePdDot.importVector(refStateVars.pnPePdDot);
    groundSpeed = refStateVars.groundSpeed;
    groundSpeedDot = refStateVars.groundSpeedDot;

}


struct StateVariables calculateStateVariables (const struct sample dataSample) {
	
	struct StateVariables out;

	// Assignment of all read values to the state variables
	out.uvw.x = dataSample.data.f[I_VX];
	out.uvw.y = dataSample.data.f[I_VY];
	out.uvw.z = dataSample.data.f[I_VZ];

	out.uvwDot.x = dataSample.data.f[I_AX];
	out.uvwDot.y = dataSample.data.f[I_AY];
	out.uvwDot.z = dataSample.data.f[I_AZ];

	out.pqr.x = dataSample.data.f[I_P];
	out.pqr.y = dataSample.data.f[I_Q];
	out.pqr.z = dataSample.data.f[I_R];

	out.phiThetaPsi.x = dataSample.data.f[I_PHI];
	out.phiThetaPsi.y = dataSample.data.f[I_THETA];
	out.phiThetaPsi.z = dataSample.data.f[I_PSI];

	out.pnPePd.x = dataSample.data.f[I_LAT];
	out.pnPePd.y = dataSample.data.f[I_LON];
	out.pnPePd.z = dataSample.data.f[I_ALT];

	// Calculate the missing state variables
	out.phiThetaPsiDot = derivativeAngularRate( out.pqr, out.phiThetaPsi );
	out.pnPePdDot = derivativeVelocity( out.uvw, out.phiThetaPsi );
	out.groundSpeed = velocity( out.uvw );
	out.groundSpeedDot = acceleration ( out.uvw, out.uvwDot );
	
	return(out);
}
