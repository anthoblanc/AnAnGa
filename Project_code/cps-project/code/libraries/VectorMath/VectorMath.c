
void vector::importVector (vector refVector){

    x = refVector.x;
    y = refVector.y;
    z = refVector.z;

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

/*##########################################################################################
                                   General Math Operations
  ########################################################################################## */

//Vector Addition
struct vector addVector (const struct vector x, const struct vector y){

        struct vector z;

        z.x = x.x+y.x;
        z.y = x.y+y.y;
        z.z = x.z+y.z;

        return(z);
}

// Vector Subratction x-y
struct vector subtractVector (const struct vector x, const struct vector y){

    struct vector z;

    z.x = x.x-y.x;
    z.y = x.y-y.y;
    z.z = x.z-y.z;

    return(z);
}

//Vector multiplied by scalar c*x
struct vector multiplyScalarToVector (const struct vector x, float c){

    struct vector z;

    z.x = x.x*c;
    z.y = x.y*c;
    z.z = x.z*c;

    return(z);
}


// Cross Product
struct vector CrossProduct (const struct vector x, const struct vector y) {

        struct vector z;

        z.x = x.y*y.z - x.z*y.y;
        z.y = x.z*y.x - x.x*y.z;
        z.z = x.x*y.y - x.y*y.x;

        return(z);
}

// Scalar Product
float ScalarProduct (const struct vector x, const struct vector y) {

        float z;

        z = x.x * y.x + x.y * y.y + x.z * y.z;

        return(z);
}

//Euclidian norm for 3D Vector
float NormVector(const struct vector v) {
        float norm;
        norm=sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
        return(norm);
}

// temporary....
struct vector radiusCoefficientMatrix ( struct vector in ){

    struct vector out;

    out.x = 1.0*in.x + 0.0*in.y + 0.0*in.z;
    out.y = 0.0*in.x + 1.0*in.y + 0.0*in.z;
    out.z = 0.0*in.x + 0.0*in.y + 1.0*in.z;

    return(out);
}
