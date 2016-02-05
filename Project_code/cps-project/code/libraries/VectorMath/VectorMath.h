#ifndef VECTORMATH_H
#define VECTORMATH_H


// Declaration of a 3-dimensional vector
class vector {

public:
    float x,y,z;

    // Copy Function
    void importVector (vector refVector);
};



// Transformation from NED-frame to Body-frame
struct vector NEDtoBODY ( const struct vector ned, const struct vector phiThetaPsi );

// Transformation from Body-frame to NED-frame
// STILL HAVE TO VERIFY IF INVERSE OF BODYTONED IS JUST THE TRANSPOSE
struct vector BODYtoNED ( const struct vector body, const struct vector phiThetaPsi );



/*##########################################################################################
                                   General Math Operations
  ########################################################################################## */

//Vector Addition x+y
struct vector addVector (const struct vector x, const struct vector y);

// Vector Subratction x-y
struct vector subtractVector (const struct vector x, const struct vector y);

//Vector multiplied by scalar c*x
struct vector multiplyScalarToVector (const struct vector x, float c);

// Cross Product
struct vector CrossProduct (const struct vector x, const struct vector y);

// Scalar Product
float ScalarProduct (const struct vector x, const struct vector y);

//Euclidian norm for 3D Vector
float NormVector(const struct vector v);




// temporary...
struct vector radiusCoefficientMatrix ( struct vector input );


#include "VectorMath.c"

#endif
