#ifndef VECTORMATH_H
#define VECTORMATH_H

/***********************************************************

VectorMath.h
	Library for 3-dimensional vectors and their operations plus some transformations.
	
	
	vector: Elements can be accessed by their names .x, .y, and .z
			Copy Function importVector(): Imports the x-, y-, and z-value of the referenced vector.
			
		addVector()
		subtractVector()
		multiplyScalarToVector()
		CrossProduct()
		ScalarProduct()
		NormVector()
		
	Transformation matrices: Transformation from body frame to North-East-Down frame with the aid of Euler angles
		NEDtoBODY()
		BODYtoNED()
		
		
	Temporary: radiusCoefficientMatrix(): Matrix for adjusting the accelerometer (StateVariablesEstimation) -> to be placed to the right spot.

************************************************************/


// Declaration of a 3-dimensional vector
class vector {
public:
	// values of the three dimensions
    float x,y,z;	
    
	// Copy Function
    void importVector (const vector refVector);
};


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


// Transformations

// Transformation from NED-frame to Body-frame
struct vector NEDtoBODY ( const struct vector ned, const struct vector phiThetaPsi );

// Transformation from Body-frame to NED-frame
struct vector BODYtoNED ( const struct vector body, const struct vector phiThetaPsi );



// For the StateVariablesEstimation. Matrix for adjusting accelerations
struct vector radiusCoefficientMatrix ( const struct vector input );


#include "VectorMath.c"

#endif
