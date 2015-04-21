#pragma once

#include <cmath>
#include <iostream>

#include "Vec3.h"

/*
 	036
	147
	258
*/

/// Vector in 3 dimensions, with basics operators overloaded.
class Mat3f {

public:
  inline Mat3f (void)	{ m[0] = m[1] = m[2] = m[3] = m[4] = m[5] = m[6] = m[7] = m[8] = 0.0; }
  
  ~Mat3f() {}

  inline Mat3f (float* mm) { 
	m[0] = mm[0];
	m[1] = mm[1];
	m[2] = mm[2];
	m[3] = mm[3]; 
	m[4] = mm[4]; 
	m[5] = mm[5]; 
	m[6] = mm[6]; 
	m[7] = mm[7]; 
	m[8] = mm[8]; 
  };

  inline Vec3f multiply(const Vec3f& v) const
  {
    return Vec3f(m[0]*v[0] + m[3]*v[1] + m[6]*v[2],
                m[1]*v[0] + m[4]*v[1] + m[7]*v[2],
                m[2]*v[0] + m[5]*v[1] + m[8]*v[2]);
  }

  void rotation(Vec3f n, float angle) {
  	n.normalize();
  	m[0]=1+(1-cos(angle))*(n[0]*n[0]-1); 			m[3]=-n[2]*sin(angle)+(1-cos(angle))*n[0]*n[1];	m[6]=n[1]*sin(angle)+(1-cos(angle))*n[0]*n[2];
  	m[1]=n[2]*sin(angle)+(1-cos(angle))*n[0]*n[1]; 	m[4]=1+(1-cos(angle))*(n[1]*n[1]-1);			m[7]=-n[0]*sin(angle)+(1-cos(angle))*n[1]*n[2];
  	m[2]=-n[1]*sin(angle)+(1-cos(angle))*n[0]*n[2];	m[5]=n[0]*sin(angle)+(1-cos(angle))*n[1]*n[2];	m[8]=1+(1-cos(angle))*(n[2]*n[2]-1);
  }

  protected:
  float m[9];
};


 