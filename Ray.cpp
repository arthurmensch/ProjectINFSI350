#include <cmath>

#include "Vec3.h"

#include "Mesh.h"
#include "Ray.h"
#include <iostream>

Ray::Ray(Vec3f &origin,Vec3f &direction){
	this->origin=origin;
	this->direction=direction;
}

bool Ray::intersectTriangle(Mesh &mesh ,Triangle &T){
	
	Vec3f normal=normalize(cross(mesh.V[T.v[0]].p-mesh.V[T.v[1]].p,mesh.V[T.v[0]].p-mesh.V[T.v[2]].p));
	Vec3f point=mesh.V[T.v[0]].p;
	float constante=dot(point,normal);
	float constanteDir=dot(this->direction,normal);
	if(std::abs(dot(normal,normalize(direction-origin)))<0.00001){//normale triangle et rayon sont perpendiculaires
	//tester si direction est dans le bon plan
		if(std::abs(constanteDir-constante)<0.00001)
			return true;
		return false;
	}
	float lambda=(constante-constanteDir)/dot(normal,this->origin-this->direction);
	
	Vec3f pointIntersect=this->direction+(this->origin-this->direction)*lambda;
	if(lambda<0.0001)//intersection à direction ou derriere
		return false;
	else if (lambda<0.999){
		Vec3f nul=Vec3f(0.0001,0.0001,0.0001);
		Vec3f u=mesh.V[T.v[1]].p-point;
		Vec3f v=mesh.V[T.v[2]].p-point;
		Vec3f w=pointIntersect-point;
		float denom=(dot(u,v)*dot(u,v)-dot(u,u)*dot(v,v));
		float s=(dot(u,v)*dot(w,v)-dot(v,v)*dot(w,u))/denom;
		if (s<-0.0001)
			return false;
		float t= (dot(u,w)*dot(u,v)-dot(u,u)*dot(w,v))/denom;
		if (t<-0.0001)
			return false;
		if ((t+s)>1.0001)
			return false;

		return true;
	}
	return false;
}
