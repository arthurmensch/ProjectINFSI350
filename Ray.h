#include "Vec3.h"
#include "Mesh.h"
#include <cmath>
#include <iostream>

class Ray{
public:
	Vec3f origin;
	Vec3f direction;
	Ray(Vec3f &origin, Vec3f &direction);
	bool intersectTriangle(Mesh &mesh,Triangle &T, float &dist);

};
