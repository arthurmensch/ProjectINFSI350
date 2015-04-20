#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include "Vec3.h"
#include "Mesh.h"
#include "Camera.h"

int grabber(int x, int y,Mesh &cage,Camera &camera);

void modifyBoundingMesh();

void glSphere (float x, float y, float z, float radius, int rgb);

void glTriangles(std::vector<Vertex> &V, std::vector<Triangle> &T, Vec3f & camPos);
void polar2Cartesian (float phi, float theta, float d, float & x, float & y, float & z);

void glDisk(Vec3f & position, Vec3f & normal, float radius, Vec3f & camPos);

void getColor(Vec3f & position,Vec3f & normal, Vec3f & camPos, float * c);
#endif // UTILS_H
