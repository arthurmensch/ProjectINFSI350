#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include "Vec3.h"
#include "Mesh.h"
#include "Camera.h"
#include "BoundingMesh.h"

int grabber(int x, int y,BoundingMesh *boundingMesh,Camera &camera);
int grabberForm(int x, int y,BoundingMesh *boundingMesh,Camera &camera);
int grabberVertex(int x, int y,BoundingMesh *boundingMesh,Camera &camera);

void translateForm(Camera &camera,BoundingMesh *boundingMesh,int x, int y, int lastX, int lastY);
void translateVertex(Camera &camera,BoundingMesh *boundingMesh, int vertexAimed,int x, int y, int lastX, int lastY);

void rotation(Camera &camera,BoundingMesh &boundingMesh,std::vector<bool> &selectedTriangle, int x, int y, int lastX, int lastY);
void scaling(Camera &camera,BoundingMesh &boundingMesh,std::vector<bool> &selectedTriangle, int x, int y, int lastX, int lastY);
Vec3f barycenter(BoundingMesh *boundingMesh);

void glSphere (float x, float y, float z, float radius, Vec3i selectedColor);
void glQuadSelect(int lastX,int lastY, int beginTransformX, int beginTransformY);
void glTriangles(std::vector<Vertex> &V, std::vector<Triangle> &T, Vec3f & camPos);
void polar2Cartesian (float phi, float theta, float d, float & x, float & y, float & z);

void glDisk(Vec3f & position, Vec3f & normal, float radius, Vec3f & camPos);

void getColor(Vec3f & position,Vec3f & normal, Vec3f & camPos, float * c);
#endif // UTILS_H
