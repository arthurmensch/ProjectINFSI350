#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include "Vec3.h"
#include "Mesh.h"
#include "Camera.h"
#include "BoundingMesh.h"

int grabber(int x, int y,Mesh &cage,Camera &camera);
int grabberForm(int x, int y,Mesh &cage,Camera &camera,std::vector<bool> &selectedTriangle);
int grabberVertex(int x, int y,Mesh &cage,Camera &camera,std::vector<bool> &selectedTriangle);

void translateForm(Camera &camera,BoundingMesh &boundingMesh,std::vector<bool> &selectedTriangle, int triangleAimed,int x, int y, int lastX, int lastY);
void translateVertex(Camera &camera,BoundingMesh &boundingMesh, int vertexAimed,int x, int y, int lastX, int lastY);

void rotation(int lastX, int x, int lastY, int y, int beginTransformX, int beginTransformY);
Vec3f barycenter(Mesh &cage,std::vector<bool> &selectedTriangle);

void glSphereWithMat(float x,float y,float z,float r,float difR,float difB,float difG,float specR,float specG,float specB,float shininess, int color);
void glSphere (float x, float y, float z, float radius, int rgb);

void glTriangles(std::vector<Vertex> &V, std::vector<Triangle> &T, Vec3f & camPos);
void polar2Cartesian (float phi, float theta, float d, float & x, float & y, float & z);

void glDisk(Vec3f & position, Vec3f & normal, float radius, Vec3f & camPos);

void getColor(Vec3f & position,Vec3f & normal, Vec3f & camPos, float * c);
#endif // UTILS_H
