#include <cmath>
#include <set>
#include <iostream>
#include <GL/glut.h>

#include "Utils.h"
#include "Camera.h"
#include "Vec3.h"
#include "Mat3.h"
#include "Mesh.h"
#include "Ray.h"


int grabber(int x, int y,BoundingMesh *boundingMesh,Camera &camera) {
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projection);
	GLdouble modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelview);
	GLdouble winX,winY;
	winX=(double)x;
	winY=(double)y;
	winY=viewport[3]-winY;
	double nearX,nearY,nearZ;
	double farX,farY,farZ;
	gluUnProject(winX,winY,0.0f,modelview,projection,viewport,&nearX,&nearY,&nearZ);
	gluUnProject(winX,winY,1.0f,modelview,projection,viewport,&farX,&farY,&farZ);

	Vec3f origin=Vec3f(farX,farY,farZ);
	Vec3f direction=Vec3f(nearX,nearY,nearZ);
	Ray boundFinder=Ray(origin,direction);
	float profondeur=(float) (origin-direction).length();
	float dist;
	int numTriangle=-1;
	Mesh * cage=boundingMesh->getCage();
	for (unsigned int i=0;i<cage->T.size();i++){
		bool intersect=boundFinder.intersectTriangle(*cage,cage->T[i],dist);
		if (intersect){
			if(dist<profondeur){
				numTriangle=i;
				profondeur=dist;
			}
		}
	}
	return numTriangle;
}

int grabberForm(int x, int y,BoundingMesh *boundingMesh,Camera &camera) {
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projection);
	GLdouble modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelview);
	GLdouble winX,winY;
	winX=(double)x;
	winY=(double)y;
	winY=viewport[3]-winY;
	double nearX,nearY,nearZ;
	double farX,farY,farZ;
	gluUnProject(winX,winY,0.0f,modelview,projection,viewport,&nearX,&nearY,&nearZ);
	gluUnProject(winX,winY,1.0f,modelview,projection,viewport,&farX,&farY,&farZ);

	Vec3f origin=Vec3f(farX,farY,farZ);
	Vec3f direction=Vec3f(nearX,nearY,nearZ);
	Ray boundFinder=Ray(origin,direction);
	float profondeur=(float) (origin-direction).length();
	float dist;
	int numTriangle=-1;
	std::set<int> selectedTriangle=boundingMesh->getTriangleSelection();
	Mesh *cage =boundingMesh->getCage();
	for (std::set<int>::iterator it=selectedTriangle.begin();it!=selectedTriangle.end();it++){
		bool intersect=boundFinder.intersectTriangle(*cage,cage->T[*it],dist);
		if (intersect){
			if(dist<profondeur){
				numTriangle=*it;
				profondeur=dist;
			}
		}
	}
	return numTriangle;
}
int grabberVertex(int x, int y,BoundingMesh *boundingMesh,Camera &camera) {
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projection);
	GLdouble modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelview);
	GLdouble winX,winY;
	winX=(double)x;
	winY=(double)y;
	winY=viewport[3]-winY;
	double nearX,nearY,nearZ;
	double farX,farY,farZ;
	gluUnProject(winX,winY,0.0f,modelview,projection,viewport,&nearX,&nearY,&nearZ);
	gluUnProject(winX,winY,1.0f,modelview,projection,viewport,&farX,&farY,&farZ);

	Vec3f origin=Vec3f(farX,farY,farZ);
	Vec3f direction=Vec3f(nearX,nearY,nearZ);
	Ray boundFinder=Ray(origin,direction);
	float profondeur=(float) (origin-direction).length();
	float dist;
	int numVertex=-1;
	std::set<int> selectedTriangle=boundingMesh->getTriangleSelection();
	Mesh *cage =boundingMesh->getCage();
	for (std::set<int>::iterator it=selectedTriangle.begin();it!=selectedTriangle.end();it++){
		for (int j=0;j<3;j++){
			bool intersect=boundFinder.intersectVertex(*cage,cage->T[*it].v[j],0.015,dist);
			if (intersect){
				if(dist<profondeur){
					numVertex=cage->T[*it].v[j];
					profondeur=dist;
				}
			}
		}
	}
	return numVertex;
}

void translateForm(Camera &camera,BoundingMesh *boundingMesh,int x,int y,int lastX,int lastY){
    Vec3f camPos;
    camera.getPos(camPos);
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    GLdouble projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX,projection);
    GLdouble modelview[16];
    glGetDoublev(GL_MODELVIEW_MATRIX,modelview);

    double startX,startY,startZ;
    double endX,endY,endZ;
    gluUnProject((double)lastX,viewport[3]-(double)lastY,0.0f,modelview,projection,viewport,&startX,&startY,&startZ);
    gluUnProject((double)x,viewport[3]-(double)y,0.0f,modelview,projection,viewport,&endX,&endY,&endZ);
    Vec3f startPoint=Vec3f((float)startX,(float)startY,(float)startZ);
    Vec3f endPoint=Vec3f((float)endX,(float)endY,(float)endZ);
	Mesh *cage =boundingMesh->getCage();
    float rapport=1.0/(camPos-startPoint).length()*(camPos-barycenter(boundingMesh)).length();
    Vec3f translation=rapport*(endPoint-startPoint);
    std::set<int> vertexMoved;
	std::set<int> selectedTriangle=boundingMesh->getTriangleSelection();
	for (std::set<int>::iterator it=selectedTriangle.begin();it!=selectedTriangle.end();it++){
    		for (unsigned int i =0;i<3;i++){
    			int numVertex=cage->T[*it].v[i];
    			std::set<int>::iterator hasMoved=vertexMoved.find(numVertex);
    			if(hasMoved==vertexMoved.end()){
    				boundingMesh->moveCageVertexIncr(numVertex,translation);
    				vertexMoved.insert(numVertex);
    			}
    		}
    	}
}

void translateVertex(Camera &camera,BoundingMesh *boundingMesh,int vertexAimed,int x,int y,int lastX,int lastY){
    Vec3f camPos;
    camera.getPos(camPos);
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    GLdouble projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX,projection);
    GLdouble modelview[16];
    glGetDoublev(GL_MODELVIEW_MATRIX,modelview);

    double startX,startY,startZ;
    double endX,endY,endZ;
    gluUnProject((double)lastX,viewport[3]-(double)lastY,0.0f,modelview,projection,viewport,&startX,&startY,&startZ);
    gluUnProject((double)x,viewport[3]-(double)y,0.0f,modelview,projection,viewport,&endX,&endY,&endZ);
    Vec3f startPoint=Vec3f((float)startX,(float)startY,(float)startZ);
    Vec3f endPoint=Vec3f((float)endX,(float)endY,(float)endZ);
    float rapport=1.0/(camPos-startPoint).length()*(camPos-boundingMesh->getCage()->V[vertexAimed].p).length();
    Vec3f translation=rapport*(endPoint-startPoint);
    boundingMesh->moveCageVertexIncr(vertexAimed,translation);
}

void rotation(Camera &camera,BoundingMesh *boundingMesh, int x, int y, int lastX, int lastY){
    // Normal to near plan

    Vec3f camPos;
    camera.getPos(camPos);
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    GLdouble projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX,projection);
    GLdouble modelview[16];
    glGetDoublev(GL_MODELVIEW_MATRIX,modelview);

    double centerX,centerY,centerZ;

    gluUnProject((double) camera.getScreenWidth()/2,(double) camera.getScreenHeight()/2,0.0f,modelview,projection,viewport,&centerX,&centerY,&centerZ);

    Vec3f n(centerX-camPos[0],centerY-camPos[1],centerZ-camPos[2]);

    Vec3f bary = barycenter(boundingMesh);

    double barywX, barywY, barywZ;
    gluProject(bary[0],bary[1],bary[2],modelview,projection,viewport,&barywX,&barywY,&barywZ);

    barywY = camera.getScreenHeight() - barywY; // Inverted Y coordinates

    // Compute rotation angle

    int vec0x = lastX - barywX;
    int vec0y = lastY - barywY;
    int vec1x = x - barywX;
    int vec1y = y - barywY;
    int sign = vec0x * vec1y - vec0y * vec1x >= 0 ? 1 : -1;
    float lengths = sqrt(vec0x*vec0x + vec0y*vec0y) * sqrt(vec1x*vec1x + vec1y*vec1y);

    float angle;

    if (lengths == 0.0)
        angle = 0.0f;
    else
        angle = sign * acos((vec0x*vec1x + vec0y*vec1y) / lengths);


    // Apply rotation

    Mat3f r;
    r.rotation(n, angle);
    std::set<int> s;
    Vec3f tmp;
	Mesh *cage=boundingMesh->getOldCage();
	std::set<int> selectedTriangle=boundingMesh->getTriangleSelection();
	for (std::set<int>::iterator it=selectedTriangle.begin();it!=selectedTriangle.end();it++){
    	for (unsigned int i =0;i<3;i++){
        	if (s.find(cage->T[*it].v[i]) == s.end()) {
            	s.insert(cage->T[*it].v[i]);
            	tmp = cage->V[cage->T[*it].v[i]].p;
               	tmp -= bary;
               	tmp = r.multiply(tmp);
                tmp += bary;
                boundingMesh->moveCageVertexIncr(cage->T[*it].v[i], tmp - cage->V[cage->T[*it].v[i]].p);
            }
        }
    }
}

void scaling(Camera &camera,BoundingMesh *boundingMesh, int x, int y, int lastX, int lastY){
    // Barycenter projection

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    GLdouble projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX,projection);
    GLdouble modelview[16];
    glGetDoublev(GL_MODELVIEW_MATRIX,modelview);


    Vec3f bary = barycenter(boundingMesh);

    double barywX, barywY, barywZ;
    gluProject(bary[0],bary[1],bary[2],modelview,projection,viewport,&barywX,&barywY,&barywZ);

    barywY = camera.getScreenHeight() - barywY; // Inverted Y coordinates


    // Compute factor

    int vec0x = x - barywX;
    int vec0y = y - barywY;
    int vec1x = lastX - barywX;
    int vec1y = lastY - barywY;

    float length0 = sqrt(vec0x*vec0x + vec0y*vec0y);
    float length1 = sqrt(vec1x*vec1x + vec1y*vec1y);

    float ratio;

    if (length0 == 0.0f || length1 == 0.0f)
        ratio = 1.0f;
    else
        ratio = length0 / length1;

    // Apply rotation
    std::set<int> s;
    Vec3f tmp;
	Mesh *cage=boundingMesh->getOldCage();
	std::set<int> selectedTriangle=boundingMesh->getTriangleSelection();
	for (std::set<int>::iterator it=selectedTriangle.begin();it!=selectedTriangle.end();it++){
    	for (unsigned int i =0;i<3;i++){
        	if (s.find(cage->T[*it].v[i]) == s.end()) {
            	s.insert(cage->T[*it].v[i]);
                tmp = cage->V[cage->T[*it].v[i]].p;
                tmp = (tmp - bary) * ratio + bary;
                boundingMesh->moveCageVertexIncr(cage->T[*it].v[i], tmp - cage->V[cage->T[*it].v[i]].p);
            }
        }
    }
}

Vec3f barycenter(BoundingMesh *boundingMesh) {
    Vec3f res;
    std::set<int> s;
	Mesh *cage=boundingMesh->getOldCage();
	std::set<int> selectedTriangle=boundingMesh->getTriangleSelection();
    	for (auto it = selectedTriangle.begin(); it != selectedTriangle.end(); ++it) {
       		for (int j = 0 ; j < 3 ; j++) {
           		if (s.find(cage->T[*it].v[j]) == s.end()) {
              			res+=cage->V[cage->T[*it].v[j]].p;
              			s.insert(cage->T[*it].v[j]);
         		}
       		}
	}

    res /= (float) s.size();

    return res;
}

void glSphere (float x, float y, float z, float radius, Vec3f col) {
    float step = 1 / (2*M_PI);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef (radius, radius, radius);

    glBegin (GL_TRIANGLE_STRIP);
    for (float phi = 0.0 ; phi < M_PI ; phi += step) {
        for (float theta = 0.0 ; theta < 2 * M_PI ; theta += step) {
            glColor3f (col[0], col[1], col[2]);
            glNormal3f (sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi));
            glVertex3f (sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi));

            glColor3f (col[0], col[1], col[2]);
            glNormal3f (sin(phi+step)*cos(theta), sin(phi+step)*sin(theta), cos(phi+step));
            glVertex3f (sin(phi+step)*cos(theta), sin(phi+step)*sin(theta), cos(phi+step));
        }
    }

    glEnd ();

    glPopMatrix();
}
void glQuadSelect(int lastX,int lastY, int beginTransformX,int beginTransformY){
    GLint mode[2];
	glGetIntegerv( GL_POLYGON_MODE, mode );
	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projection);
	GLdouble modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelview);
	Vec3d debut,debutm,finm,fin,normal;
	gluUnProject((double)beginTransformX,viewport[3]-(double)beginTransformY,0.05f,modelview,projection,viewport,&debut[0],&debut[1],&debut[2]);
	gluUnProject((double)beginTransformX,viewport[3]-(double)lastY,0.05f,modelview,projection,viewport,&debutm[0],&debutm[1],&debutm[2]);
	gluUnProject((double)lastX,viewport[3]-(double)lastY,0.05f,modelview,projection,viewport,&finm[0],&finm[1],&finm[2]);
	gluUnProject((double)lastX,viewport[3]-(double)beginTransformY,0.05f,modelview,projection,viewport,&fin[0],&fin[1],&fin[2]);
	normal=normalize(cross(debutm-debut,fin-debut));
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glBegin(GL_QUADS);
	if((lastX-beginTransformX)*(lastY-beginTransformY)>0){
		glColor3f(1.0,1.0,1.0);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3d(debut[0],debut[1],debut[2]);
		glColor3f(1.0,1.0,1.0);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3d(debutm[0],debutm[1],debutm[2]);
		glColor3f(1.0,1.0,1.0);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3d(finm[0],finm[1],finm[2]);
		glColor3f(1.0,1.0,1.0);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3d(fin[0],fin[1],fin[2]);
	}
	else{
		normal=-normal;
		glColor3f(1.0,1.0,1.0);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3d(debut[0],debut[1],debut[2]);
		glColor3f(1.0,1.0,1.0);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3d(fin[0],fin[1],fin[2]);
		glColor3f(1.0,1.0,1.0);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3d(finm[0],finm[1],finm[2]);
		glColor3f(1.0,1.0,1.0);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3d(debutm[0],debutm[1],debutm[2]);
	}
	glEnd();
	glPopMatrix();
	glPolygonMode( GL_FRONT_AND_BACK, mode[1] );
}
void polar2Cartesian (float phi, float theta, float d, float & x, float & y, float & z)
{
    x = d*sin (theta) * cos (phi);
    y = d*cos (theta);
    z = d*sin (theta) * sin (phi);
}

void glTriangles(std::vector<Vertex> &V, std::vector<Triangle> &T, Vec3f & camPos) {
glBegin (GL_TRIANGLES);
    for (unsigned int i = 0; i < T.size (); i++)
        for (unsigned int j = 0; j < 3; j++)
        {
            Vertex v = V[T[i].v[j]];
            float c[3];
            glColor3f (c[0], c[1], c[2]);
            glNormal3f (v.n[0], v.n[1], v.n[2]); // Specifies current normal vertex
            glVertex3f (v.p[0], v.p[1], v.p[2]); // Emit a vertex (one triangle is emitted each time 3 vertices are emitted)
        }
    glEnd ();
}
