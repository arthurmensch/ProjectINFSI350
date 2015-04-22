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


int grabber(int x, int y,Mesh &cage,Camera &camera) {
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projection);
	GLdouble modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelview);
	GLdouble winX,winY,winZ;
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
	for (unsigned int i=0;i<cage.T.size();i++){
		bool intersect=boundFinder.intersectTriangle(cage,cage.T[i],dist);
		if (intersect){
			if(dist<profondeur){
				numTriangle=i;
				profondeur=dist;
			}
		}
	}
	return numTriangle;
}

int grabberForm(int x, int y,Mesh &cage,Camera &camera,std::vector<bool> &selectedTriangle) {
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projection);
	GLdouble modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelview);
	GLdouble winX,winY,winZ;
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
	for (unsigned int i=0;i<cage.T.size();i++){
		if(selectedTriangle[i]){
			bool intersect=boundFinder.intersectTriangle(cage,cage.T[i],dist);
			if (intersect){
				if(dist<profondeur){
					numTriangle=i;
					profondeur=dist;
				}
			}
		}
	}
	return numTriangle;
}
int grabberVertex(int x, int y,Mesh &cage,Camera &camera,std::vector<bool> &selectedTriangle) {
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projection);
	GLdouble modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelview);
	GLdouble winX,winY,winZ;
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
	for (unsigned int i=0;i<selectedTriangle.size();i++){
		if(selectedTriangle[i]){
			for (int j=0;j<3;j++){
				bool intersect=boundFinder.intersectVertex(cage,cage.T[i].v[j],0.01,dist);
				if (intersect){
					if(dist<profondeur){
						numVertex=cage.T[i].v[j];
						profondeur=dist;
					}
				}
			}
		}
	}
	return numVertex;
}

void translateForm(Camera &camera,BoundingMesh &boundingMesh,std::vector<bool> &selectedTriangle,int triangleAimed,int x,int y,int lastX,int lastY){
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
    float rapport=1.0/(camPos-startPoint).length()*(camPos-barycenter(*boundingMesh.cage, selectedTriangle)).length();
    Vec3f translation=rapport*(endPoint-startPoint);
    std::set<int> vertexMoved;
    for (unsigned int j=0;j<selectedTriangle.size();j++){
    	if(selectedTriangle[j]){
    		for (unsigned int i =0;i<3;i++){
    			int numVertex=boundingMesh.cage->T[j].v[i];
    			std::set<int>::iterator hasMoved=vertexMoved.find(numVertex);
    			if(hasMoved==vertexMoved.end()){
    				boundingMesh.moveCageVertexIncr(numVertex,translation);
    				vertexMoved.insert(numVertex);
    			}
    		}
    	}
    }
}

void translateVertex(Camera &camera,BoundingMesh &boundingMesh,int vertexAimed,int x,int y,int lastX,int lastY){
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
    float rapport=1.0/(camPos-startPoint).length()*(camPos-boundingMesh.cage->V[vertexAimed].p).length();
    Vec3f translation=rapport*(endPoint-startPoint);
    boundingMesh.moveCageVertexIncr(vertexAimed,translation);
}

void rotation(Camera &camera,BoundingMesh &boundingMesh,std::vector<bool> &selectedTriangle, int x, int y, int lastX, int lastY){
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

    Vec3f bary = barycenter(*boundingMesh.cage, selectedTriangle);

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

    for (unsigned int i = 0; i < selectedTriangle.size(); ++i) {
        if(selectedTriangle[i]) {
            for (int j = 0 ; j < 3 ; j++) {
                if (s.find(boundingMesh.cage->T[i].v[j]) == s.end()) {
                    s.insert(boundingMesh.cage->T[i].v[j]);
                    tmp = boundingMesh.cage->V[boundingMesh.cage->T[i].v[j]].p;
                    tmp -= bary;
                    tmp = r.multiply(tmp);
                    tmp += bary;
                    boundingMesh.moveCageVertexIncr(boundingMesh.cage->T[i].v[j], tmp - boundingMesh.cage->V[boundingMesh.cage->T[i].v[j]].p);
                }
            }
        }
    }
}

Vec3f barycenter(Mesh &cage,std::vector<bool> &selectedTriangle) {
    Vec3f res;
    std::set<int> s;
    for (unsigned int i = 0; i < selectedTriangle.size(); ++i) {
        if(selectedTriangle[i]) {
            for (int j = 0 ; j < 3 ; j++) {
                if (s.find(cage.T[i].v[j]) == s.end()) {
                    res+=cage.V[cage.T[i].v[j]].p;
                    s.insert(cage.T[i].v[j]);
                }
            }
        }
    }

    res /= (float) s.size();

    return res;
}

void glSphere (float x, float y, float z, float radius, Vec3i col) {
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
            getColor(v.p, v.n, camPos,c);
            glColor3f (c[0], c[1], c[2]);
            glNormal3f (v.n[0], v.n[1], v.n[2]); // Specifies current normal vertex
            glVertex3f (v.p[0], v.p[1], v.p[2]); // Emit a vertex (one triangle is emitted each time 3 vertices are emitted)
        }
    glEnd ();
}

void glDisk(Vec3f & position, Vec3f & normal, float radius, Vec3f & camPos) {
    int n = 10;
    float c[3];
    getColor(position, normal, camPos, c);
    float coordinates[n][3];
    for(int i = 0; i < n; i++)
    {
            coordinates[i][0] = radius * cos( ((float) i) / ((float) n) * 2*M_PI);
            coordinates[i][1] = radius * sin( ((float) i) / ((float) n) * 2 * M_PI);
            coordinates[i][2] = 0;
    }
    glMatrixMode (GL_MODELVIEW);
    glPushMatrix ();
    glTranslatef (position[0], position[1], position[2]);
    glRotatef(acos(normal[2])/M_PI*180,-normal[1],normal[0],0);

    glBegin(GL_TRIANGLES);
    for(int i = 0; i < n-1; i++)
    {
            glColor3f(c[0],c[1],c[2]);
            glNormal3f(0, 0, 1);
            glVertex3f(0, 0, 0);

            glColor3f(c[0],c[1],c[2]);
            glNormal3f(0, 0, 1);
            glVertex3f(coordinates[i][0], coordinates[i][1], coordinates[i][2]);

            glColor3f(c[0],c[1],c[2]);
            glNormal3f(0, 0, 1);
            glVertex3f(coordinates[i+1][0], coordinates[i+1][1], coordinates[i+1][2]);

        }
    glEnd();
    glPopMatrix ();
}

void getColor(Vec3f & position, Vec3f & normal, Vec3f & camPos, float * c) {
    Vec3f light = Vec3f(0.0,0.0,1.0);
    float F0 = 0.91;
    float alpha = 1;

    Vec3f wi = light - position;
    float dist_source = wi.normalize();
    Vec3f w0 = camPos - position;
//    float dist_target = w0.normalize();

    Vec3f wh = w0 + wi;
    wh.normalize();

    float widotwh = dot(wi,wh);
    float vndotwh = dot(normal,wh);
    float vndotwi = dot(normal,wi);
    float w0dotwh = dot(w0,wh);
    float w0dotvn = dot(w0,normal);

    float D = 1 / (M_PI*alpha*alpha*(vndotwh*vndotwh*vndotwh*vndotwh))*exp((1 - 1/(vndotwh*vndotwh))/(alpha*alpha));
    float temp = 1- fmax(0,widotwh);
    float F = F0 + (1-F0)*temp*temp*temp*temp*temp;
    float G = fmin(fmin(1,2*vndotwh*vndotwi/w0dotwh),2*vndotwh*w0dotvn/w0dotwh);

    float f = D*F*G;

    for (int k = 0; k < 3; k++)
    {
        c[k] = 5 * f *dot(w0,normal) / (dist_source+1)/ (dist_source+1);
    }
}
