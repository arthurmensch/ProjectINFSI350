#include "Utils.h"
#include "Camera.h"
#include "Vec3.h"
#include <GL/glut.h>
#include <cmath>
#include "Mesh.h"
#include "Ray.h"
//#include <glm/glm.hpp>

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

int grabberVertex(int x, int y,Mesh &cage,Camera &camera) {
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
void translateTriangle(Camera &camera,BoundingMesh &boundingMesh,int triangle,float x,float y,float lastX,float lastY){
//au début du click
//lastX
//lastY
//pendant le click on a X,Y
//-> il faut le renvoyer a x,y,z dnas le plan orthogonal a camera
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
float rapport=1.0/(camPos-startPoint).length()*(camPos-boundingMesh.cage->V[boundingMesh.cage->T[triangle].v[0]].p).length();
Vec3f translation=rapport*(endPoint-startPoint);
for (unsigned int i =0;i<3;i++){
	Vec3f pointToUpdate=boundingMesh.cage->V[boundingMesh.cage->T[triangle].v[i]].p;
	//boundingMesh->updateMesh(triangle,i,pointToUpdate[0]+translation[0],pointToUpdate[1]+translation[1],pointToUpdate[2]+translation[2]);
}
}

void translateVertex(Camera &camera,BoundingMesh &boundingMesh,int vertex,float x,float y,float lastX,float lastY){
//au début du click
//lastX
//lastY
//pendant le click on a X,Y
//-> il faut le renvoyer a x,y,z dnas le plan orthogonal a camera
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
float rapport=1.0/(camPos-startPoint).length()*(camPos-boundingMesh.cage->V[vertex].p).length();
Vec3f translation=rapport*(endPoint-startPoint);
Vec3f pointToUpdate=boundingMesh.cage->V[vertex].p;
//boundingMesh.updateMesh(triangle,i,pointToUpdate[0]+translation[0],pointToUpdate[1]+translation[1],pointToUpdate[2]+translation[2]);
}

void modifyBoundingMesh() {}

void glSphereWithMat(float x,float y,float z,float r,float difR,float difG,float difB,float specR,float specG,float specB,float shininess,int color){
glEnable(GL_COLOR_MATERIAL);
GLfloat material_color[4] ={difR,difG,difB,1.0f};
GLfloat material_specular[4]={specR,specG,specB,1.0f};
glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,material_specular);
glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,material_color);
glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shininess);
glSphere(x,y,z,r,color);
GLfloat material_color_init [4] = {1,1,1,1.0f};
GLfloat material_specular_init [4] = {0.85,0.85,0.85,1.0};
glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, material_specular_init);
glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, material_color_init);
glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 128.0f);
glDisable(GL_COLOR_MATERIAL);

}

void glSphere(float x,float y,float z, float radius,int rgb){
    int n = 20;
    int m = 10;
    rgb = rgb % 3;
    float c[3] = {0,0,0};
    c[rgb] = 100.0f;
    float coordinates[n][m][5];
    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < m; j++)
        {
            polar2Cartesian(i / (float) (n-1) * 2 *M_PI, j / (float) (m-1) * M_PI, radius, coordinates[i][j][0], coordinates[i][j][1], coordinates[i][j][2]);
            coordinates[i][j][3] = (float) i / (float) (n-1);
            coordinates[i][j][4] = (float) j / (float) (m-1);
        }
    }

    glMatrixMode (GL_MODELVIEW);
    glPushMatrix ();
    glTranslatef (x, y, z);
    glBegin(GL_TRIANGLES);
    for(int i = 0; i < n-1; i++)
    {
        for(int j = 0; j < m-1; j++)
        {
           // glTexCoord2f (coordinates[i][j][3], coordinates[i][j][4]),
            //glColor3f(c[0], c[1], c[2]);
            glNormal3f(coordinates[i][j][0], coordinates[i][j][1], coordinates[i][j][2]);
            glVertex3f(coordinates[i][j][0], coordinates[i][j][1], coordinates[i][j][2]);

            //glTexCoord2f (coordinates[i+1][j][3], coordinates[i+1][j][4]);
            //glColor3f(c[0],c[1],c[2]);
            glNormal3f(coordinates[i][j][0], coordinates[i][j][1], coordinates[i][j][2]);
            glVertex3f(coordinates[i+1][j][0], coordinates[i+1][j][1], coordinates[i+1][j][2]);

            //glTexCoord2f (coordinates[i][j+1][3], coordinates[i][j+1][4]);
            //glColor3f(c[0],c[1],c[2]);
            glNormal3f(coordinates[i][j][0], coordinates[i][j][1], coordinates[i][j][2]);
            glVertex3f(coordinates[i][j+1][0], coordinates[i][j+1][1], coordinates[i][j+1][2]);

            //glTexCoord2f (coordinates[i][j+1][3], coordinates[i][j+1][4]);
            //glColor3f(c[0],c[1],c[2]);
            glNormal3f(coordinates[i][j][0], coordinates[i][j][1], coordinates[i][j][2]);
            glVertex3f(coordinates[i][j+1][0], coordinates[i][j+1][1], coordinates[i][j+1][2]);

            //glTexCoord2f (coordinates[i+1][j][3], coordinates[i+1][j][4]);
            //glColor3f(c[0],c[1],c[2]);
            glNormal3f(coordinates[i][j][0], coordinates[i][j][1], coordinates[i][j][2]);
            glVertex3f(coordinates[i+1][j][0], coordinates[i+1][j][1], coordinates[i+1][j][2]);

            //glTexCoord2f (coordinates[i+1][j+1][3], coordinates[i+1][j+1][4]);
            //glColor3f(c[0],c[1],c[2]);
            glNormal3f(coordinates[i][j][0], coordinates[i][j][1], coordinates[i][j][2]);
            glVertex3f(coordinates[i+1][j+1][0], coordinates[i+1][j+1][1], coordinates[i+1][j+1][2]);
	}
    }
    glEnd();
    glPopMatrix ();
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
    float dist_target = w0.normalize();

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
