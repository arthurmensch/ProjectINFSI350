#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <GL/glut.h>

#include "Camera.h"
#include "BoundingMesh.h"
#include "Utils.h"

extern Camera camera;
extern Mesh mesh;
extern BoundingMesh *boundingMesh;
extern std::vector<bool> selectedTriangle;
extern std::string globalName;


static bool fullScreen = false;
static bool selectionMode = false;
static bool rotate = false;
static bool translate = false;
static bool scale = false;

static int beginTransformX;
static int beginTransformY;
static bool vertexMoving=false;
static int indexAimed;
static int lastX;
static int lastY;
static int count;

//general functions for movemement,selection/deselection etc
void displayCarre();
void toggleSelect(int x, int y,BoundingMesh *boundingMesh, Camera &camera);
void selectSquare(int x, int y, int lastX, int lastY,BoundingMesh *boundingMesh);

void translateStruct(int x, int y,int lastX,int lastY,BoundingMesh *boundingMesh, Camera &camera,int indexAimed,bool &vertexMoving);

class Interface {
public:
	Interface();
	static void motion (int x, int y);
	static void passiveMotion (int x, int y);
	static void reshape(int w, int h) {camera.resize (w, h);}
	static void keyDown (unsigned char keyPressed, int x, int y);
	static void keyUp(unsigned char keyReleased, int x, int y);
	static void mouse (int button, int state, int x, int y);
};

