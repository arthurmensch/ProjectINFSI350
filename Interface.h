#pragma once

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

static bool fullScreen = true;
static bool selectionMode = true;

class Interface {
public:
	Interface();
	static void motion (int x, int y) {camera.handleMouseMoveEvent (x, y);}
	static void reshape(int w, int h) {camera.resize (w, h);}
	static void key (unsigned char keyPressed, int x, int y);
	static void mouse (int button, int state, int x, int y);
};

