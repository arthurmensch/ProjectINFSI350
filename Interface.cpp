#include "Interface.h"

void toggleSelect(int x, int y, BoundingMesh *boundingMesh, Camera &camera){
	int triangle=grabber(x,y,boundingMesh,camera);
	if(triangle>-1){
		bool selected=boundingMesh->triangleIsSelected(triangle);
		std::set<int> triangleIndex = std::set<int>();
        	triangleIndex.insert(triangle);
		if(selected)
			boundingMesh->removeTrianglesFromSelection(triangleIndex);
		else
			boundingMesh->addTrianglesToSelection(triangleIndex);
	}
}

void displayCarre(){
	if(transformState == SELECTION){
		glQuadSelect(lastX,lastY,beginTransformX,beginTransformY);
	}
}

void selectSquare(int x, int y, int lastX, int lastY,BoundingMesh *boudingMesh){
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projection);
	GLdouble modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelview);
	GLdouble winX,winY,winZ;
	Mesh *cage=boundingMesh->getCage();
	std::set<int> trianglesIndex;
	for (unsigned int i=0;i<cage->T.size();i++){
		int nbvertex=0;
		for (int j=0;j<3;j++){
			Vec3f currentPoint=cage->V[cage->T[i].v[j]].p;
			gluProject((double)currentPoint[0],(double)currentPoint[1],(double)currentPoint[2],modelview,projection,viewport,&winX,&winY,&winZ);
			winY=viewport[3]-winY;
			if((x-lastX)*(winX-lastX)>0 && (y-lastY)*(winY-lastY)>0 && (x-lastX)*(x-winX)>0&&(y-lastY)*(y-winY)>0)

				nbvertex++;
		}
		if(nbvertex==3)
			trianglesIndex.insert(i);
	}
	boundingMesh->addTrianglesToSelection(trianglesIndex);
}

void translateStruct(int x, int y, int lastX,int lastY,BoundingMesh *boundingMesh, Camera &camera, int indexAimed, bool &vertexMoving,bool end){
    if(vertexMoving){
        if(indexAimed > -1)
            translateVertex(camera,boundingMesh,indexAimed,x,y,lastX,lastY);
        else
            vertexMoving = false;
    	if(end)
            vertexMoving=false;
    }
    else{
    	translateForm(camera,boundingMesh,x,y,lastX,lastY);
    }
}

Interface::Interface() {
	glutMotionFunc (motion);
    glutPassiveMotionFunc (passiveMotion);
	glutReshapeFunc (reshape);
    glutKeyboardFunc (keyDown);
    glutKeyboardUpFunc(keyUp);
    glutMouseFunc (mouse);
    count = 0;
}

void Interface::keyDown (unsigned char keyPressed, int x, int y) {
    switch (keyPressed) {
    case 'f':
        if (fullScreen) {
            glutReshapeWindow (camera.getScreenWidth (), camera.getScreenHeight ());
            fullScreen = false;
        } else {
            glutFullScreen ();
            fullScreen = true;
        }
        break;
	case 'a':
		boundingMesh->release(false);
		transformState = NONE;
		boundingMesh->clearSelection();
		break;
    case 'q':
    case 27:
        exit (0);
        break;
    case 'w':
        GLint mode[2];
		glGetIntegerv (GL_POLYGON_MODE, mode);
		glPolygonMode (GL_FRONT_AND_BACK, mode[1] ==  GL_FILL ? GL_LINE : GL_FILL);
        break;
    case 'r':
        if (transformState != ROTATE) {
		if(boundingMesh->getTriangleSelection().size()>0){
            boundingMesh->release(true);
			lastX = x;
			lastY = y;
    		beginTransformX = x;
    		beginTransformY = y;
            transformState = ROTATE;
		}
        }
        else {
            boundingMesh->release(false);
        	transformState = NONE;
        }
        break;
    case 't':
    	if (transformState != TRANSLATE) {
		if(boundingMesh->getTriangleSelection().size()>0){//don't go into translate mode is no triangle selected
            boundingMesh->release(true);
            glutSetWindowTitle("Translation");
            indexAimed=grabberVertex(x,y,boundingMesh,camera);

        	if(indexAimed>-1)//vertex grabbed
            	vertexMoving=true;

            beginTransformX = x;
    	    beginTransformY = y;
	        lastX=x;
            lastY=y;
            transformState = TRANSLATE;
		}
        }
        else {
            glutSetWindowTitle("Translation canceled");
            boundingMesh->release(false);
            transformState = NONE;
        }
        break;

    case 'e':
        if (transformState != SCALE) {
		if(boundingMesh->getTriangleSelection().size()>0){
			boundingMesh->release(true);
			lastX = x;
			lastY = y;
            beginTransformX = x;
            beginTransformY = y;
            transformState = SCALE;
		}
        }
        else {
            boundingMesh->release(false);
            transformState = NONE;
        }
        break;

    case 'c' :
        if(transformState != SELECTION){
            beginTransformX=x;
            beginTransformY=y;
            transformState = SELECTION;
        }
        else{
            transformState = NONE;
        }
        break;

    case 'm':
        boundingMesh->updateCage();
        break;
    case 'z':
        boundingMesh->reset();
        break;
    case 'x':
        boundingMesh->save("models/saved.off");
        glutSetWindowTitle("Saved");
    default:
        break;
    }
}

void Interface::keyUp(unsigned char keyReleased, int x, int y) {
	switch(keyReleased) {
        case 's':
    		glutSetWindowTitle ("Selection");
        	toggleSelect(x,y,boundingMesh,camera);
		break;
	}
}

void Interface::motion (int x, int y) {
    camera.handleMouseMoveEvent (x, y);
}

void Interface::passiveMotion (int x, int y) {
    count++;
    if(count == 5)
        count = 0;

    switch (transformState) {
        case TRANSLATE:
            translateStruct(lastX, lastY,beginTransformX,beginTransformY, boundingMesh,camera,indexAimed, vertexMoving,false);
            break;
        case ROTATE:
            rotation(camera,boundingMesh, lastX, lastY, beginTransformX, beginTransformY);
            break;
        case SCALE:
            scaling(camera,boundingMesh, lastX, lastY, beginTransformX, beginTransformY);
            break;
    }

    if (transformState != NONE && transformState != SELECTION)
        if(!count)
            boundingMesh->makeChange();
    camera.handleMouseMoveEvent (x, y);
    lastX = x;
    lastY = y;
}

void Interface::mouse (int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (transformState == SELECTION)
            selectSquare(lastX,lastY,beginTransformX,beginTransformY,boundingMesh);

        else if (transformState != NONE) //last effective transformation
            boundingMesh->release(true);

        transformState = NONE;
    }

    else if (button == GLUT_RIGHT_BUTTON) {
        if (transformState != NONE && transformState != SELECTION)
            boundingMesh->release(false);
    }

    if (glutGetModifiers() != GLUT_ACTIVE_SHIFT) {
        camera.handleMouseClickEvent (button, state, x, y);
    }
}
