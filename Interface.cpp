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
	if(selectionMode){
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

void translateStruct(int x, int y, int lastX,int lastY,BoundingMesh *boundingMesh, Camera &camera, int indexAimed, bool &vertexMoving){
    if(vertexMoving){
        translateVertex(camera,boundingMesh,indexAimed,x,y,lastX,lastY);
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
		vertexMoving=false;
		translate = false ;
		rotate = false ;
		selectionMode = false ;
		scale = false ;
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
        if (!rotate) {
		if(boundingMesh->getTriangleSelection().size()>0){
                	boundingMesh->release(false);
			vertexMoving=false;
			translate = false;
            		scale = false;
			lastX = x;
			lastY = y;
            		beginTransformX = x;
            		beginTransformY = y;
            		rotate = true;
		}
        }
        else {
		boundingMesh->release(false);
        	rotate = false;
        }
        break;
    case 't':
    	if (!translate) {
		if(boundingMesh->getTriangleSelection().size()>0){//don't go into translate mode is no triangle selected
                	boundingMesh->release(false);
                	rotate = false;
			scale = false;
            		glutSetWindowTitle("Translation");
            		indexAimed=grabberVertex(x,y,boundingMesh,camera);
           		if(indexAimed>-1)//vertex grabbed
            	    		vertexMoving=true;
        		beginTransformX = x;
    	        	beginTransformY = y;
	        	lastX=x;
                	lastY=y;
             		translate = true;
		}
        }
        else {
            glutSetWindowTitle("Translation canceled");
            boundingMesh->release(false);
		vertexMoving=false;
            translate = false;
        }
        break;

    case 'e':
        if (!scale) {
		if(boundingMesh->getTriangleSelection().size()>0){
			boundingMesh->release(false);
			vertexMoving=false;
            		translate = false;
            		rotate = false;
			lastX = x;
			lastY = y;
            		beginTransformX = x;
            		beginTransformY = y;
            		scale = true;
		}
        }
        else {
		boundingMesh->release(false);
            scale = false;
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

        case 'c' :
            if(!selectionMode){
                beginTransformX=x;
                beginTransformY=y;
                selectionMode=true;
            }
            else{
                selectionMode = false;
            }
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
    if (translate) {
        translateStruct(lastX, lastY,beginTransformX,beginTransformY, boundingMesh,camera,indexAimed, vertexMoving);

    }
    if (rotate) {
        rotation(camera,boundingMesh, lastX, lastY, beginTransformX, beginTransformY);

    }

    if (scale) {
        scaling(camera,boundingMesh, lastX, lastY, beginTransformX, beginTransformY);

    }
    if(scale || rotate || translate)
        if(!count)
            boundingMesh->makeChange();
    camera.handleMouseMoveEvent (x, y);
    lastX = x;
    lastY = y;
}

void Interface::mouse (int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (translate){//last effective translation
            boundingMesh->release(true);
		vertexMoving=false;
            translate = false;
        }
        else if (rotate){
			boundingMesh->release(true);
            rotate = false;
	}
	else if(scale){
		boundingMesh->release(true);
		scale = false;
	}
	else if (selectionMode){
               	selectSquare(lastX,lastY,beginTransformX,beginTransformY,boundingMesh);
		selectionMode = false;
	}
    }

    else if (button == GLUT_RIGHT_BUTTON) {
        if (translate) {//cancel translation
            boundingMesh->release(false);
		vertexMoving=false;
            translate = false;
        }
        else if (rotate) {
            boundingMesh->release(false);
            rotate = false;
        }

        else if (scale) { // Cancel scale
			boundingMesh->release(false);
            scale = false ;
        }
	else if (selectionMode)
		selectionMode = false;
    }

    if (glutGetModifiers() != GLUT_ACTIVE_SHIFT) {
        camera.handleMouseClickEvent (button, state, x, y);
    }
}
