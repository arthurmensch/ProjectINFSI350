#include "Interface.h"

void Un_Select(int x, int y, BoundingMesh *boundingMesh, Camera &camera){
	int triangle=grabber(x,y,boundingMesh,camera);
	if(triangle>-1){
		bool selected=boundingMesh->triangleIsSelected(triangle);
		std::set<int> triangleIndex;
		triangleIndex.insert(triangle);
		if(selected)
			boundingMesh->removeTrianglesFromSelection(triangleIndex);
		else
			boundingMesh->addTrianglesToSelection(triangleIndex);
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
	translateVertex(camera,boundingMesh,indexAimed,x,y,lastX,lastY);
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
        	if (translate) {
            	translateStruct(beginTransformX, beginTransformY,lastX,lastY, boundingMesh,camera,indexAimed, vertexMoving,true);
                translate = false;
            }

            beginTransformX = x;
            beginTransformY = y;
        	rotate = true;
        }
        else {
            rotation(beginTransformX,lastX,beginTransformY,lastY,beginTransformX,beginTransformY);
        	rotate = false;
        }
        break;
    case 't':
    	if (!translate) {
    		if (rotate) {
                rotation(beginTransformX,lastX,beginTransformY,lastY,beginTransformX,beginTransformY);
                rotate = false;
            }
            glutSetWindowTitle("Translation");
            indexAimed=grabberVertex(x,y,boundingMesh,camera);
            if(indexAimed>-1)//vertex grabbed
                vertexMoving=true;
            /*else{//if no vertex grabbed try grab triangle
                indexAimed=grabberForm(x,y,*(boundingMesh->cage),camera,selectedTriangle);
            }*/
            //if(indexAimed>-1){//triangle or vertex grabbed
                beginTransformX = x;
                beginTransformY = y;
                lastX=x;
                lastY=y;
                translate = true;
            //}
            /*else{//nothing grabbed, you should aim properly dude
                translate=false;
                glutSetWindowTitle("Nothing selected, try again");
            }*/
        }
        else {//cancel translation
            glutSetWindowTitle("Translation canceled");
            translateStruct(beginTransformX, beginTransformY,lastX,lastY, boundingMesh,camera,indexAimed, vertexMoving,true);
            translate = false;
        }
        break;
    case 's':
		beginTransformX=x;
		beginTransformY=y;
    	selectionMode = true;
    	glutSetWindowTitle ("Selection");
        Un_Select(x,y,boundingMesh,camera);
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
		selectSquare(x,y,beginTransformX,beginTransformY,boundingMesh);
		selectionMode = false;
		glutSetWindowTitle ("End selection");
		break;
	}

}

void Interface::motion (int x, int y) {
    camera.handleMouseMoveEvent (x, y);
}

void Interface::passiveMotion (int x, int y) {
    count++;
    if (count == 20)
        count = 0;
    if (translate) {//translation
        translateStruct(lastX, lastY,beginTransformX,beginTransformY, boundingMesh,camera,indexAimed, vertexMoving,false);
    }

    if (rotate) {
        rotation(lastX, x, lastY, y,beginTransformX,beginTransformY);
    }

    camera.handleMouseMoveEvent (x, y);

    lastX = x;
    lastY = y;
}

void Interface::mouse (int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (translate){//last effective translation
            translateStruct(x, y,lastX,lastY, boundingMesh,camera,indexAimed, vertexMoving,true);
            translate = false;
        }
	else if (rotate)
            rotate = false;
    }

    else if (button == GLUT_RIGHT_BUTTON) {
        if (translate) {//cancel translation
            translateStruct(beginTransformX, beginTransformY,lastX,lastY, boundingMesh,camera,indexAimed, vertexMoving,true);
            translate = false;
        }

        else if (rotate) {
            rotation(beginTransformX,lastX,beginTransformY,lastY,beginTransformX,beginTransformY);
            rotate = false;
        }
    }

	if(selectionMode){
//		Un_Select(x,y,*(boundingMesh->cage),camera,selectedTriangle);
	}

    if (glutGetModifiers() != GLUT_ACTIVE_SHIFT) {
        camera.handleMouseClickEvent (button, state, x, y);
    }

    else {
    }
}
