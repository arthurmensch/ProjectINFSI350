#include "Interface.h"

void Un_Select(int x, int y, Mesh &cage, Camera &camera, std::vector<bool> &selectedTriangle){
	int triangle=grabber(x,y,cage,camera);
	if(triangle>-1){
		if(selectedTriangle[(unsigned int)triangle])
			selectedTriangle[(unsigned int)triangle]=false;
		else
			selectedTriangle[(unsigned int)triangle]=true;
	}
}

void translateStruct(int x, int y, int lastX,int lastY,BoundingMesh &boundingMesh, Camera &camera, std::vector<bool> &selectedTriangle, int indexAimed, bool &vertexMoving,bool end){
if(vertexMoving){
	translateVertex(camera,boundingMesh,indexAimed,x,y,lastX,lastY);
	if(end)
		vertexMoving=false;
}
else{
	translateForm(camera,boundingMesh,selectedTriangle,indexAimed,x,y,lastX,lastY);
}


}
Interface::Interface() {
	glutMotionFunc (motion);
    glutPassiveMotionFunc (passiveMotion);
	glutReshapeFunc (reshape);
    glutKeyboardFunc (keyDown);
    glutKeyboardUpFunc(keyUp);
    glutMouseFunc (mouse);
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
    case 'q':
    case 27:
        exit (0);
        break;
    case 'w':
        GLint mode[2];
		glGetIntegerv (GL_POLYGON_MODE, mode);
		glPolygonMode (GL_FRONT_AND_BACK, mode[1] ==  GL_FILL ? GL_LINE : GL_FILL);
        break;
    case 'l':
        mesh.smooth();
        break;
    case 'g':
        mesh.smoothGeom();
        break;
    case 'r':
        if (!rotate) {
        	if (translate) {
            	translateStruct(beginTransformX, beginTransformY,lastX,lastY, *boundingMesh,camera,selectedTriangle,indexAimed, vertexMoving,true);
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
            indexAimed=grabberVertex(x,y,*(boundingMesh->cage),camera,selectedTriangle);
            if(indexAimed>-1)//vertex grabbed
                vertexMoving=true;
            else{//if no vertex grabbed try grab triangle
                indexAimed=grabberForm(x,y,*(boundingMesh->cage),camera,selectedTriangle);
            }
            if(indexAimed>-1){//triangle or vertex grabbed
                beginTransformX = x;
                beginTransformY = y;
                lastX=x;
                lastY=y;
                translate = true;
            }
            else{//nothing grabbed, you should aim properly dude
                translate=false;
                glutSetWindowTitle("Nothing selected, try again");
            }
        }
        else {//cancel translation
            translateStruct(beginTransformX, beginTransformY,lastX,lastY, *boundingMesh,camera,selectedTriangle,indexAimed, vertexMoving,true);
            translate = false;
        }
        break;
    case 's':
    	selectionMode = true;
    	glutSetWindowTitle ("Selection");
        Un_Select(x,y,*(boundingMesh->cage),camera,selectedTriangle);
    	break;
    case '3':
        mesh.simplifyMesh(12);
        break;
    case 'u':
        boundingMesh->updateCage();
        break;
    case 'i':
        boundingMesh->reset();
        break;
    case 'n':
        boundingMesh->save("models/saved.off");
    default:
        break;
    }
}

void Interface::keyUp(unsigned char keyReleased, int x, int y) {
	switch(keyReleased) {
	case 's':
		selectionMode = false;
		glutSetWindowTitle ("Plus Selection");
		break;
	}

}

void Interface::motion (int x, int y) {
    camera.handleMouseMoveEvent (x, y);
}

void Interface::passiveMotion (int x, int y) {
    if (translate) {//translation
    translateStruct(x, y,lastX,lastY, *boundingMesh,camera,selectedTriangle,indexAimed, vertexMoving,false);
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
	    translateStruct(x, y,lastX,lastY, *boundingMesh,camera,selectedTriangle,indexAimed, vertexMoving,true);
            translate = false;
	    boundingMesh->makeChange();
        }
	else if (rotate)
            rotate = false;
    }

    else if (button == GLUT_RIGHT_BUTTON) {
        if (translate) {//cancel translation
	    translateStruct(beginTransformX,beginTransformY,lastX,lastY, *boundingMesh,camera,selectedTriangle,indexAimed, vertexMoving,true);
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
