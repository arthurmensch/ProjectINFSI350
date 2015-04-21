#include "Interface.h"

Interface::Interface() {
	glutMotionFunc (motion);
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
			if(vertexMoving){//Bring back to original vertex place
				translateVertex(camera,*boundingMesh,indexMoving,beginTransformX,beginTransformY,lastX,lastY);
				vertexMoving=false;
			}
			else{//bring back to original triangle place
				translateTriangle(camera,*boundingMesh,indexMoving,beginTransformX,beginTransformY,lastX,lastY);
            		}
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
		indexMoving=grabberVertex(x,y,*(boundingMesh->cage),camera,selectedTriangle);
		if(indexMoving>-1)//vertex grabbed
			vertexMoving=true;
		else{//if no vertex grabbed try grab triangle
			indexMoving=grabber(x,y,*(boundingMesh->cage),camera);
		}
		if(indexMoving>-1){//triangle grabbed
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
		if(vertexMoving){//Bring back to original vertex place
			translateVertex(camera,*boundingMesh,indexMoving,beginTransformX,beginTransformY,lastX,lastY);
			vertexMoving=false;
		}
		else{//bring back to original triangle place
			translateTriangle(camera,*boundingMesh,indexMoving,beginTransformX,beginTransformY,lastX,lastY);
            	}
		translate = false;
        }
    	break;
    case 's':
    	selectionMode = true;
    	glutSetWindowTitle ("Selection");
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
    if (translate) {
	if(vertexMoving){
		translateVertex(camera,*boundingMesh,indexMoving,x,y,lastX,lastY);
	}
	else{
		translateTriangle(camera,*boundingMesh,indexMoving,x,y,lastX,lastY);
	}
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
        if (translate){
		if(vertexMoving){
			translateVertex(camera,*boundingMesh,indexMoving,x,y,lastX,lastY);
			vertexMoving=false;
		}
		else{
			translateTriangle(camera,*boundingMesh,indexMoving,x,y,lastX,lastY);
		}
            	translate = false;
		boundingMesh->makeChange();
        }
	else if (rotate)
            rotate = false;
    }

    else if (button == GLUT_RIGHT_BUTTON) {
        if (translate) {
		if(vertexMoving){//Bring back to original vertex place
			translateVertex(camera,*boundingMesh,indexMoving,beginTransformX,beginTransformY,lastX,lastY);
			vertexMoving=false;
		}
		else{//bring back to original triangle place
			translateTriangle(camera,*boundingMesh,indexMoving,beginTransformX,beginTransformY,lastX,lastY);
		}
            	translate = false;
        }

        else if (rotate) {
            rotation(beginTransformX,lastX,beginTransformY,lastY,beginTransformX,beginTransformY);
            rotate = false;
        }
    }

	if(selectionMode){
		int triangle=grabber(x,y,*(boundingMesh->cage),camera);

		if(triangle>-1)
			selectedTriangle[(unsigned int)triangle]=true;
	}

    if (glutGetModifiers() != GLUT_ACTIVE_SHIFT) {
        camera.handleMouseClickEvent (button, state, x, y);
    }

    else {
    }
}
