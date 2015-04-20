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
        	if (translate)
        		translate = false;
        	rotate = true;
        }
        else
        	rotate = false;
        break;
    case 't':
    	if (!translate) {
    		if (rotate)
    			rotate = false;
    		translate = true;
    	}
    	else
    		translate = false;
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

void Interface::mouse (int button, int state, int x, int y) {
	if(selectionMode){
		int triangle=grabber(x,y,*(boundingMesh->cage),camera);

		if(triangle>-1){
			selectionMode=true;
			selectedTriangle[(unsigned int)triangle]=true;
		}
//		}
//	}
//	if(rotate||translate){
//	modifyBoundingMesh();
	}
    if (glutGetModifiers() != GLUT_ACTIVE_SHIFT) {
        camera.handleMouseClickEvent (button, state, x, y);
    }
    else {
    }
}
