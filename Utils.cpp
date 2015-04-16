#include "Utils.h"
#include "Camera.h"
#include "Vec3.h"
#include <math>

void grabber(int x, int y) {

    scale_x = camera.getNearPlane() *tan(camera.getFovAngle())*M_PI/180);
    scale_y = scale_x / camera.getAspectRatio();

    x = (2*x/width -1) * scale_x;
    y = (2*y/width -1) * scale_y;

    Vec3f ray = Vec3f(x,y,camera.getNearPlane());
}
