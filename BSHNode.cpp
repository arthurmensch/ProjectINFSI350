#include "Mesh.h"
#include "BSHNode.h"
#include "Vec3.h"
#include "Utils.h"
#include "Camera.h"

#include <iostream>
#include <algorithm>
#include <list>
#include <set>


BSHNode::BSHNode(std::vector<Vertex> &V, std::vector<Triangle> &T, int _depth) {
    Vec3f h; // Separation plane normal
    Vec3f x[3]; //x,y,z
    Vec3f ht;
    float dmax;
    float d;

    this->Tsize = T.size();
    this->depth = _depth;

    std::set<Vertex*> usedV;

    for(int i = 0; i < this->Tsize; i++) {
        for(int k = 0; k < 3; k++) {
                usedV.insert(&V[T[i].v[k]]);
        }
    }

    int Vsize = usedV.size();
    this->vertices = &V;

    this->normal = Vec3f(0,0,0);

    //Bounding sphere computation

    //Approximate center
    x[0] = V[0].p;

    for(int l = 0; l < 2; l++) {
        dmax = 0;
        for(Vertex * V : usedV) {
            this->normal += V->n;
            d = (x[l] - V->p).length();
            if (d > dmax) {
                x[l+1] = V->p;
                dmax = d;
            }
        }
        //dmax = ||x-y|| at the end of l = 0
    }
    this->normal.normalize();
    this->position = (x[2]+x[1])/2;
    h = x[2] - this->position; //Approximate radius initialization
    this->radius = h.length();

    //Radius computation
    for(Vertex * V : usedV) {
        ht = V->p - this->position;
        d = ht.length();
        if (d > this->radius) {
            h = ht;
            this->radius = h.length();
        }
    }

    Vec3f mean;
    for(Vertex * V : usedV) {
        mean = mean + V->p;
    }
    mean /= Vsize;


    std::cerr << std::endl
              << "New bounding sphere"
              << std::endl
              << "Depth : " << this->depth
              << std::endl
              << "Num triangles : " << this->Tsize
              << std::endl
              << "Radius : " << this->radius
              << std::endl
              << "Position : " << this->position
              << std::endl
              << "Normal : " << this->normal;
    //Left Right separation

    std::vector<Triangle> Tleft = std::vector<Triangle>();
    std::vector<Triangle> Tright = std::vector<Triangle>();

    for(int i = 0; i < this->Tsize; i++) {
        bool left = false;
        for(int k = 0; k < 3; k++) {
             if (dot(V[T[i].v[k]].p-mean,h) > 0) {
                 left = true;
             }
        }
        if (left) {
            Tleft.push_back(T[i]);
        }
        else {
            Tright.push_back(T[i]);
        }
    }

    std::cerr << std::endl
              << "Separation :"
              << std::endl
              << "Left : " << Tleft.size()
              << std::endl
              << "Right : " << Tright.size();


    if (!Tleft.empty() && !Tright.empty()) {

        std::cerr << std::endl
                  << "Fork";

        this->isLeaf = false;
        this->leftChild = new BSHNode(V,Tleft,this->depth + 1);
        this->rightChild = new BSHNode(V,Tright,this->depth + 1);
    }
    else { //If leaf, store triangles
        this->isLeaf = true;
        if(Tleft.empty()) {
            this->triangles = Tright;
        }
        else {
            this->triangles = Tleft;
        }
        this->leftChild = nullptr;
        this->rightChild = nullptr;
    }
}

BSHNode::~BSHNode() {
    delete this->leftChild;
    delete this->rightChild;
}

void BSHNode::drawGL(Vec3f camPos) {
    glDisk(this->position, this->normal, this->radius, camPos);
}

void BSHNode::renderRecursive(Vec3f camPos) {
        float dist = (this->position - camPos).length();
        if(this->radius < 0.1 && dist > 3) {
            this->drawGL(camPos);
        }
        else if (!this->isLeaf) {
            this->leftChild->renderRecursive(camPos);
            this->rightChild->renderRecursive(camPos);
        }
        else {
            glTriangles(*(this->vertices), this->triangles, camPos);
        }
}

