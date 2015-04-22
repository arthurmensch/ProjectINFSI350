// --------------------------------------------------------------------------
// Copyright(C) 2009-2015
// Tamy Boubekeur
//
// All rights reserved.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License (http://www.gnu.org/licenses/gpl.txt)
// for more details.
// --------------------------------------------------------------------------

#include "Mesh.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <map>
#include <set>
#include <GL/glut.h>

using namespace std;

Mesh::Mesh() {
}

Mesh::~Mesh() {

}

Mesh::Mesh(const Mesh & mesh) {
    T = mesh.T;
    V = mesh.V;
    recomputeNormals();
}

Mesh & Mesh::operator=(const Mesh & mesh) {
    T = mesh.T;
    V = mesh.V;
    recomputeNormals();}

void Mesh::loadOFF (const std::string & filename) {
	ifstream in (filename.c_str ());
    if (!in)
        exit (1);
	string offString;
    unsigned int sizeV, sizeT, tmp;
    in >> offString >> sizeV >> sizeT >> tmp;
    V.resize (sizeV);
    T.resize (sizeT);
    for (unsigned int i = 0; i < sizeV; i++)
        in >> V[i].p;
    int s;
    for (unsigned int i = 0; i < sizeT; i++) {
        in >> s;
        for (unsigned int j = 0; j < 3; j++)
            in >> T[i].v[j];
    }
    in.close ();
    //centerAndScaleToUnit ();
    recomputeNormals ();
}

void Mesh::saveOFF(const std::string & filename) {
    unsigned int sizeV = V.size();
    unsigned int sizeT = T.size();
    ofstream out (filename.c_str());
    if(!out) {
        exit(1);
    }
    out << "OFF" << std::endl << sizeV << " " << sizeT << " " << 0 << std::endl;
    for (unsigned int i = 0; i < sizeV; i++)
        out << V[i].p << std::endl;
    for (unsigned int i = 0; i < sizeT; i++) {
        out << 3;
        for (unsigned int j = 0; j < 3; j++)
            out << " " <<T[i].v[j];
        out << std::endl;
    }
    out.close();
}

void Mesh::recomputeNormals () {
    for (unsigned int i = 0; i < V.size (); i++)
        V[i].n = Vec3f (0.0, 0.0, 0.0);
    for (unsigned int i = 0; i < T.size (); i++) {
        Vec3f e01 = V[T[i].v[1]].p -  V[T[i].v[0]].p;
        Vec3f e02 = V[T[i].v[2]].p -  V[T[i].v[0]].p;
        Vec3f n = cross (e01, e02);
        n.normalize ();
        for (unsigned int j = 0; j < 3; j++)
            V[T[i].v[j]].n += n;
    }
    for (unsigned int i = 0; i < V.size (); i++)
        V[i].n.normalize ();
}

void Mesh::centerAndScaleToUnit () {
    Vec3f c;
    for  (unsigned int i = 0; i < V.size (); i++)
        c += V[i].p;
    c /= V.size ();
    float maxD = dist (V[0].p, c);
    for (unsigned int i = 0; i < V.size (); i++){
        float m = dist (V[i].p, c);
        if (m > maxD)
            maxD = m;
    }
    for  (unsigned int i = 0; i < V.size (); i++)
        V[i].p = (V[i].p - c) / maxD;
}

void Mesh::smooth() {
    typedef std::set<unsigned int> InnerSet;
    typedef std::map<unsigned int, InnerSet> OuterMap;
    OuterMap connectivity = OuterMap();

    std::vector<Vec3f> sumNeighbours = std::vector<Vec3f>(V.size(), Vec3f(0,0,0));
    int degree[V.size()];

    for(Triangle t : T) {
        for(unsigned int i = 0; i < 3 ; i++) {
            OuterMap::iterator outer_iter = connectivity.find(t.v[i]);
            if(outer_iter != connectivity.end()) {
                InnerSet &innerSet = outer_iter->second;
                innerSet.insert(t.v[(i+2) % 3]);
                innerSet.insert(t.v[(i+1) % 3]);
            }
            else {
                connectivity.insert(pair<unsigned int, InnerSet>(t.v[i], InnerSet({t.v[(i+2) % 3], t.v[(i+1) % 3]})));
            }
        }
    }

    for(unsigned int j = 0; j < V.size(); j++) {
            int n = 0;
            for(auto iter=connectivity.find(j)->second.begin(); iter!=connectivity.find(j)->second.end();++iter){
                sumNeighbours[j] += V[*iter].p;
                n++;
            }
            degree[j] = n;
    }
    for(unsigned int j = 0; j < V.size(); j++) {
        V[j].p = sumNeighbours[j] / degree[j];
    }
    recomputeNormals();
}

void Mesh::smoothGeom() {

    std::vector<Vec3f> sumNeighbours = std::vector<Vec3f>(V.size(), Vec3f(0,0,0));
    std::vector<float>area = std::vector<float>(V.size(),0);

    for(Triangle t : T) {
        for(unsigned int i = 0; i < 3 ; i++) {
                // Edge 0 - 1
                Vec3f a = V[t.v[i]].p - V[t.v[(i+2) % 3]].p; //20
                Vec3f b = V[t.v[(i+1) % 3]].p - V[t.v[(i+2) % 3]].p; //21
                float cos2_ab = dot(a,b)*dot(a,b);
                float sin2_ab = 1 - cos2_ab;
                float cot_ab= sqrt(cos2_ab / sin2_ab);
                area[t.v[i]] += cot_ab;
                area[t.v[(i+1) % 3]] += cot_ab;
                Vec3f add = 0.5f * cot_ab * (V[t.v[i]].p - V[t.v[(i+1) % 3]].p);
                sumNeighbours[t.v[i]] += add;
                sumNeighbours[t.v[(i+1) % 3]] -= add;
        }
    }

    for(unsigned int j = 0; j < V.size(); j++) {
        V[j].p -= sumNeighbours[j] / area[j];
    }
    recomputeNormals();
}

void Mesh::draw() {

    glBegin (GL_TRIANGLES);
    for (unsigned int i = 0; i < T.size (); i++)
        for (unsigned int j = 0; j < 3; j++) {
            const Vertex & v = V[T[i].v[j]];
            glColor4f(1,1,1,0.5);
            glNormal3f (v.n[0], v.n[1], v.n[2]); // Specifies current normal vertex
            glVertex3f (v.p[0], v.p[1], v.p[2]); // Emit a vertex (one triangle is emitted each time 3 vertices are emitted)
        }
    glEnd ();
}

void Mesh::draw(Vec3i selectedColor) {
    // Normal Triangles
    glBegin (GL_TRIANGLES);
    for (unsigned int i = 0; i < T.size (); i++) {
        if (!selectedTriangle[i]) {
            for (unsigned int j = 0; j < 3; j++) {
                const Vertex & v = V[T[i].v[j]];
                glNormal3f (v.n[0], v.n[1], v.n[2]); // Specifies current normal vertex
                glVertex3f (v.p[0], v.p[1], v.p[2]); // Emit a vertex (one triangle is emitted each time 3 vertices are emitted)
            }
        }
    }
    glEnd ();

    // Selected Triangles
    glDisable(GL_LIGHTING);
    glEnable(GL_POLYGON_OFFSET_LINE);
    glBegin (GL_TRIANGLES);
    for (unsigned int i = 0; i < T.size (); i++) {
        if (selectedTriangle[i]) {
            for (unsigned int j = 0; j < 3; j++) {
                const Vertex & v = V[T[i].v[j]];
                glColor3f(selectedColor[0], selectedColor[1], selectedColor[2]);
                glNormal3f (v.n[0], v.n[1], v.n[2]); // Specifies current normal vertex
                glVertex3f (v.p[0], v.p[1], v.p[2]); // Emit a vertex (one triangle is emitted each time 3 vertices are emitted)
            }
            
        }
    }

    glEnd ();
    glDisable(GL_POLYGON_OFFSET_LINE);
    glEnable(GL_LIGHTING);
}

void Mesh::simplifyMesh(unsigned int r) {
    Vec3f Vmin, Vmax;
    for(Vertex v : V) {
        for(int i = 0; i < 3; i++) {
            if (v.p[i] < Vmin[i])
                Vmin[i] = v.p[i];
            else if( v.p[i] > Vmax[i])
                Vmax[i] = v.p[i];
        }
    }
    float edge = 0;
    for(int i = 0; i < 3; i++) {
        float x = Vmax[i] - Vmin[i];
        if (x > edge)
            edge = x;
    }
    for(int j = 0; j < 3; j++)
        Vmin[j] -= 0.05 * edge;
    edge *= 1.1;

    float divider = edge / r;
    unsigned int r3 = r*r*r;
    unsigned int r2 = r*r;

    std::vector<Vertex> newV = std::vector<Vertex>(r3,Vertex(Vec3f(0,0,0),Vec3f(0,0,0)));
    int num[r3];
    for(unsigned int i = 0; i < r3; i++)
        num[i] = 0;

    for (auto it = T.begin(); it != T.end();) {
        for(int i = 0; i < 3 ; i++) {
            Vec3f coordinates = (V[it->v[i]].p-Vmin) / divider;
            unsigned int rep[3] = {0,0,0};
            for(int j = 0; j < 3; j ++) {
                rep[j] = (unsigned int) floor(coordinates[j]);
            }
            unsigned int index = r2*rep[0]+r*rep[1]+rep[2];
            newV[index].p += V[it->v[i]].p;
            num[index] += 1;
            it->v[i] = index;
        }
        if  (it->v[0] == it->v[1] ||  it->v[1] == it->v[2] ||it->v[2] == it->v[0]) {
            it = T.erase(it);
        }
        else {
            ++it;
        }
    }

    for(unsigned int i = 0; i < r3; i++) {
        if(num[i] != 0) {
            newV[i].p /= num[i];
       }
    }

    V = newV;
    recomputeNormals();

    for(auto it = T.begin(); it != T.end(); ++it) {
        for(int j = 0; j < 3; j ++) {
          std::cerr << V[it->v[j]].p << std::endl;
        }
    }
}
