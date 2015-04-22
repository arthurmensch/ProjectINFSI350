#include "BoundingMesh.h"
#include <cmath>
#include <GL/glut.h>
#include <string>

BoundingMesh::BoundingMesh() {
}

BoundingMesh * BoundingMesh::generate(const char * modelFilename, const char * modelCage) {
    Mesh bounded = Mesh();
    bounded.loadOFF(std::string(modelFilename));
    Mesh cage = Mesh();
    cage.loadOFF(std::string(modelCage));
    BoundingMesh * boundingMesh = new BoundingMesh(&bounded, &cage);
    return boundingMesh;
}

BoundingMesh::~BoundingMesh()
{
    delete cageInitial;
    delete bounded;
    delete oldBounded;
    delete cage;
    delete oldCage;
}


BoundingMesh::BoundingMesh(Mesh * m_bounded, Mesh * m_cage) {
    update = false;
    bounded = new Mesh(*m_bounded);
    cage = new Mesh(*m_cage);
    oldCage = new Mesh(*cage);
    cageInitial = new Mesh(*cage);
    oldBounded = new Mesh(*bounded);
    s = std::vector<float>(cage->T.size(),1.);
    olds = s;
    trianglesToChange = std::set<int>();
    verticesToChange = std::set<int>();
    computeCoordinates();
    makeChangeFull();
}

void BoundingMesh::updateCage() {
    moveCageTriangleIncr(10,Vec3f(0,-0.5,0));
    moveCageTriangleIncr(10,Vec3f(0,-0.5,0));
    moveCageTriangleIncr(10,Vec3f(0,-0.5,0));
}

float sign(float x) {
        return x < 0 ? - 1 : 1;
}

void BoundingMesh::computeCoordinates() {
    float eps = 1e-6;
    int Vsize_b = bounded->V.size();
    int Vsize_c = cage->V.size();
    int Tsize_c = cage->T.size();
    vertexCoordinates = std::vector<std::vector<float>>(Vsize_b);
    normalCoordinates = std::vector<std::vector<float>>(Vsize_b);
    for(unsigned int i = 0; i< bounded->V.size(); i++) {
        vertexCoordinates[i].resize(Vsize_c);
        normalCoordinates[i].resize(Tsize_c);
    }
    for(unsigned int i = 0; i< bounded->V.size(); i++) {
        Vertex eta = bounded->V[i];
        for(unsigned int j = 0; j< cage->T.size(); j++) {
            Triangle t = cage->T[j];
            Vec3f v[3];
            for(int l= 0; l < 3; l++)
                v[l] = cage->V[t.v[l]].p-eta.p;
            Vec3f n = t.computeNormal(*cage);
            Vec3f p = dot(v[0],n) * n;
            float s[3];
            float I[3];
            float II[3];
            Vec3f N[3];
            for(int l= 0; l < 3; l++) {
                s[l] = sign(dot(cross(v[l]-p,v[(l+1) %3]-p),n));
                I[l] = GCTriInt(p,v[l],v[(l+1) % 3],Vec3f(0,0,0));
                II[l] = GCTriInt(Vec3f(0,0,0),v[(l+1) % 3],v[l],Vec3f(0,0,0));
                N[l] = cross(v[(l+1) % 3],v[l]);
                N[l].normalize();
            }
            float Is= - fabs(s[0]*I[0]+s[1]*I[1]+s[2]*I[2]);
            normalCoordinates[i][j] = - Is;
            Vec3f w = n*Is + N[0] * II[0] + N[1] * II[1] + N[2] * II[2];
            if (w.squaredLength() > eps)
                for(int l= 0; l < 3; l++)
                    vertexCoordinates[i][t.v[l]] += dot(N[(l+1) % 3],w)/dot(N[(l+1) % 3],v[l]);
        }
    }
}

float BoundingMesh::GCTriInt(Vec3f p, Vec3f v1, Vec3f v2, Vec3f eta) {
    Vec3f v2mv1 = v2 - v1;
    Vec3f pmv1 = p - v1;
    Vec3f v1mp = v1 - p;
    Vec3f v2mp = v2 - p;
    v2mv1.normalize();
    pmv1.normalize();
    v1mp.normalize();
    v2mp.normalize();

    double cosalpha = (double) dot(v2mv1,pmv1);
    if (cosalpha > 1-1e-5)
        return 0;
    if(cosalpha < -1+1e-5)
         return 0;
    double cosbeta = (double) dot(v1mp,v2mp);
    if (cosbeta > 1-1e-5)
         return 0;
    if(cosbeta < -1+1e-5)
         return 0;

    double beta = std::acos(cosbeta);
    double sinalpha = std::sqrt(1-cosalpha*cosalpha);
    double sinbeta = std::sqrt(1-cosbeta*cosbeta);

    double lambda = (p-v1).squaredLength()*(1-cosalpha*cosalpha);
    double c = (p-eta).squaredLength();
    double I[2];
    double Sa[2] = {sinalpha, sinalpha*cosbeta+cosalpha*sinbeta};
    double Ca[2] = {-cosalpha, -cosalpha*cosbeta+sinalpha*sinbeta};
    for(int i = 0; i < 2; i++) {
        double S = Sa[i];
        double C = Ca[i];
        I[i] = 0;
        double logt = std::sqrt(lambda) * (std::log(2*std::sqrt(lambda)*S*S/((1-C)*(1-C)))+std::log(1-2*c*C/(c*(1+C)+lambda + std::sqrt(lambda*lambda+lambda*c*S*S))));
        double atant = std::atan(std::sqrt(c)*C / (std::sqrt(lambda + S*S*c)));
        if (isnan(logt)) {
            std::cerr << "nan ";
            return 0;
        }
        I[i] =  (logt + 2*std::sqrt(c)*atant)* (- sign(S) / 2);
    }
    double res = - 1 / (4*M_PI)* fabs (I[0]-I[1]-std::sqrt(c)*beta);

    return (float) res;
}


void BoundingMesh::save(const std::string & filename) {
    bounded->saveOFF(filename);
}

void BoundingMesh::moveCageVertexIncr(unsigned int vertexIndex, Vec3f targetVertex) {
    moveCageVertex(vertexIndex, cage->V[vertexIndex].p + targetVertex);
}

void BoundingMesh::moveCageTriangleIncr(unsigned int triangleIndex, Vec3f targetVertex) {
    for(unsigned int i = 0; i < 3; i++)
        moveCageVertexIncr(cage->T[triangleIndex].v[i], targetVertex);
}

void BoundingMesh::moveCageVertex(unsigned int vertexIndex, Vec3f targetVertex) {
    cage->V[vertexIndex].p = targetVertex;
    for(auto it = trianglesToChange.begin(); it != trianglesToChange.end(); ++it) {
        Vec3f u = cage->V[cage->T[*it].v[1]].p - cage->V[cage->T[*it].v[0]].p;
        Vec3f v = cage->V[cage->T[*it].v[2]].p - cage->V[cage->T[*it].v[0]].p;
        Vec3f ui = cageInitial->V[cageInitial->T[*it].v[1]].p - cageInitial->V[cageInitial->T[*it].v[0]].p;
        Vec3f vi = cageInitial->V[cageInitial->T[*it].v[2]].p - cageInitial->V[cageInitial->T[*it].v[0]].p;
        s[*it] = sqrt(u.squaredLength()*v.squaredLength()+ ui.squaredLength()*vi.squaredLength() -2*(dot(ui,vi)*dot(u,v)))/(sqrt(2*cross(ui,vi).squaredLength()));
    }
}

void BoundingMesh::prepareVertexCoordinatesOldBounded(unsigned int vertexIndex) {
    auto v = bounded->V.begin();
    for(auto v_old = oldBounded->V.begin(); v != oldBounded->V.end(); ++v)
            v_old->p = v->p - oldCage->V[vertexIndex].p;
}

void BoundingMesh::prepareTriangleCoordinatesOldBounded(unsigned int j, float s) {
    auto v = bounded->V.begin();
    for(auto v_old = oldBounded->V.begin(); v != oldBounded->V.end(); ++v)
        v_old->p = v->p - oldCage->T[j].computeNormal(*oldCage)*s;
}

void BoundingMesh::addVerticesToSelection(std::set<int> vertexIndices) {
    for(auto vertexIndex = vertexIndices.begin(); vertexIndex != vertexIndices.end(); ++vertexIndex) {
        verticesToChange.insert(*vertexIndex);
        prepareVertexCoordinatesOldBounded(*vertexIndex);
    }
    auto t_init = cageInitial->T.begin();
    auto s_it = s.begin();
    int j = 0;
    for(auto t = cage->T.begin(); t != cage->T.end(); ++t) {
        bool ownVertex = false;
        for(unsigned int l = 0; l < 3; l++)
            ownVertex |= (vertexIndices.find(t->v[l]) != vertexIndices.end());
        if (ownVertex) {
            trianglesToChange.insert(j);
            Vec3f u = oldCage->V[t->v[1]].p - oldCage->V[t->v[0]].p;
            Vec3f v = oldCage->V[t->v[2]].p - oldCage->V[t->v[0]].p;
            Vec3f ui = cageInitial->V[t_init->v[1]].p - cageInitial->V[t_init->v[0]].p;
            Vec3f vi = cageInitial->V[t_init->v[2]].p - cageInitial->V[t_init->v[0]].p;
            *s_it = sqrt(u.squaredLength()*v.squaredLength()+ ui.squaredLength()*vi.squaredLength() -2*(dot(ui,vi)*dot(u,v)))/(sqrt(2*cross(ui,vi).squaredLength()));
            prepareTriangleCoordinatesOldBounded(j,*s_it);
        }
        ++t_init;
        ++s_it;
        ++j;
    }
}

void BoundingMesh::removeVerticesFromSelection(std::set<int> vertexIndices) {

}

void BoundingMesh::addTrianglesToSelection(std::set<int> triangleIndices) {

}

void BoundingMesh::removeTrianglesFromSelection(std::set<int> triangleIndices) {

}

void BoundingMesh::release(bool validate) {
    if(!validate) { //cancel
        cage->T = oldCage->T;
        cage->recomputeNormals();
        makeChangeFull();
   }
    else {
        makeChangeFull();
        oldCage->T = cage->T;
        oldCage->recomputeNormals(); //refactor this
    }
}

void BoundingMesh::makeChange() {
        for(auto it = verticesToChange.begin(); it != verticesToChange.end(); ++it){
            auto coord_v = normalCoordinates.begin();
            auto v_old = oldBounded->V.begin();
            for(auto v = bounded->V.begin(); v != bounded->V.end(); ++v) {
                v->p = v_old->p + (*coord_v)[*it] * (cage->T[*it].computeNormal(*cage)*s[*it]);
                ++coord_v;
                ++v_old;
            }
        }
        for(auto it = verticesToChange.begin(); it != verticesToChange.end(); ++it){
            auto coord_v = vertexCoordinates.begin();
            auto v_old = oldBounded->V.begin();
            for(auto v = bounded->V.begin(); v != bounded->V.end(); ++v) {
                v->p = v_old->p + (*coord_v)[*it] * cage->V[*it].p;
                ++coord_v;
                ++v_old;
            }
        }
        bounded->recomputeNormals();
}

void BoundingMesh::makeChangeFull() {
    auto coord_v_normal = normalCoordinates.begin();
    auto coord_v_vertex = vertexCoordinates.begin();
    for(auto v = bounded->V.begin(); v != bounded->V.end(); ++v) {
        v->p = Vec3f(0,0,0);
        for(unsigned int j = 0; j < cage->T.size(); j++) {
                v->p += (*coord_v_normal)[j] * (cage->T[j].computeNormal(*cage)*s[j]);
                olds[j] = s[j];
        }
        for(unsigned int j = 0; j < cage->V.size(); j++) {
            v->p += (*coord_v_vertex)[j] * (cage->V[j].p);
            oldCage->V[j] = cage->V[j];
        }
        ++coord_v_normal;
        ++coord_v_vertex;
    }
    bounded->recomputeNormals();
}

void BoundingMesh::reset() {
    Mesh * cageDelete = cage;
    Mesh * oldCageDelete = oldCage;
    cage = new Mesh(*cageInitial);
    oldCage = new Mesh(*cage);
    s = std::vector<float>(cage->T.size(),1.);
    olds = s;
    delete cageDelete;
    delete oldCageDelete;
    makeChangeFull();
}

void BoundingMesh::draw() {
    GLint mode[2];
	glGetIntegerv( GL_POLYGON_MODE, mode );
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE ); // Wireframe for the bounding box
    //glEnable(GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    cage->draw();
    //glDisable(GL_BLEND);
    //glEnable( GL_DEPTH_TEST);
    glPolygonMode( GL_FRONT_AND_BACK, mode[1] );
    bounded->draw();

}
