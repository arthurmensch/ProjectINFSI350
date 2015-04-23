#include "BoundingMesh.h"
#include <cmath>
#include <GL/glut.h>
#include <string>
#include "Utils.h"
#include <omp.h>

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
    s = std::vector<float>(cage->T.size(),1.);
    olds = s;
    trianglesToChange = std::set<int>();
    verticesToChange = std::set<int>();
    selectedTriangles = std::set<int>();
    computeCoordinates();
    makeChangeFull();
    oldBounded = new Mesh(*bounded);
    //prepareOldBounded(verticesToChange, trianglesToChange, false);
}

void BoundingMesh::updateCage() {
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
    #pragma omp parallel for
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
    moveCageVertex(vertexIndex, oldCage->V[vertexIndex].p + targetVertex);
}

void BoundingMesh::moveCageVertex(unsigned int vertexIndex, Vec3f targetVertex) {
    cage->V[vertexIndex].p = targetVertex;
    for(auto it = trianglesToChange.begin(); it != trianglesToChange.end(); ++it) {
        updateS(*it); //s is only updated when moving a vertex
    }
}

void BoundingMesh::updateS(unsigned int i) {
    Vec3f u = cage->V[cage->T[i].v[1]].p - cage->V[cage->T[i].v[0]].p;
    Vec3f v = cage->V[cage->T[i].v[2]].p - cage->V[cage->T[i].v[0]].p;
    Vec3f ui = cageInitial->V[cageInitial->T[i].v[1]].p - cageInitial->V[cageInitial->T[i].v[0]].p;
    Vec3f vi = cageInitial->V[cageInitial->T[i].v[2]].p - cageInitial->V[cageInitial->T[i].v[0]].p;
    float a = sqrt(2*cross(ui,vi).squaredLength());
    s[i] = a == 0 ? 1 : std::sqrt(u.squaredLength()*v.squaredLength()+ ui.squaredLength()*vi.squaredLength() -2*(dot(ui,vi)*dot(u,v)))/a;
}

void BoundingMesh::addVerticesToSelection(std::set<int> vertexIndices) {
    std::set<int> triangleToPrepare = std::set<int>();
    std::set<int> vertexToPrepare = std::set<int>();
    for(auto vertexIndex = vertexIndices.begin(); vertexIndex != vertexIndices.end(); ++vertexIndex) {
        auto pair = verticesToChange.insert(*vertexIndex);
        if (pair.second)
            vertexToPrepare.insert(*vertexIndex);
    }
    auto t_init = cageInitial->T.begin();
    int j = 0;
    for(auto t = cage->T.begin(); t != cage->T.end(); ++t) {
        bool ownVertex = false;
        for(unsigned int l = 0; l < 3; l++)
            ownVertex |= (vertexIndices.find(t->v[l]) != vertexIndices.end());
        if (ownVertex) {
            auto pair = trianglesToChange.insert(j);
            if (pair.second)
                triangleToPrepare.insert(j);
        }
        ++t_init;
        ++j;
    }
    prepareOldBounded(vertexIndices,triangleToPrepare,false);

}

void BoundingMesh::removeVerticesFromSelection(std::set<int> verticesToRestore) {
    for(auto vertexIndex = verticesToRestore.begin(); vertexIndex != verticesToRestore.end(); ++vertexIndex) {
        verticesToChange.erase(*vertexIndex);
    }
    std::set<int> trianglesToRestore = std::set<int>();
    for(auto it = trianglesToChange.begin(); it != trianglesToChange.end(); ++it) {
        bool ownVertex = false;
        for(unsigned int l = 0; l < 3; l++)
            ownVertex |= (verticesToChange.find(cage->T[*it].v[l]) != verticesToChange.end());
        if(!ownVertex) {
            trianglesToRestore.insert(*it);
            trianglesToChange.erase(it);
            it--;
        }
    }
    prepareOldBounded(verticesToRestore,trianglesToRestore,true);
}

void BoundingMesh::prepareOldBounded(std::set<int> vertexIndices, std::set<int> triangleIndices, bool restore) {
    float sign = restore ? - 1 : 1;
    oldBounded->V = bounded->V;
    #pragma omp for
    for(unsigned int v_old = 0 ; v_old < oldBounded->V.size(); ++v_old) {
        for(auto vertexIndex = vertexIndices.begin(); vertexIndex != vertexIndices.end(); ++vertexIndex) {
            oldBounded->V[v_old].p -= sign * vertexCoordinates[v_old][*vertexIndex] * sign * oldCage->V[*vertexIndex].p;
        }
        for(auto triangleIndex = triangleIndices.begin(); triangleIndex != triangleIndices.end(); ++triangleIndex) {
            oldBounded->V[v_old].p -= sign * normalCoordinates[v_old][*triangleIndex] * oldCage->T[*triangleIndex].computeNormal(*oldCage)*olds[*triangleIndex];
        }
    }
}

void BoundingMesh::addTrianglesToSelection(std::set<int> triangleIndices) {
    std::set<int> vertexIndices = std::set<int>();
    for(auto triangleIndex = triangleIndices.begin(); triangleIndex != triangleIndices.end(); ++triangleIndex) {
        selectedTriangles.insert(*triangleIndex);
        for(int l = 0; l < 3; l++) {
            vertexIndices.insert(cage->T[*triangleIndex].v[l]);
        }
    }
    addVerticesToSelection(vertexIndices);
}

void BoundingMesh::removeTrianglesFromSelection(std::set<int> triangleIndices) {
    std::set<int> verticesToRestore = std::set<int>();
    for(auto triangleIndex = triangleIndices.begin(); triangleIndex != triangleIndices.end(); ++triangleIndex) {
        selectedTriangles.erase(*triangleIndex);
        for(int l = 0; l < 3; l++) {
            bool inTriangle = false;
            for(auto selected_it = selectedTriangles.begin(); selected_it != selectedTriangles.end(); ++selected_it) {
                for(unsigned int l = 0; l < 3; l++)
                    inTriangle |= ((unsigned int) *triangleIndex == cage->T[*selected_it].v[l]);
                if(!inTriangle)
                    verticesToRestore.insert(cage->T[*triangleIndex].v[l]);
            } //Probably buggy
        }
    }
    removeVerticesFromSelection(verticesToRestore);

}

void BoundingMesh::release(bool validate) {
    if(!validate) {
        *cage = *oldCage;
        s = olds;
   }
    else {
        *oldCage = *cage; //refactor this
        olds = s;
    }
    makeChangeFull();
    prepareOldBounded(verticesToChange,trianglesToChange,false);
}

void BoundingMesh::makeChange() {
        bounded->V = oldBounded->V;
        #pragma omp for
        for(unsigned int v = 0; v < bounded->V.size(); ++v) {
            for(auto it = trianglesToChange.begin(); it != trianglesToChange.end(); ++it){
                bounded->V[v].p += normalCoordinates[v][*it] * cage->T[*it].computeNormal(*cage)*s[*it];// - (*coord_v)[*it] * oldCage->T[*it].computeNormal(*oldCage)*olds[*it];
            }
            for(auto it = verticesToChange.begin(); it != verticesToChange.end(); ++it){
                bounded->V[v].p += vertexCoordinates[v][*it] * cage->V[*it].p;// - (*coord_v)[*it] * oldCage->V[*it].p;
            }
        }
        bounded->recomputeNormals();
}

void BoundingMesh::makeChangeFull() {
    #pragma omp for
    for(unsigned int v = 0; v < bounded->V.size(); ++v) {
        bounded->V[v].p = Vec3f(0,0,0);
        for(unsigned int j = 0; j < cage->T.size(); j++) {
                bounded->V[v].p += normalCoordinates[v][j] * (cage->T[j].computeNormal(*cage)*s[j]);
        }
        for(unsigned int j = 0; j < cage->V.size(); j++) {
            bounded->V[v].p += vertexCoordinates[v][j] * (cage->V[j].p);
        }
    }
    bounded->recomputeNormals();
}

void BoundingMesh::reset() {
    *cage = *cageInitial;
    *oldCage = *cage;
    s = std::vector<float>(cage->T.size(),1.);
    olds = s;
    makeChangeFull();
    *oldBounded = *bounded;
    trianglesToChange = std::set<int>();
    verticesToChange =  std::set<int>();
    selectedTriangles = std::set<int>();
}

void BoundingMesh::clearSelection() {
    selectedTriangles = std::set<int>();
    trianglesToChange = std::set<int>();
    verticesToChange = std::set<int>();
    prepareOldBounded(verticesToChange,trianglesToChange,false);
}

std::set<int> BoundingMesh::getTriangleSelection() {
    return selectedTriangles;
}

void BoundingMesh::draw(Vec3i selectedColor) {
    GLint mode[2];
	glGetIntegerv( GL_POLYGON_MODE, mode );
    bounded->draw();
    glDisable (GL_LIGHTING);
	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    cage->draw();
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    for (auto it = selectedTriangles.begin(); it != selectedTriangles.end(); ++it){
        for(unsigned int j=0;j<3;j++){
            Vec3i selectedColor(255,255,0);
            //std::cerr << cage->T[*it].v[j] << " " << cage->V[cage->T[*it].v[j]].p <<std::endl;
            Vec3f center= cage->V[cage->T[*it].v[j]].p;
            glSphere(center[0],center[1],center[2],0.02,selectedColor);
        }
    }
    glPolygonMode( GL_FRONT_AND_BACK, mode[1] );
    glEnable (GL_LIGHTING);

}
