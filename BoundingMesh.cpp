#include "BoundingMesh.h"
#include <cmath>

BoundingMesh::BoundingMesh() {
}

BoundingMesh * BoundingMesh::generate() {
    Mesh * bounded = new Mesh();
    bounded->loadOFF("models/horse.off");
    Mesh * cage = new Mesh();
    cage->loadOFF("models/square.off");
    BoundingMesh * boundingMesh = new BoundingMesh(bounded, cage);
    boundingMesh->computeCoordinates();
    boundingMesh->updateBoundedMesh();
    return boundingMesh;
}

BoundingMesh::~BoundingMesh()
{
    delete bounded;
    delete cage;
}


BoundingMesh::BoundingMesh(Mesh * m_bounded, Mesh * m_cage) {
    bounded = m_bounded;
    cage = m_cage;
}

void BoundingMesh::updateCage() {

}

float sign(float x) {
        return x < 0 ? - 1 : 1;
}

void BoundingMesh::computeCoordinates() {
    float eps = 1e-10;
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
            for(int l= 0; l < 3; l++) {
                v[l] = cage->V[t.v[l]].p-eta.p;
            }
            Vec3f n = t.computeNormal(*cage);
            //std::cerr << "n " << n << std::endl;
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
            //std::cerr << "I " << I[0] << " " << I[1] << " "  << I[2] << std::endl;
            //std::cerr << "II " << II[0] << " " << II[1] << " "  << II[2] << std::endl;

            float Is= - fabs(s[0]*I[0]-s[1]*I[1]-s[2]*I[2]);
          //  std::cerr << "Normal" << Is << std::endl;
            normalCoordinates[i][j] = - Is;
            Vec3f w = n*Is + N[0] * II[0] + N[1] * II[1] + N[2] * II[2];
            if (w.squaredLength() > eps) {
                for(int l= 0; l < 3; l++) {
                    //std::cerr << "Vertex" << dot(N[(l+1) % 3],w)/dot(N[(l+1) % 3],v[l]) << std::endl;
                    vertexCoordinates[i][t.v[l]] += dot(N[(l+1) % 3],w)/dot(N[(l+1) % 3],v[l]);
                }
            }
        }
    }
}

float BoundingMesh::GCTriInt(Vec3f p, Vec3f v1, Vec3f v2, Vec3f eta) {
    //std::cerr << " v1 " << v1 << " v2 " << v2 << std::endl;
    Vec3f v2mv1 = v2 - v1;
    Vec3f pmv1 = p - v1;
    Vec3f v1mp = v1 - p;
    Vec3f v2mp = v2 - p;
    v2mv1.normalize();
    pmv1.normalize();
    v1mp.normalize();
    v2mp.normalize();

    float cosalpha = dot(v2mv1,pmv1);
    if (cosalpha > 1)
        cosalpha = 1;
    if(cosalpha < -1)
        cosalpha = -1;
    float alpha = acos(cosalpha);
    float cosbeta = dot(v1mp,v2mp);
    if (cosbeta > 1)
        cosbeta = 1;
    if(cosbeta < -1)
        cosbeta = -1;

    float beta = acos(cosbeta);
    float lambda = (p-v1).squaredLength()*sin(alpha)*sin(alpha);
    float c = (p-eta).squaredLength();
    float theta[2] = {(float)M_PI-alpha,(float)M_PI-alpha-beta};
    float I[2];
    for(int i = 0; i < 2; i++) {
        float S = sin(theta[i]);
        float C = cos(theta[i]);
        I[i] = 0;
        float logt = sqrt(lambda) * log(2*sqrt(lambda)*S*S/((1-C)*(1-C))*(1-2*c*C/(c*(1+C)+lambda + sqrt(lambda*lambda+lambda*c*S*S))));
        float atant = 2*sqrt(c)*atan(sqrt(c)*C / (sqrt(lambda + S*S*c)));
        if (isnan(logt))
            logt = 0;
        if (isnan(atant))
            atant = 2 * sqrt(c) * M_PI / 2 * sign(C);
        I[i] =  (logt + atant)* - sign(S) / 2;
        if (isnan(I[i]))
            std::cerr << "Nan error";
    }
    float res = - 1 / (4*M_PI)* fabs (I[0]-I[1]-sqrt(c)*beta);

    return res;
}

void BoundingMesh::updateBoundedMesh() {
    for(unsigned int i = 0; i < bounded->V.size(); i++) {
        bounded->V[i].p = Vec3f(0,0,0);
        for(unsigned int j = 0; j < cage->T.size(); j++) {
            bounded->V[i].p += normalCoordinates[i][j] * cage->T[j].computeNormal(*cage);
        }
        for(unsigned int j = 0; j < cage->V.size(); j++) {
            bounded->V[i].p += vertexCoordinates[i][j] * cage->V[j].p;
        }
    }
    bounded->recomputeNormals();
   // bounded->centerAndScaleToUnit();
   for (Vertex v : bounded->V)
    std::cerr << v.p << std::endl;
}

void BoundingMesh::draw() {
    cage->draw();
      // std::cerr << v.p << std::endl;
    bounded->draw();
}
