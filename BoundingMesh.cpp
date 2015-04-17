#include "BoundingMesh.h"
#include <cmath>
BoundingMesh::~BoundingMesh()
{
    //dtor
}


BoundingMesh::BoundingMesh(Mesh & m_bounded, Mesh & m_cage) {
    bounded = & m_bounded;
    cage = & m_cage;
}

void BoundingMesh::updateCage() {

}

float sign(float x) {
        return x < 0 ? - 1 : 0;
}

void BoundingMesh::computeCoordinates() {
    float eps = 1e-6;
    int Vsize_b = bounded->V.size();
    int Vsize_c = cage->V.size();
    int Tsize_c = cage->T.size();
    vertexCoordinates = std::vector<std::vector<float>>(Vsize_b);
    normalCoordinates = std::vector<std::vector<float>>(Vsize_b);
    for(auto vec : vertexCoordinates)
        vec = std::vector<float>(Vsize_c,0);
    for(auto vec : normalCoordinates)
        vec = std::vector<float>(Tsize_c,0);
    int i = 0;
    for(Vertex eta : bounded->V) {
        int j = 0;
        for(Triangle t : cage->T) {
            Vec3f v[3];
            for(int l= 0; l < 3; l++) {
                v[l] = cage->V[t.v[l]].p-eta.p;
            }
            Vec3f n = t.computeNormal(*cage);
            Vec3f p = dot(v[0],n) * n;
            float s[3];
            float I[3];
            float II[3];
            Vec3f N[3];
            for(int l= 0; l < 3; l++) {
                s[l] = sign(dot(cross(v[l]-p,v[(l+1) %3]-p),n));
                I[l] = GCTriInt(p,v[j],v[(j+1) % 3],eta.p);
                II[l] = GCTriInt(Vec3f(0,0,0),v[(j+1) % 3],v[j],eta.p);
                N[l] = cross(v[(j+1) % 3],v[j]);
                N[l].normalize();
            }
            float Is= - s[0]*I[0]-s[1]*I[1]-s[2]*I[2];
            normalCoordinates[i][j] = - Is;
            Vec3f w = n*I + N[0] * II[0] + N[1] * II[1] + N[2] * II[2];
            if (w.squaredLength() > eps) {
                for(int l= 0; l < 3; l++) {
                    normalCoordinates[i][t.v[l]] += dot(N[(l+1) % 3],w)/dot(N[(l+1) % 3],v[l]);
                }
            }
        }
    }
}

float BoundingMesh::GCTriInt(Vec3f p, Vec3f v1, Vec3f v2, Vec3f eta) {
    float alpha = acos(dot((v2-v1),(p-v1))); //TODO normalize
    float beta = acos(dot((v1-p),(v2-p))); //TODO Normalize
    float lambda = (p-v1).squaredLength()*sin(alpha)*sin(alpha);
    float c = (p-eta).squaredLength();
    float theta[2] = {M_PI-alpha,M_PI-alpha-beta};
    float I[2];
    for(int i = 0; i < 2; i++) {
        float S = sin(theta[i]);
        float C = cos(theta[i]);
        I[i] = - sign(S)/2 * (2*sqrt(c)*atan(sqrt(c)*C / (sqrt(lambda + S*S*c)))) +
        sqrt(lambda) * log(2*sqrt(lambda)*S*S/((1-C)*(1-C))*(1-2*c*S/(c*(1+C)+lambda + sqrt(lambda*lambda+lambda*c*S*S))));
    }
    return - 1 / (4*M_PI)*abs(I[0]-I[1]-sqrt(c)*beta);
}

