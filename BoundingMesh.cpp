#include "BoundingMesh.h"

BoundingMesh::BoundingMesh()
{
    //ctor
}

BoundingMesh::~BoundingMesh()
{
    //dtor
}


BoundingMesh::BoundingMesh(Mesh & m_bounded, Mesh & m_cage) {
    bounded = & m_bounded;
    cage = & m_cage;
}

BoundingMesh::updateCage() {

}

BoundingMesh::computeCoordinates() {
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
        for(Triangle t : cage->T) {
            Vec3f v[3];
            for(int l= 0; l < 3; l++) {
                v[l] = cage->V[t.v[l]]-eta;
            }
            Vec3f n = t.computeNormal();
            Vec3f p = dot(v[0],n) * n;
            float s[3];
            float I[3];
            float II[3];
            float q[3];
            float N[3];
            for(int l= 0; l < 3; l++) {
                s[l] = sign(dot(cross(v[l]-p,v[(l+1) %3]-p),n));
                I[l] = GCTriInt(p,v[j],v[(j+1) % 3],0);
                II[l] = GCTriInt(0,v[(j+1) % 3],v[j],0);
                q[l] = cross(v[(j+1) % 3],v[j]);
                N[l] = q[l].normalize();
            }
            I = - s[0]*I[0]-s[1]*I[1]-s[2]*I[2];
            normalCoordinates[i][]
        }
    }
}
