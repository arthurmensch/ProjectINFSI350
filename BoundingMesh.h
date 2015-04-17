#ifndef BOUNDINGMESH_H
#define BOUNDINGMESH_H

#include "Mesh.h"


class BoundingMesh
{
    public:
        BoundingMesh(Mesh & m_bounded, Mesh & m_cage);
        virtual ~BoundingMesh();
        void computeCoordinates();
        void updateBoundedMesh();
        void updateCage();
    protected:
    private:
        Mesh * bounded;
        Mesh * cage;
        std::map<Triangle,Vec3f> normalMap;
        std::vector<std::vector<float>> vertexCoordinates; //ordered like vertex in bounded->V
        std::vector<std::vector<float>> normalCoordinates; //ordered like triangles in bounded->T
};

#endif // BOUNDINGMESH_H
