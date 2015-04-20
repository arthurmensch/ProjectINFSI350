#ifndef BOUNDINGMESH_H
#define BOUNDINGMESH_H

#include "Mesh.h"
#include <map>
#include <vector>


class BoundingMesh
{
    public:
        BoundingMesh();
        BoundingMesh(Mesh * m_bounded, Mesh * m_cage);
        virtual ~BoundingMesh();
        void computeCoordinates();
        void updateBoundedMesh();
        void updateCage();
        float GCTriInt(Vec3f p, Vec3f v1, Vec3f v2, Vec3f eta);
        void draw();
        static BoundingMesh * generate();
	Mesh *cage;
    protected:
    private:
        Mesh * bounded;
        std::map<Triangle,Vec3f> normalMap;
        std::vector<std::vector<float>> vertexCoordinates; //ordered like vertex in bounded->V
        std::vector<std::vector<float>> normalCoordinates; //ordered like triangles in bounded->T
};

#endif // BOUNDINGMESH_H
