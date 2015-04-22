#ifndef BOUNDINGMESH_H
#define BOUNDINGMESH_H

#include "Mesh.h"
#include <map>
#include <vector>
#include <set>


class BoundingMesh
{
    public:
        BoundingMesh();
        BoundingMesh(Mesh * m_bounded, Mesh * m_cage);
        virtual ~BoundingMesh();
        static BoundingMesh * generate(const char * modelFilename, const char * modelCage);

        void computeCoordinates();
        void updateCage();
        void draw();
        void reset();
        void save(const std::string & filename);
        void moveCageVertex(unsigned int vertexIndex, Vec3f targetVertex);
        void moveCageVertexIncr(unsigned int vertexIndex, Vec3f targetVertex);
        void moveCageTriangleIncr(unsigned int triangleIndex, Vec3f targetVertex);
        void makeChange();
        void makeChangeFull();
        void release(bool validate);

        void addVerticesToSelection(std::set<int> vertexIndices);
        void removeVerticesFromSelection(std::set<int> vertexIndices);
        void addTrianglesToSelection(std::set<int> triangleIndices);
        void removeTrianglesFromSelection(std::set<int> triangleIndices);

        void prepareVertexCoordinatesOldBounded(unsigned int vertexIndex);
        void prepareTriangleCoordinatesOldBounded(unsigned int j, float s);


	Mesh *cage;
    private:
        float GCTriInt(Vec3f p, Vec3f v1, Vec3f v2, Vec3f eta);

        Mesh * bounded;
        Mesh * oldBounded;
        Mesh * cageInitial;
        Mesh * oldCage;
        std::vector<float> s;
        std::vector<float> olds;
        std::map<Triangle,Vec3f> normalMap;
        std::vector<std::vector<float>> vertexCoordinates; //ordered like vertex in bounded->V
        std::vector<std::vector<float>> normalCoordinates; //ordered like triangles in bounded->T
        std::set<int> trianglesToChange;
        std::set<int> verticesToChange;
        bool update;
};

#endif // BOUNDINGMESH_H
