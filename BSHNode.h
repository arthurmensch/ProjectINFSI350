#include "Mesh.h"
#include "Camera.h"

class BSHNode
{
public:
    BSHNode();
    BSHNode(std::vector<Vertex> &V, std::vector<Triangle> &T, int depth);
    ~BSHNode();
    void drawGL(Vec3f camPos);
    void renderRecursive(Vec3f camPos);
private:
    Vec3f position;
    Vec3f normal;
    Vec3f color;
    float radius;
    int Tsize;
    int depth;
    bool isLeaf;
    std::vector<Triangle> triangles;
    std::vector<Vertex>* vertices;
    BSHNode *leftChild;
    BSHNode *rightChild;
};
