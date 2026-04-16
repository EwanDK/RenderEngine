#pragma once
#include <raylib.h>
#include <vector>
#include "Drawable.h"

class CaveGen : public Drawable{
public:
    void GenerateMesh(Shader litShader);
    void Draw();
    void WaterDeposition();
    void SortHeight(bool decreasing, Model inModel, Mesh inMesh);
    void WaterDrip();
    void BuildMeshSection(Mesh& outMesh, Model& outModel, std::vector<std::vector<int>>& outAdjacency, Vector3 normal, float sizeX, float sizeY, int subdivX, int subdivY, float noiseScale, float maxDisplacement, Shader litShader);
    std::vector<int> FindLocalMinima(const Mesh& mesh, const std::vector<std::vector<int>>& adjacency);
    Mesh ceiling;
    Mesh floor;
    Model ceilingModel;
    Model floorModel;
    std::vector<std::vector<int>> ceilingAdjacency; // adjacency[i] = neighbor vertex indices of i
    std::vector<std::vector<int>> floorAdjacency;
    std::vector<int> orderedVertices; // vertex indices sorted by decreasing Y (highest first)
    std::vector<float> water;
};
