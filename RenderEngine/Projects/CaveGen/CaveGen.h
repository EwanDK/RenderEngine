#pragma once
#include <raylib.h>
#include <vector>
#include "Drawable.h"

class CaveGen : public Drawable
{
public:
    void GenerateMesh(Shader litShader);
    void Draw();
    void WaterDeposition();
    void SortHeight();
    Mesh ceiling;
    Model model;
    std::vector<std::vector<int>> adjacency; // adjacency[i] = neighbor vertex indices of i
    std::vector<int> orderedVertices;        // vertex indices sorted by decreasing Y (highest first)
};
