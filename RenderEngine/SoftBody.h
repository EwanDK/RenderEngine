#pragma once
#include <vector>

#include "raylib.h"

struct Point{
    Vector3 position;
    Vector3 speed;
    Vector3 force;
    float mass;
    const float radius = 1.f;
    int index;
};

struct Spring{
    Point* points[2];
    float baseDistance;
    float stiffness;
    float damping;
    int virtualDistance=0;
};


class SoftBody
{
public:
    SoftBody();
    void SolveSpring(Spring& spring);
    void ClampSpringForce(Spring& spring);
    void ApplyShapeMatching(float stiffness);
    void Solve(float dt);
    void Draw();
    void RecomputeNormals(Mesh& mesh);
    void Update(float dt);
    Model model;
private:
    std::vector<Spring> springs;
    std::vector<Point> points;
    float* restPositions;
};
