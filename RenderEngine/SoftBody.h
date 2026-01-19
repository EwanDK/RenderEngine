#pragma once
#include <vector>

#include "raylib.h"
#include "raymath.h"
#include "MathUtils.h"

struct Spring;
struct Point;

class SoftBody
{
public:
    SoftBody();
    void SolveSpring(Spring spring);
    void ClampSpringForce(Spring& spring);
    void ApplyShapeMatching(float stiffness);
    void Solve(float dt);
    Model model;
private:
    std::vector<Spring> springs;
    std::vector<Point> points;
    float* restPositions;
};
