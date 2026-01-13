#pragma once
#include <vector>

#include "raylib.h"
#include "raymath.h"
#include "MathUtils.h"

struct Spring;

class SoftBody
{
public:
    SoftBody();
    Model model;
private:
    std::vector<Spring> springs;
    float* restPositions;
};
