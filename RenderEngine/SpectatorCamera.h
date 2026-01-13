#pragma once
#include "raylib.h"

class SpectatorCamera{
public:

private:
    Vector3 position;
    float scale;
    Quaternion rotation;
    Matrix worldTransform;
    bool updateWT;
};
