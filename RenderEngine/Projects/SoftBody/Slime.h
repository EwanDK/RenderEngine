#pragma once
#include "SoftBody.h"
#include "ThirdPersonCamera.h"

namespace Input
{
    struct InputAction;
}

class Slime{

public:
    Slime();
    void Update(const std::vector<Input::InputAction>& actions,float dt,Vector3 force);
    void ApplyForce(Vector3 force);

private:
    SoftBody core,skin;
    ThirdPersonCamera camera;

};
