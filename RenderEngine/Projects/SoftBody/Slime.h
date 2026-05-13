#pragma once
#include "SoftBody.h"
#include "ThirdPersonCamera.h"
#include "Drawable.h"

class Slime : public Drawable {
public:
    Slime();
    void Update(float dt);
    void Draw() override;
    void DrawHUD() override;
    const ThirdPersonCamera& GetThirdPersonCamera() const { return camera; }
    void SetDebug(bool showClusters, bool showParticles, bool showContacts, bool showSprings);
    SoftBody core;
private:
    
    Vector3 centroid = {};
    ThirdPersonCamera camera;
    float jumpTimer = 0.f;
    bool wasSpaceHeld = false;
    float squatHeight = 0.f;
};