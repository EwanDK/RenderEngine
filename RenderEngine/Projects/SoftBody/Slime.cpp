#include "Slime.h"
#include "raylib.h"
#include "raymath.h"
#include "EventBus.h"

// UV sphere (rings=10, slices=10): vertexCount = 2 + 9*10 = 92
// index 0 = top pole, indices 1-10 = ring 1, indices 81-90 = ring 9, index 91 = bottom pole
static constexpr int kTopPole = 0;
static constexpr int kBottomPole = 91;
static constexpr int kRing1Start = 1;
static constexpr int kRing2Start = 11;
static constexpr int kRing8Start = 71;;
static constexpr int kRing9Start = 81;
static constexpr int kSlices = 10;

Slime::Slime() : core(SoftBody::ConstraintMode::ShapeMatching), camera(&centroid, 50.f, {0.f, 0.f, 0.f}) {
    core.SetGroundPlane({{0.f, 1.f, 0.f}, -50.f});
    core.SetLockUpright(true);
    core.AddMuscle(kTopPole, kBottomPole, 50.f, 1.f, 1000.f, 0);
    for (int i = 0; i < kSlices; i++) {
        core.AddMuscle(kRing1Start + i, kRing9Start + i, 50.f, 0.5f, 1000.f, 0);
        core.AddMuscle(kRing2Start + i, kRing8Start + i, 50.f, 0.5f, 1000.f, 0);
    }
    AddToFlat(EventBus::Get());
}

void Slime::Update(float dt) {
    camera.Update(dt);
    Vector3 forward = camera.GetForwardXZ();
    Vector3 right   = camera.GetRightXZ();

    Vector3 moveForce = {};
    const float forceMag = 15.f;
    if (IsKeyDown(KEY_W) || IsKeyDown(KEY_Z)) moveForce += forward * forceMag;
    if (IsKeyDown(KEY_S)) moveForce -= forward * forceMag;
    if (IsKeyDown(KEY_D)) moveForce += right * forceMag;
    if (IsKeyDown(KEY_A) || IsKeyDown(KEY_Q)) moveForce -= right * forceMag;

    bool spaceHeld = IsKeyDown(KEY_SPACE);

    auto& pts = core.GetPoints();
    auto measureHeight = [&]() {
        float minY = pts[0].position.y, maxY = pts[0].position.y;
        for (const auto& p : pts) {
            if (p.position.y < minY) minY = p.position.y;
            if (p.position.y > maxY) maxY = p.position.y;
        }
        return maxY - minY;
    };

    if (!wasSpaceHeld && spaceHeld)
        squatHeight = measureHeight();

    if (wasSpaceHeld && !spaceHeld) {
        jumpTimer = 1.0f;
        float compression = fmaxf(0.f, squatHeight - measureHeight());
        float jumpSpeed = compression * 5.f;
        for (auto& p : pts) p.speed.y += jumpSpeed;
    }
    wasSpaceHeld = spaceHeld;

    Vector3 jumpForce = {};
    if (jumpTimer > 0.f) {
        jumpTimer -= dt;
        jumpForce = {0.f, 9.81f * 3.f, 0.f}; // 3x gravity upward = 2x net lift
    }

    core.SetMuscleGroup(0, spaceHeld);
    core.Solve(dt, moveForce/* + jumpForce*/);

    centroid = {};
    for (const auto& p : pts) centroid += p.position;
    centroid *= 1.f / (float)pts.size();
}

void Slime::Draw() {
    core.Draw();
}

void Slime::DrawHUD() {
    core.DrawHUD();
}

void Slime::SetDebug(bool showClusters, bool showParticles, bool showContacts, bool showSprings) {
    core.SetDebug(showClusters, showParticles, showContacts, showSprings);
}