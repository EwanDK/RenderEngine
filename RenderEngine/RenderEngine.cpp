#include "raylib.h"
#include "raymath.h"
#include "Collision/CollisionSystem.h"
#include "Input/InputSystem.h"

#include "EventBus.h"
#include "Renderer.h"
#include "Projects/SoftBody/Slime.h"


int main(int, char**)
{
    InitWindow(WINDOW_W, WINDOW_H, NAME);
    DisableCursor();

    Renderer renderer(EventBus::Get());
    Slime slime;
    
    Vector3 cubePos = {0.0f, -50.0f, 0.0f};
    Vector3 cubeSize = {80.0f, 20.0f, 80.0f};
    Mesh cubeMesh = GenMeshCube(cubeSize.x, cubeSize.y, cubeSize.z);
    UploadMesh(&cubeMesh, false);
    Model cubeModel = LoadModelFromMesh(cubeMesh);

    // Extract cube verts in world space and register as static collider
    std::vector<Vector3> cubeVerts;
    Matrix cubeTransform = MatrixTranslate(cubePos.x, cubePos.y, cubePos.z);
    Collision::ConvexShape cubeShape = Collision::ConvexShapeFromMesh(cubeMesh, cubeTransform, cubeVerts);
    slime.core.AddStaticCollider(cubeShape);

#if FPSCAP
    SetTargetFPS(60);
#endif

    bool showClusters  = false;
    bool showParticles = false;
    bool showContacts  = false;
    bool showSprings   = false;

    Input::InputSystem inputSystem;

    while (!WindowShouldClose())
    {
        auto actions = inputSystem.Poll();
        float dt = GetFrameTime();

        for (const auto& a : actions) {
            if (a.type != Input::InputEvent::Pressed) continue;
            switch (a.action) {
            case Input::ActionID::ToggleClusters:  showClusters  = !showClusters;  break;
            case Input::ActionID::ToggleParticles: showParticles = !showParticles; break;
            case Input::ActionID::ToggleContacts:  showContacts  = !showContacts;  break;
            case Input::ActionID::ToggleSprings:   showSprings   = !showSprings;   break;
            default: break;
            }
        }

        slime.SetDebug(showClusters, showParticles, showContacts, showSprings);
        slime.Update(dt);
        EventBus::Get().Flush((float)GetTime());
        renderer.Draw(slime.GetThirdPersonCamera().GetCamera());
    }

    CloseWindow();
    return 0;
}