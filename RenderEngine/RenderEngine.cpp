#include "raylib.h"
#include "raymath.h"
#include "SoftBody.h"
#include "Collision/CollisionSystem.h"
#include "Collision/GJK.h"
#include <vector>

int main(int, char**)
{
    InitWindow(1280,720,"Soft Body Simulation");

    Camera camera = { 0 };
    camera.position = Vector3{ 0.0f, 150.0f, 300.0f };
    camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SoftBody soft = SoftBody();
    soft.Translate({0.0f, 100.0f, 0.0f}); // Start above the cube so it falls onto it

    // Ground plane well below the cube
    Collision::GroundPlane ground({0, 1, 0}, -200.0f);
    soft.SetGroundPlane(ground);

    // Static cube obstacle — top face at y=10, bottom at y=-10
    Vector3 cubePos = {0.0f, 0.0f, 0.0f};
    Vector3 cubeSize = {80.0f, 20.0f, 80.0f};
    Mesh cubeMesh = GenMeshCube(cubeSize.x, cubeSize.y, cubeSize.z);
    UploadMesh(&cubeMesh, false);
    Model cubeModel = LoadModelFromMesh(cubeMesh);

    // Extract cube verts in world space and register as static collider
    std::vector<Vector3> cubeVerts;
    Matrix cubeTransform = MatrixTranslate(cubePos.x, cubePos.y, cubePos.z);
    Collision::ConvexShape cubeShape = Collision::ConvexShapeFromMesh(cubeMesh, cubeTransform, cubeVerts);
    soft.AddStaticCollider(cubeShape);

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        soft.Update(GetFrameTime());

        // Draw static cube obstacle
        DrawModel(cubeModel, cubePos, 1.0f, GRAY);
        DrawModelWires(cubeModel, cubePos, 1.0f, DARKGRAY);

        // Draw ground plane visual
        DrawPlane(Vector3{0.0f, -60.0f, 0.0f}, Vector2{400.0f, 400.0f}, LIGHTGRAY);
        DrawGrid(20, 10.0f);

        EndMode3D();

        // HUD
        DrawText("Soft Body + GJK Collision", 10, 10, 20, DARKGRAY);
        DrawFPS(10, 40);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
