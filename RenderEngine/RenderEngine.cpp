#include "raylib.h"
#include "SoftBody.h"
#include "Collision/CollisionSystem.h"

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

    // Set up ground plane at y = -60 (soft body starts centered at origin with radius ~50)
    Collision::GroundPlane ground({0, 1, 0}, -60.0f);
    soft.SetGroundPlane(ground);

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        soft.Update(GetFrameTime());

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
