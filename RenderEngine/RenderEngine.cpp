#include "raylib.h"
#include "SoftBody.h"

int main(int, char**)
{
    InitWindow(800,400,"test");
    Camera camera = { 0 };
    camera.position = Vector3{ 0.0f, 500.0f, 500.0f };
    camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    
    SoftBody soft = SoftBody();
    
    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);
        //DrawModelWires(model,Vector3(0.f,0.f,0.f),1,BLUE);
        soft.Update(GetFrameTime());
        soft.Draw();
        DrawGrid(20, 10.0f);
        EndMode3D();
        EndDrawing();
        soft.Solve(GetFrameTime());
    }
    CloseWindow();
    return 0;
}
