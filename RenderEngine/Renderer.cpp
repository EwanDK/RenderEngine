#include "Renderer.h"
#include "Drawable.h"
#include "DrawableEvents.h"

Renderer::Renderer(EventBus& bus) : bus(bus) {
    litShader = LoadShader(TextFormat("Res/lighting.vert", 330), TextFormat("Res/lighting.frag", 330));
    litShader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(litShader, "viewPos");
    int ambientLoc = GetShaderLocation(litShader, "ambient");
    float ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
    SetShaderValue(litShader, ambientLoc, ambient, SHADER_UNIFORM_VEC4);

    flatSubID = bus.Subscribe<AddToFlatEvent>([this](const AddToFlatEvent& e) {
        flatDrawables.push_back(e.drawable);
    });
    litSubID = bus.Subscribe<AddToLitEvent>([this](const AddToLitEvent& e) {
        litDrawables.push_back(e.drawable);
    });
}

Renderer::~Renderer() {
    bus.Unsubscribe(flatSubID);
    bus.Unsubscribe(litSubID);
    UnloadShader(litShader);
}

void Renderer::Draw(Camera3D camera) {
    SetShaderValue(litShader, litShader.locs[SHADER_LOC_VECTOR_VIEW],
        &camera.position, SHADER_UNIFORM_VEC3);

    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(camera);

    for (auto* d : flatDrawables)
        d->Draw();

    BeginShaderMode(litShader);
    for (auto* d : litDrawables)
        d->Draw();
    EndShaderMode();
    
    
    
    DrawGrid(20, 50);
    DrawCube(Vector3{0,-50,0},80.0f, 20.0f, 80.0f,RED);
    DrawPlane(Vector3{0.0f, -60.0f, 0.0f}, Vector2{400.0f, 400.0f}, LIGHTGRAY);

    EndMode3D();

    DrawText("Soft Body + GJK Collision", 10, 10, 20, DARKGRAY);
    DrawFPS(10, 40);
    for (auto* d : flatDrawables)
        d->DrawHUD();

    EndDrawing();
}