#include "Renderer.h"
#include "Drawable.h"
#include "DrawableEvents.h"

Renderer::Renderer(EventBus& bus) : bus(bus){
    litShader = LoadShader(TextFormat("Res/lighting.vert", 330), TextFormat("Res/lighting.frag", 330));
    litShader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(litShader, "viewPos");
    int ambientLoc = GetShaderLocation(litShader, "ambient");
    float ambient[4] = {0.1f, 0.1f, 0.1f, 1.0f};
    SetShaderValue(litShader, ambientLoc, ambient, SHADER_UNIFORM_VEC4);

    flatSubID = bus.Subscribe<AddToFlatEvent>([this](const AddToFlatEvent& e){
        flatDrawables.push_back(e.drawable);
    });
    litSubID = bus.Subscribe<AddToLitEvent>([this](const AddToLitEvent& e){
        litDrawables.push_back(e.drawable);
    });
}

Renderer::~Renderer(){
    bus.Unsubscribe(flatSubID);
    bus.Unsubscribe(litSubID);
    UnloadShader(litShader);
}

void Renderer::Draw(Camera3D camera){
    SetShaderValue(litShader, litShader.locs[SHADER_LOC_VECTOR_VIEW], &camera.position, SHADER_UNIFORM_VEC3);

    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(camera);

    for (auto* d : flatDrawables)
        d->Draw();

    BeginShaderMode(litShader);
    for (auto* d : litDrawables)
        d->Draw();
    EndShaderMode();

    EndMode3D();
    DrawText("[F5] Next Water Deposition step", 10, 70, 15,BLUE);
    EndDrawing();
}
