#include "raylib.h"
#include "raymath.h"
#include "Collision/CollisionSystem.h"
#include "Input/InputSystem.h"
#include "SpectatorCamera.h"
#include <vector>
#include <cmath>
#include "EventBus.h"
#include "Renderer.h"


int main(int, char**)
{
    InitWindow(WINDOW_W, WINDOW_H, NAME);

    SpectatorCamera camera({ 0.0f, 150.0f, 0.0f });
    Renderer renderer(EventBus::Get());
    camera.InitFlashlight(renderer.getLitShader()); //tmp

    
#if FPSCAP
    SetTargetFPS(60);
#endif
    

    Input::InputSystem inputSystem;

    while (!WindowShouldClose())
    {
        // Poll input system and process actions
        auto actions = inputSystem.Poll();
        float dt = GetFrameTime();

        camera.Update(actions, dt);

        for (const auto& a : actions) {
            if (a.type != Input::InputEvent::Pressed) continue;
            switch (a.action) {
                
                default: break;
            }
        }

        EventBus::Get().Flush(GetTime());
        renderer.Draw(camera.GetCamera());
    }

    CloseWindow();
    return 0;
}
