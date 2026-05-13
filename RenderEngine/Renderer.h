#pragma once
#include <vector>
#include <functional>
#include "raylib.h"
#include "EventBus.h"

#define FPSCAP false
#define NAME "Base Engine Test"
#define WINDOW_W 1280
#define WINDOW_H 720

class Renderer {
public:
    explicit Renderer(EventBus& bus);
    ~Renderer();
    void Draw(Camera3D camera);
    Shader getLitShader(){return litShader;} //TODO input handling in Event queue and Camera in Renderer

private:
    std::vector<class Drawable*> flatDrawables;
    std::vector<class Drawable*> litDrawables;
    Shader litShader;
    EventBus& bus;
    EventBus::SubscriptionID flatSubID;
    EventBus::SubscriptionID litSubID;
};