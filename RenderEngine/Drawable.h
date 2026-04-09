#pragma once

class EventBus;

class Drawable {
public:
    virtual ~Drawable() = default;
    virtual void Draw() = 0;

    void AddToFlat(EventBus& bus);
    void AddToLit(EventBus& bus);
};