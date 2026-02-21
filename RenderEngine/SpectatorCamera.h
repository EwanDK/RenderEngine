#pragma once
#include "raylib.h"
#include "Input/InputSystem.h"
#include <vector>

class SpectatorCamera {
public:
    float moveSpeed        = 80.0f;
    float mouseSensitivity = 0.002f;

    SpectatorCamera(Vector3 position, float yaw = 0.0f, float pitch = 0.0f);

    // Pass this frame's action list and delta time.
    void Update(const std::vector<Input::InputAction>& actions, float dt);

    // Returns a raylib Camera ready to pass to BeginMode3D.
    Camera GetCamera() const;

private:
    Vector3 m_position;
    float   m_yaw;   // radians, rotation around world Y axis
    float   m_pitch; // radians, clamped to ~+-89 degrees
};
