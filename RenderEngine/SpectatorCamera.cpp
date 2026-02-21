#include "SpectatorCamera.h"
#include "raymath.h"
#include <cmath>
#include <algorithm>

static constexpr float kPitchLimit = 1.5533430f; // ~89 degrees

SpectatorCamera::SpectatorCamera(Vector3 position, float yaw, float pitch)
    : m_position(position), m_yaw(yaw), m_pitch(pitch)
{
    DisableCursor();
}

void SpectatorCamera::Update(const std::vector<Input::InputAction>& actions, float dt)
{
    // --- Mouse look ---
    Vector2 delta = GetMouseDelta();
    m_yaw   += delta.x * mouseSensitivity;
    m_pitch -= delta.y * mouseSensitivity; // invert Y: screen-down = pitch down
    m_pitch  = std::clamp(m_pitch, -kPitchLimit, kPitchLimit);

    // --- Flat movement vectors (XZ plane only, independent of pitch) ---
    // yaw=0 -> looking -Z; yaw=pi/2 -> looking +X
    Vector3 flatForward = { sinf(m_yaw), 0.0f, -cosf(m_yaw) };
    Vector3 flatRight   = { cosf(m_yaw), 0.0f,  sinf(m_yaw) };

    Vector3 move = { 0.0f, 0.0f, 0.0f };
    for (const auto& a : actions) {
        if (a.type != Input::InputEvent::Held) continue;
        switch (a.action) {
            case Input::ActionID::CamForward: move = move + flatForward; break;
            case Input::ActionID::CamBack:    move = move - flatForward; break;
            case Input::ActionID::CamRight:   move = move + flatRight;   break;
            case Input::ActionID::CamLeft:    move = move - flatRight;   break;
            case Input::ActionID::CamUp:      move.y += 1.0f;            break;
            case Input::ActionID::CamDown:    move.y -= 1.0f;            break;
            default: break;
        }
    }

    // Normalize to prevent diagonal speed boost, then scale by speed and dt
    float len = sqrtf(move.x * move.x + move.y * move.y + move.z * move.z);
    if (len > 0.0f) {
        m_position = m_position + move * (moveSpeed * dt / len);
    }
}

Camera SpectatorCamera::GetCamera() const
{
    // Full 3D look direction (applies pitch on top of yaw)
    Vector3 dir = {
         cosf(m_pitch) * sinf(m_yaw),
         sinf(m_pitch),
        -cosf(m_pitch) * cosf(m_yaw)
    };

    Camera cam     = { 0 };
    cam.position   = m_position;
    cam.target     = m_position + dir;
    cam.up         = { 0.0f, 1.0f, 0.0f };
    cam.fovy       = 45.0f;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}
