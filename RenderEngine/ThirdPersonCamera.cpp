#include "ThirdPersonCamera.h"
#include "raymath.h"
#include <cmath>
#include <algorithm>

ThirdPersonCamera::ThirdPersonCamera(const Vector3* owner,
                                     float armLen,
                                     Vector3 off,
                                     float yaw,
                                     float pitch)
    : m_owner(owner), m_yaw(yaw), m_pitch(pitch)
{
    armLength = armLen;
    offset    = off;
}

void ThirdPersonCamera::Update(float /*dt*/)
{
    Vector2 delta = GetMouseDelta();
    m_yaw   += delta.x * mouseSensitivity;
    m_pitch -= delta.y * mouseSensitivity;   // invert Y: drag-down = look up
    m_pitch  = std::clamp(m_pitch, kPitchMin, kPitchMax);
}

Camera ThirdPersonCamera::GetCamera() const
{
    // Direction the camera "looks" (from camera toward anchor).
    // yaw=0  -> camera behind on +Z axis  (looking toward -Z)
    // yaw=pi -> camera in front on -Z axis
    Vector3 lookDir = {
         cosf(m_pitch) * sinf(m_yaw),
         sinf(m_pitch),
        -cosf(m_pitch) * cosf(m_yaw)
    };

    // Anchor point: owner position + local offset
    Vector3 anchor = *m_owner + offset;

    // Camera sits behind the anchor along -lookDir
    Vector3 camPos = anchor - lookDir * armLength;

    Camera cam     = { 0 };
    cam.position   = camPos;
    cam.target     = anchor;
    cam.up         = { 0.0f, 1.0f, 0.0f };
    cam.fovy       = fovy;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}