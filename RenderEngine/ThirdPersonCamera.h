#pragma once
#include "raylib.h"

// A third-person orbit camera attached to an owner's world position.
//
// The "swivel arm" model:
//   anchor    = *owner + offset          (attachment point on the actor)
//   cameraPos = anchor - lookDir * armLength   (behind the anchor)
//   cam looks toward anchor
//
// Yaw  pivots around the world-Y axis at the anchor.
// Pitch tilts the arm up/down.

class ThirdPersonCamera {
public:
    float armLength        = 10.0f;
    float mouseSensitivity = 0.003f;
    float fovy             = 45.0f;

    // offset in world space added to *owner to get the arm anchor
    // (e.g. {0, 1.5f, 0} to attach at head height instead of feet)
    Vector3 offset = { 0.0f, 0.0f, 0.0f };

    // owner must outlive this camera
    ThirdPersonCamera(const Vector3* owner,
                      float armLength  = 10.0f,
                      Vector3 offset   = { 0.0f, 0.0f, 0.0f },
                      float yaw        = 0.0f,
                      float pitch      = 0.3f);

    // Read mouse delta and update yaw/pitch.
    void Update(float dt);

    // Returns a raylib Camera3D ready for BeginMode3D.
    Camera GetCamera() const;

    float GetYaw()   const { return m_yaw; }
    float GetPitch() const { return m_pitch; }

    // Flat XZ unit vectors aligned to the camera arm direction
    Vector3 GetForwardXZ() const;
    Vector3 GetRightXZ()   const;

private:
    const Vector3* m_owner;
    float          m_yaw;
    float          m_pitch;

    static constexpr float kPitchMin = -1.3f;   // ~-75 deg
    static constexpr float kPitchMax =  1.3f;   // ~+75 deg
};