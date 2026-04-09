#pragma once
#include "raylib.h"
#include "raymath.h"
#include "GJK.h"
#include <vector>
#include <optional>

#ifndef SOFTBODY_POINT_DEFINED
#define SOFTBODY_POINT_DEFINED
struct Point {
    Vector3 position;
    Vector3 speed;
    Vector3 force;
    float mass;
    const float radius = 1.f;
    int index;
    bool isStatic = false;
};
#endif

namespace Collision {

    struct GroundPlane {
        Vector3 normal;
        float height;

        GroundPlane(Vector3 n, float h)
            : normal(Vector3Normalize(n)), height(h) {}
    };

    // Per-particle half-space collision against a ground plane.
    // Returns number of particles that were corrected.
    int ResolveGroundCollision(
        std::vector<Point>& points,
        const GroundPlane& ground,
        float restitution = 0.3f,
        float friction = 0.85f
    );

    // GJK+EPA test between two convex shapes.
    // Returns collision result if intersecting.
    std::optional<CollisionResult> TestConvex(
        const ConvexShape& a,
        const ConvexShape& b
    );

    // Per-particle collision response against a static convex shape.
    // Tests each particle individually via GJK+EPA and pushes penetrating ones out.
    void ResolveSoftVsStatic(
        std::vector<Point>& points,
        const ConvexShape& staticShape,
        float restitution = 0.3f,
        float friction = 0.5f
    );

    // Debug contact info for visualization
    struct DebugContact {
        Vector3 position;
        Vector3 normal;
        float depth;
    };

    // Per-particle collision for a subset of particles (used by cluster broadphase).
    void ResolveSoftVsStaticSubset(
        std::vector<Point>& points,
        const std::vector<int>& indices,
        const ConvexShape& staticShape,
        float restitution = 0.3f,
        float friction = 0.5f,
        std::vector<DebugContact>* debugContacts = nullptr
    );

} // namespace Collision
