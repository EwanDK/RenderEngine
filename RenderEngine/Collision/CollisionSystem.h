#pragma once
#include "raylib.h"
#include "raymath.h"
#include "GJK.h"
#include <vector>
#include <optional>

// Forward declare Point (from SoftBody.h)
struct Point;

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

    // Apply collision response to soft body particles against a static shape.
    // Uses the contact normal/depth to push penetrating particles out
    // and apply velocity impulse.
    void ResolveSoftVsStatic(
        std::vector<Point>& points,
        const CollisionResult& contact,
        float restitution = 0.3f,
        float friction = 0.5f
    );

} // namespace Collision
