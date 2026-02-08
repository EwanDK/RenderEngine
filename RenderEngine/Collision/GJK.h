#pragma once
#include "raylib.h"
#include "raymath.h"
#include <vector>

namespace Collision {

    // A convex shape defined by a set of vertices.
    // Non-owning: the caller must keep the vertex data alive.
    struct ConvexShape {
        const Vector3* vertices;
        int count;
        Vector3 center;

        // Find the vertex furthest along the given direction
        Vector3 Support(Vector3 direction) const;
    };

    // Build a ConvexShape from a raw point array
    ConvexShape ConvexShapeFromPoints(const Vector3* points, int count);

    // Build a ConvexShape by extracting vertices from a raylib Mesh
    // NOTE: returned shape owns no memory -- the extractedVerts output vector must stay alive
    ConvexShape ConvexShapeFromMesh(const Mesh& mesh, std::vector<Vector3>& extractedVerts);

    // Result of a detailed collision query (GJK + EPA)
    struct CollisionResult {
        bool collided;
        Vector3 normal;       // penetration normal (from A toward B)
        float depth;          // penetration depth along normal
        Vector3 contactPoint; // approximate contact point in world space
    };

    // Boolean intersection test (GJK only, no EPA)
    bool Intersect(const ConvexShape& a, const ConvexShape& b);

    // Detailed intersection test (GJK + EPA for penetration info)
    CollisionResult IntersectDetailed(const ConvexShape& a, const ConvexShape& b);

} // namespace Collision
