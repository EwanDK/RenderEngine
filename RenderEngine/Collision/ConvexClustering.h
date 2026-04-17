#pragma once
#include "raylib.h"
#include "GJK.h"
#include <vector>

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

    // A convex cluster: a spatial subset of the soft body's particles,
    // with a per-frame convex hull for broadphase collision testing.
    struct ConvexCluster {
        std::vector<int> pointIndices;       // indices into SoftBody::points
        std::vector<Vector3> hullVertices;   // rebuilt each frame from current positions
        ConvexShape shape{nullptr, 0, {0,0,0}}; // non-owning, points into hullVertices
    };

    // K-means++ clustering on rest positions. Run once at construction.
    // Returns k clusters with pointIndices populated (hulls NOT built yet).
    std::vector<ConvexCluster> BuildClusters(
        const float* restPositions,
        int vertexCount,
        int k
    );

    // Rebuild convex hulls for all clusters from current particle positions.
    // Call each frame (or each constraint iteration) before broadphase testing.
    void UpdateClusterHulls(
        std::vector<ConvexCluster>& clusters,
        const std::vector<Point>& points
    );

    // 3D QuickHull: compute convex hull vertices from an input point set.
    // Returns the subset of points forming the hull.
    // For inputs with fewer than 4 points, returns all input points.
    std::vector<Vector3> QuickHull3D(const std::vector<Vector3>& inputPoints);

} // namespace Collision
