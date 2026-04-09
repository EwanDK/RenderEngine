//#include "SoftBody.h"
#include "CollisionSystem.h"
#include "MathUtils.h"

namespace Collision {

int ResolveGroundCollision(
    std::vector<Point>& points,
    const GroundPlane& ground,
    float restitution,
    float friction)
{
    int correctedCount = 0;
    const float skinWidth = 0.01f;

    for (size_t i = 0; i < points.size(); i++) {
        // Signed distance from point to plane: positive = above, negative = below
        float signedDist = Vector3DotProduct(ground.normal, points[i].position) - ground.height;

        if (signedDist < 0.0f) {
            // Particle is below the ground -- push it out
            points[i].position += ground.normal * (-signedDist + skinWidth);

            // Velocity response
            float vn = Vector3DotProduct(points[i].speed, ground.normal);
            if (vn < 0.0f) {
                // Moving into the ground -- reflect normal component with restitution
                points[i].speed -= ground.normal * ((1.0f + restitution) * vn);

                // Apply friction to tangential component
                Vector3 vNormal = ground.normal * Vector3DotProduct(points[i].speed, ground.normal);
                Vector3 vTangent = points[i].speed - vNormal;
                float tangentLen = Vector3Length(vTangent);
                if (tangentLen > 1e-6f) {
                    points[i].speed = vNormal + vTangent * friction;
                }
            }

            correctedCount++;
        }
    }

    return correctedCount;
}

std::optional<CollisionResult> TestConvex(const ConvexShape& a, const ConvexShape& b) {
    CollisionResult result = IntersectDetailed(a, b);
    if (!result.collided) {
        return std::nullopt;
    }
    return result;
}

void ResolveSoftVsStatic(
    std::vector<Point>& points,
    const ConvexShape& staticShape,
    float restitution,
    float friction)
{
    const float skinWidth = 0.01f;

    for (size_t i = 0; i < points.size(); i++) {
        // Test this single particle against the static shape
        ConvexShape pointShape;
        pointShape.vertices = &points[i].position;
        pointShape.count = 1;
        pointShape.center = points[i].position;

        CollisionResult result = IntersectDetailed(pointShape, staticShape);
        if (!result.collided) continue;

        // Normal points from particle toward static shape — negate to push out
        Vector3 pushDir = result.normal * -1.0f;
        float depth = result.depth;

        points[i].position += pushDir * (depth + skinWidth);

        // Velocity response
        float vn = Vector3DotProduct(points[i].speed, pushDir);
        if (vn < 0.0f) {
            points[i].speed -= pushDir * ((1.0f + restitution) * vn);

            Vector3 vNormal = pushDir * Vector3DotProduct(points[i].speed, pushDir);
            Vector3 vTangent = points[i].speed - vNormal;
            if (Vector3Length(vTangent) > 1e-6f) {
                points[i].speed = vNormal + vTangent * friction;
            }
        }
    }
}

void ResolveSoftVsStaticSubset(
    std::vector<Point>& points,
    const std::vector<int>& indices,
    const ConvexShape& staticShape,
    float restitution,
    float friction,
    std::vector<DebugContact>* debugContacts)
{
    const float skinWidth = 0.01f;

    for (int i : indices) {
        ConvexShape pointShape;
        pointShape.vertices = &points[i].position;
        pointShape.count = 1;
        pointShape.center = points[i].position;

        CollisionResult result = IntersectDetailed(pointShape, staticShape);
        if (!result.collided) continue;

        Vector3 pushDir = result.normal * -1.0f;
        float depth = result.depth;

        points[i].position += pushDir * (depth + skinWidth);

        float vn = Vector3DotProduct(points[i].speed, pushDir);
        if (vn < 0.0f) {
            points[i].speed -= pushDir * ((1.0f + restitution) * vn);

            Vector3 vNormal = pushDir * Vector3DotProduct(points[i].speed, pushDir);
            Vector3 vTangent = points[i].speed - vNormal;
            if (Vector3Length(vTangent) > 1e-6f) {
                points[i].speed = vNormal + vTangent * friction;
            }
        }

        if (debugContacts) {
            debugContacts->push_back({points[i].position, pushDir, depth});
        }
    }
}

} // namespace Collision
