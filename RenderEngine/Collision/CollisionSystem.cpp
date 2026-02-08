#include "SoftBody.h"
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
    const CollisionResult& contact,
    float restitution,
    float friction)
{
    const float skinWidth = 0.01f;

    // The contact plane: particles on the "wrong side" need correction.
    // Plane defined by: dot(normal, p) = dot(normal, contactPoint)
    float planeD = Vector3DotProduct(contact.normal, contact.contactPoint);

    for (size_t i = 0; i < points.size(); i++) {
        float signedDist = Vector3DotProduct(contact.normal, points[i].position) - planeD;

        if (signedDist < 0.0f) {
            // This particle penetrates the static shape
            float penetration = -signedDist;

            // Position correction: push out along contact normal
            points[i].position += contact.normal * (penetration + skinWidth);

            // Velocity impulse along collision normal
            float vn = Vector3DotProduct(points[i].speed, contact.normal);
            if (vn < 0.0f) {
                // Remove normal velocity component and add restitution bounce
                points[i].speed -= contact.normal * ((1.0f + restitution) * vn);

                // Apply friction to tangential component
                Vector3 vNormal = contact.normal * Vector3DotProduct(points[i].speed, contact.normal);
                Vector3 vTangent = points[i].speed - vNormal;
                float tangentLen = Vector3Length(vTangent);
                if (tangentLen > 1e-6f) {
                    points[i].speed = vNormal + vTangent * friction;
                }
            }
        }
    }
}

} // namespace Collision
