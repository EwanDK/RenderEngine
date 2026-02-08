#include "GJK.h"
#include "MathUtils.h"
#include <algorithm>
#include <array>
#include <float.h>

namespace Collision {

// ============================================================
// ConvexShape
// ============================================================

Vector3 ConvexShape::Support(Vector3 direction) const {
    float bestDot = -FLT_MAX;
    Vector3 bestVertex = vertices[0];

    for (int i = 0; i < count; i++) {
        float d = Vector3DotProduct(vertices[i], direction);
        if (d > bestDot) {
            bestDot = d;
            bestVertex = vertices[i];
        }
    }
    return bestVertex;
}

ConvexShape ConvexShapeFromPoints(const Vector3* points, int count) {
    ConvexShape shape;
    shape.vertices = points;
    shape.count = count;

    // Compute center (average of all points)
    Vector3 sum = {0, 0, 0};
    for (int i = 0; i < count; i++) {
        sum += points[i];
    }
    shape.center = (count > 0) ? sum * (1.0f / count) : Vector3{0, 0, 0};
    return shape;
}

ConvexShape ConvexShapeFromMesh(const Mesh& mesh, std::vector<Vector3>& extractedVerts) {
    extractedVerts.resize(mesh.vertexCount);
    for (int i = 0; i < mesh.vertexCount; i++) {
        extractedVerts[i] = {
            mesh.vertices[i * 3],
            mesh.vertices[i * 3 + 1],
            mesh.vertices[i * 3 + 2]
        };
    }
    return ConvexShapeFromPoints(extractedVerts.data(), (int)extractedVerts.size());
}

// ============================================================
// GJK internals
// ============================================================

static constexpr int GJK_MAX_ITERATIONS = 32;
static constexpr float GJK_EPSILON = 1e-6f;

// Minkowski difference support: furthest point of (A - B) along direction
static Vector3 MinkowskiSupport(const ConvexShape& a, const ConvexShape& b, Vector3 dir) {
    return a.Support(dir) - b.Support(dir * -1.0f);
}

// Triple product: (a x b) x c = b * dot(c,a) - a * dot(c,b)
static Vector3 TripleProduct(Vector3 a, Vector3 b, Vector3 c) {
    return b * Vector3DotProduct(c, a) - a * Vector3DotProduct(c, b);
}

// Simplex: up to 4 points, stored newest-last
struct Simplex {
    Vector3 pts[4];
    int size;

    Simplex() : size(0) {}

    void push(Vector3 v) {
        // Shift existing points down, put new point at the end
        // We keep the convention: newest point is at index [size-1]
        // But for the DoSimplex logic below, newest = 'A' = pts[size-1]
        pts[size] = v;
        size++;
    }

    void set(Vector3 a) { pts[0] = a; size = 1; }
    void set(Vector3 a, Vector3 b) { pts[0] = b; pts[1] = a; size = 2; }
    void set(Vector3 a, Vector3 b, Vector3 c) { pts[0] = c; pts[1] = b; pts[2] = a; size = 3; }
};

// DoSimplex subroutines
// Convention: A is the newest point (pts[size-1]), origin is at {0,0,0}

static bool DoSimplexLine(Simplex& s, Vector3& dir) {
    // s = {B, A}  where A=s.pts[1] is newest
    Vector3 A = s.pts[1];
    Vector3 B = s.pts[0];
    Vector3 AB = B - A;
    Vector3 AO = A * -1.0f; // toward origin

    if (Vector3DotProduct(AB, AO) > 0) {
        // Origin is in the region between A and B
        dir = TripleProduct(AB, AO, AB);
        if (Vector3LengthSqr(dir) < GJK_EPSILON) {
            // AB and AO are parallel -- pick any perpendicular
            dir = Vector3CrossProduct(AB, Vector3{1, 0, 0});
            if (Vector3LengthSqr(dir) < GJK_EPSILON)
                dir = Vector3CrossProduct(AB, Vector3{0, 0, 1});
        }
        // simplex stays {B, A}
    } else {
        // Origin is behind A
        s.set(A);
        dir = AO;
    }
    return false;
}

static bool DoSimplexTriangle(Simplex& s, Vector3& dir) {
    // s = {C, B, A}  where A=s.pts[2] is newest
    Vector3 A = s.pts[2];
    Vector3 B = s.pts[1];
    Vector3 C = s.pts[0];
    Vector3 AB = B - A;
    Vector3 AC = C - A;
    Vector3 AO = A * -1.0f;
    Vector3 ABC = Vector3CrossProduct(AB, AC); // triangle normal

    // Check if origin is outside edge AC (on the side away from B)
    Vector3 abcCrossAC = Vector3CrossProduct(ABC, AC);
    if (Vector3DotProduct(abcCrossAC, AO) > 0) {
        if (Vector3DotProduct(AC, AO) > 0) {
            // Region AC: closest to edge AC
            s.set(A, C);
            dir = TripleProduct(AC, AO, AC);
            if (Vector3LengthSqr(dir) < GJK_EPSILON)
                dir = Vector3CrossProduct(AC, ABC);
        } else {
            // Fall through to AB check
            s.set(A, B);
            return DoSimplexLine(s, dir);
        }
        return false;
    }

    // Check if origin is outside edge AB (on the side away from C)
    Vector3 abCrossABC = Vector3CrossProduct(AB, ABC);
    if (Vector3DotProduct(abCrossABC, AO) > 0) {
        s.set(A, B);
        return DoSimplexLine(s, dir);
    }

    // Origin is inside the triangle (projected onto its plane)
    if (Vector3DotProduct(ABC, AO) > 0) {
        // Origin is above the triangle
        dir = ABC;
        // keep {C, B, A}
    } else {
        // Origin is below the triangle -- flip winding
        s.set(A, B, C);
        dir = ABC * -1.0f;
    }
    return false;
}

static bool DoSimplexTetrahedron(Simplex& s, Vector3& dir) {
    // s = {D, C, B, A}  where A=s.pts[3] is newest
    Vector3 A = s.pts[3];
    Vector3 B = s.pts[2];
    Vector3 C = s.pts[1];
    Vector3 D = s.pts[0];
    Vector3 AB = B - A;
    Vector3 AC = C - A;
    Vector3 AD = D - A;
    Vector3 AO = A * -1.0f;

    // Face ABC (opposite D): normal should point away from D
    Vector3 ABC = Vector3CrossProduct(AB, AC);
    if (Vector3DotProduct(ABC, AD) > 0) ABC = ABC * -1.0f; // flip if pointing toward D

    // Face ACD (opposite B): normal should point away from B
    Vector3 ACD = Vector3CrossProduct(AC, AD);
    if (Vector3DotProduct(ACD, AB) > 0) ACD = ACD * -1.0f;

    // Face ADB (opposite C): normal should point away from C
    Vector3 ADB = Vector3CrossProduct(AD, AB);
    if (Vector3DotProduct(ADB, AC) > 0) ADB = ADB * -1.0f;

    // Check each face
    if (Vector3DotProduct(ABC, AO) > 0) {
        // Origin is on the ABC side -- reduce to triangle
        s.set(A, B, C);
        dir = ABC;
        return DoSimplexTriangle(s, dir);
    }
    if (Vector3DotProduct(ACD, AO) > 0) {
        s.set(A, C, D);
        dir = ACD;
        return DoSimplexTriangle(s, dir);
    }
    if (Vector3DotProduct(ADB, AO) > 0) {
        s.set(A, D, B);
        dir = ADB;
        return DoSimplexTriangle(s, dir);
    }

    // Origin is inside the tetrahedron
    return true;
}

static bool DoSimplex(Simplex& s, Vector3& dir) {
    switch (s.size) {
        case 2: return DoSimplexLine(s, dir);
        case 3: return DoSimplexTriangle(s, dir);
        case 4: return DoSimplexTetrahedron(s, dir);
    }
    return false;
}

// Core GJK: returns true if intersecting, and fills the simplex
static bool GJK_Run(const ConvexShape& a, const ConvexShape& b, Simplex& simplex) {
    // Initial search direction: from center of A toward center of B
    Vector3 dir = b.center - a.center;
    if (Vector3LengthSqr(dir) < GJK_EPSILON) {
        dir = {1.0f, 0.0f, 0.0f}; // arbitrary if centers coincide
    }

    // First support point
    Vector3 support = MinkowskiSupport(a, b, dir);
    simplex.push(support);
    dir = support * -1.0f; // search toward origin

    for (int i = 0; i < GJK_MAX_ITERATIONS; i++) {
        support = MinkowskiSupport(a, b, dir);

        // If the new point did not pass the origin, no intersection
        if (Vector3DotProduct(support, dir) < 0) {
            return false;
        }

        simplex.push(support);

        if (DoSimplex(simplex, dir)) {
            return true; // origin is enclosed
        }

        // Guard against zero direction (degenerate case)
        if (Vector3LengthSqr(dir) < GJK_EPSILON) {
            return false;
        }
    }

    return false; // did not converge
}

// ============================================================
// EPA (Expanding Polytope Algorithm)
// ============================================================

static constexpr int EPA_MAX_ITERATIONS = 64;
static constexpr float EPA_TOLERANCE = 0.0001f;

struct EPAFace {
    int indices[3];
    Vector3 normal;
    float distance; // distance from origin to face plane
};

struct EPAEdge {
    int a, b;
};

// Add a face to the polytope, ensuring normal points away from centroid
static void AddEPAFace(std::vector<EPAFace>& faces, const std::vector<Vector3>& verts,
                        int i0, int i1, int i2, Vector3 centroid) {
    Vector3 e1 = verts[i1] - verts[i0];
    Vector3 e2 = verts[i2] - verts[i0];
    Vector3 n = Vector3CrossProduct(e1, e2);
    float len = Vector3Length(n);
    if (len < 1e-8f) return; // degenerate face, skip
    n = n * (1.0f / len);

    // Ensure normal points away from centroid
    if (Vector3DotProduct(n, verts[i0] - centroid) < 0) {
        n = n * -1.0f;
        std::swap(i1, i2);
    }

    float dist = Vector3DotProduct(n, verts[i0]);
    if (dist < 0) {
        // Origin is on the wrong side -- flip everything
        n = n * -1.0f;
        dist = -dist;
        std::swap(i1, i2);
    }

    EPAFace face;
    face.indices[0] = i0;
    face.indices[1] = i1;
    face.indices[2] = i2;
    face.normal = n;
    face.distance = dist;
    faces.push_back(face);
}

// Add edge to boundary, removing shared edges (silhouette detection)
static void AddEdgeUnique(std::vector<EPAEdge>& edges, int a, int b) {
    for (size_t i = 0; i < edges.size(); i++) {
        if (edges[i].a == b && edges[i].b == a) {
            // Remove the shared edge
            edges.erase(edges.begin() + i);
            return;
        }
    }
    edges.push_back({a, b});
}

static CollisionResult EPA_Run(const ConvexShape& a, const ConvexShape& b, Simplex& gjkSimplex) {
    CollisionResult result = {};
    result.collided = true;

    // EPA needs a tetrahedron (4 points). If GJK gave us fewer, we can't proceed.
    if (gjkSimplex.size < 4) {
        // Collision detected but can't compute penetration depth.
        // Return a basic result with estimated normal.
        if (gjkSimplex.size == 3) {
            Vector3 A = gjkSimplex.pts[0];
            Vector3 B = gjkSimplex.pts[1];
            Vector3 C = gjkSimplex.pts[2];
            Vector3 n = Vector3CrossProduct(B - A, C - A);
            result.normal = SafeNormalize(n);
            result.depth = 0.001f;
            result.contactPoint = (a.center + b.center) * 0.5f;
        } else {
            result.normal = SafeNormalize(b.center - a.center);
            result.depth = 0.001f;
            result.contactPoint = (a.center + b.center) * 0.5f;
        }
        return result;
    }

    // Initialize polytope from GJK simplex
    std::vector<Vector3> polytope;
    polytope.reserve(64);
    for (int i = 0; i < 4; i++) {
        polytope.push_back(gjkSimplex.pts[i]);
    }

    // Centroid of initial tetrahedron
    Vector3 centroid = (polytope[0] + polytope[1] + polytope[2] + polytope[3]) * 0.25f;

    // Build 4 initial faces
    std::vector<EPAFace> faces;
    faces.reserve(64);
    AddEPAFace(faces, polytope, 0, 1, 2, centroid);
    AddEPAFace(faces, polytope, 0, 3, 1, centroid);
    AddEPAFace(faces, polytope, 0, 2, 3, centroid);
    AddEPAFace(faces, polytope, 1, 3, 2, centroid);

    for (int iter = 0; iter < EPA_MAX_ITERATIONS; iter++) {
        if (faces.empty()) break;

        // Find the closest face to the origin
        int closestIdx = 0;
        float minDist = faces[0].distance;
        for (size_t f = 1; f < faces.size(); f++) {
            if (faces[f].distance < minDist) {
                minDist = faces[f].distance;
                closestIdx = (int)f;
            }
        }

        Vector3 minNormal = faces[closestIdx].normal;

        // Get new support point along the closest face's normal
        Vector3 newPoint = MinkowskiSupport(a, b, minNormal);
        float newDist = Vector3DotProduct(newPoint, minNormal);

        // Check convergence
        if (newDist - minDist < EPA_TOLERANCE) {
            result.normal = minNormal;
            result.depth = minDist;
            result.contactPoint = (a.center + b.center) * 0.5f + minNormal * (minDist * 0.5f);
            return result;
        }

        // Remove faces that can "see" the new point
        std::vector<EPAEdge> edges;
        for (int f = (int)faces.size() - 1; f >= 0; f--) {
            Vector3 facePoint = polytope[faces[f].indices[0]];
            if (Vector3DotProduct(faces[f].normal, newPoint - facePoint) > 0) {
                // Face can see the new point -- remove and collect boundary edges
                AddEdgeUnique(edges, faces[f].indices[0], faces[f].indices[1]);
                AddEdgeUnique(edges, faces[f].indices[1], faces[f].indices[2]);
                AddEdgeUnique(edges, faces[f].indices[2], faces[f].indices[0]);
                faces.erase(faces.begin() + f);
            }
        }

        // Add new vertex
        int newIdx = (int)polytope.size();
        polytope.push_back(newPoint);

        // Create new faces from boundary edges to the new point
        for (auto& edge : edges) {
            AddEPAFace(faces, polytope, edge.a, edge.b, newIdx, centroid);
        }
    }

    // Return best estimate if we didn't converge
    if (!faces.empty()) {
        int closestIdx = 0;
        float minDist = faces[0].distance;
        for (size_t f = 1; f < faces.size(); f++) {
            if (faces[f].distance < minDist) {
                minDist = faces[f].distance;
                closestIdx = (int)f;
            }
        }
        result.normal = faces[closestIdx].normal;
        result.depth = minDist;
        result.contactPoint = (a.center + b.center) * 0.5f + result.normal * (minDist * 0.5f);
    } else {
        result.normal = SafeNormalize(b.center - a.center);
        result.depth = 0.001f;
        result.contactPoint = (a.center + b.center) * 0.5f;
    }

    return result;
}

// ============================================================
// Public API
// ============================================================

bool Intersect(const ConvexShape& a, const ConvexShape& b) {
    Simplex simplex;
    return GJK_Run(a, b, simplex);
}

CollisionResult IntersectDetailed(const ConvexShape& a, const ConvexShape& b) {
    Simplex simplex;
    bool hit = GJK_Run(a, b, simplex);
    if (!hit) {
        CollisionResult result = {};
        result.collided = false;
        return result;
    }
    return EPA_Run(a, b, simplex);
}

} // namespace Collision
