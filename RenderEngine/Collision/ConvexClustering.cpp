#include "ConvexClustering.h"
#include "SoftBody.h"
#include "MathUtils.h"
#include <algorithm>
#include <random>
#include <limits>
#include <cmath>
#include <set>

namespace Collision {

// ============================================================================
// K-means++ clustering
// ============================================================================

std::vector<ConvexCluster> BuildClusters(
    const float* restPositions,
    int vertexCount,
    int k)
{
    if (k <= 0 || vertexCount <= 0) return {};

    // If k >= vertexCount, one vertex per cluster
    if (k >= vertexCount) {
        std::vector<ConvexCluster> clusters(vertexCount);
        for (int i = 0; i < vertexCount; i++) {
            clusters[i].pointIndices.push_back(i);
        }
        return clusters;
    }

    // Extract rest positions as Vector3
    std::vector<Vector3> positions(vertexCount);
    for (int i = 0; i < vertexCount; i++) {
        positions[i] = { restPositions[i*3], restPositions[i*3+1], restPositions[i*3+2] };
    }

    // K-means++ seeding
    std::vector<Vector3> centroids(k);
    std::mt19937 rng(42); // deterministic for reproducibility

    // First centroid: random vertex
    std::uniform_int_distribution<int> dist(0, vertexCount - 1);
    centroids[0] = positions[dist(rng)];

    // Subsequent centroids: probability proportional to squared distance from nearest centroid
    std::vector<float> minDists(vertexCount, std::numeric_limits<float>::max());
    for (int c = 1; c < k; c++) {
        for (int i = 0; i < vertexCount; i++) {
            Vector3 diff = positions[i] - centroids[c-1];
            float d2 = Vector3DotProduct(diff, diff);
            if (d2 < minDists[i]) minDists[i] = d2;
        }

        float totalWeight = 0.0f;
        for (int i = 0; i < vertexCount; i++) totalWeight += minDists[i];

        std::uniform_real_distribution<float> weightDist(0.0f, totalWeight);
        float r = weightDist(rng);
        float cumulative = 0.0f;
        int chosen = 0;
        for (int i = 0; i < vertexCount; i++) {
            cumulative += minDists[i];
            if (cumulative >= r) { chosen = i; break; }
        }
        centroids[c] = positions[chosen];
    }

    // Iterate: assign vertices to nearest centroid, update centroids
    std::vector<int> assignments(vertexCount, 0);
    const int maxIterations = 50;

    for (int iter = 0; iter < maxIterations; iter++) {
        bool changed = false;

        // Assign
        for (int i = 0; i < vertexCount; i++) {
            float bestDist = std::numeric_limits<float>::max();
            int bestCluster = 0;
            for (int c = 0; c < k; c++) {
                Vector3 diff = positions[i] - centroids[c];
                float d2 = Vector3DotProduct(diff, diff);
                if (d2 < bestDist) {
                    bestDist = d2;
                    bestCluster = c;
                }
            }
            if (assignments[i] != bestCluster) {
                assignments[i] = bestCluster;
                changed = true;
            }
        }

        if (!changed) break;

        // Update centroids
        for (int c = 0; c < k; c++) {
            Vector3 sum = {0, 0, 0};
            int count = 0;
            for (int i = 0; i < vertexCount; i++) {
                if (assignments[i] == c) {
                    sum += positions[i];
                    count++;
                }
            }
            if (count > 0) {
                centroids[c] = sum * (1.0f / static_cast<float>(count));
            }
        }
    }

    // Build cluster structs
    std::vector<ConvexCluster> clusters(k);
    for (int i = 0; i < vertexCount; i++) {
        clusters[assignments[i]].pointIndices.push_back(i);
    }

    // Remove empty clusters
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
            [](const ConvexCluster& c) { return c.pointIndices.empty(); }),
        clusters.end()
    );

    return clusters;
}

// ============================================================================
// QuickHull 3D
// ============================================================================

namespace {

struct QHFace {
    int v[3];
    Vector3 normal;
    float dist; // distance from origin along normal
    std::vector<int> outsideSet;
    bool alive = true;
};

void ComputeFaceNormal(QHFace& face, const std::vector<Vector3>& pts, const Vector3& centroid) {
    Vector3 a = pts[face.v[0]];
    Vector3 b = pts[face.v[1]];
    Vector3 c = pts[face.v[2]];

    Vector3 n = Vector3CrossProduct(b - a, c - a);
    n = SafeNormalize(n);

    // Ensure normal points away from centroid
    if (Vector3DotProduct(n, a - centroid) < 0.0f) {
        n = n * -1.0f;
        std::swap(face.v[1], face.v[2]);
    }

    face.normal = n;
    face.dist = Vector3DotProduct(n, a);
}

// Signed distance from point to face plane
float PointFaceDist(const Vector3& p, const QHFace& face) {
    return Vector3DotProduct(face.normal, p) - face.dist;
}

struct Edge {
    int a, b;
    bool operator==(const Edge& o) const { return (a == o.a && b == o.b) || (a == o.b && b == o.a); }
};

} // anonymous namespace

std::vector<Vector3> QuickHull3D(const std::vector<Vector3>& inputPoints) {
    const int n = static_cast<int>(inputPoints.size());
    if (n < 4) return inputPoints;

    // 1. Find extreme points along each axis
    int extremes[6] = {0, 0, 0, 0, 0, 0}; // minX, maxX, minY, maxY, minZ, maxZ
    for (int i = 1; i < n; i++) {
        if (inputPoints[i].x < inputPoints[extremes[0]].x) extremes[0] = i;
        if (inputPoints[i].x > inputPoints[extremes[1]].x) extremes[1] = i;
        if (inputPoints[i].y < inputPoints[extremes[2]].y) extremes[2] = i;
        if (inputPoints[i].y > inputPoints[extremes[3]].y) extremes[3] = i;
        if (inputPoints[i].z < inputPoints[extremes[4]].z) extremes[4] = i;
        if (inputPoints[i].z > inputPoints[extremes[5]].z) extremes[5] = i;
    }

    // 2. Find most distant pair among extremes
    int bestA = 0, bestB = 1;
    float bestDist2 = 0.0f;
    for (int i = 0; i < 6; i++) {
        for (int j = i + 1; j < 6; j++) {
            Vector3 diff = inputPoints[extremes[i]] - inputPoints[extremes[j]];
            float d2 = Vector3DotProduct(diff, diff);
            if (d2 > bestDist2) {
                bestDist2 = d2;
                bestA = extremes[i];
                bestB = extremes[j];
            }
        }
    }

    if (bestDist2 < 1e-10f) return inputPoints; // All points coincident

    // 3. Find point most distant from line (bestA, bestB)
    Vector3 lineDir = SafeNormalize(inputPoints[bestB] - inputPoints[bestA]);
    float maxLineDist = 0.0f;
    int bestC = -1;
    for (int i = 0; i < n; i++) {
        if (i == bestA || i == bestB) continue;
        Vector3 toPoint = inputPoints[i] - inputPoints[bestA];
        Vector3 projected = lineDir * Vector3DotProduct(toPoint, lineDir);
        float d2 = Vector3LengthSqr(toPoint - projected);
        if (d2 > maxLineDist) {
            maxLineDist = d2;
            bestC = i;
        }
    }
    if (bestC < 0) return inputPoints; // All collinear

    // 4. Find point most distant from triangle plane
    Vector3 triNormal = SafeNormalize(Vector3CrossProduct(
        inputPoints[bestB] - inputPoints[bestA],
        inputPoints[bestC] - inputPoints[bestA]
    ));
    float maxPlaneDist = 0.0f;
    int bestD = -1;
    for (int i = 0; i < n; i++) {
        if (i == bestA || i == bestB || i == bestC) continue;
        float d = fabsf(Vector3DotProduct(inputPoints[i] - inputPoints[bestA], triNormal));
        if (d > maxPlaneDist) {
            maxPlaneDist = d;
            bestD = i;
        }
    }
    if (bestD < 0) return inputPoints; // All coplanar

    // Orient: ensure D is on the negative side of ABC
    float sign = Vector3DotProduct(inputPoints[bestD] - inputPoints[bestA], triNormal);
    if (sign > 0.0f) std::swap(bestB, bestC);

    // 5. Build initial tetrahedron (4 faces)
    // Compute centroid for outward normal orientation
    Vector3 centroid = (inputPoints[bestA] + inputPoints[bestB] + inputPoints[bestC] + inputPoints[bestD]) * 0.25f;

    std::vector<QHFace> faces;
    faces.reserve(64);

    auto addFace = [&](int i0, int i1, int i2) -> int {
        QHFace f;
        f.v[0] = i0; f.v[1] = i1; f.v[2] = i2;
        ComputeFaceNormal(f, inputPoints, centroid);
        faces.push_back(std::move(f));
        return static_cast<int>(faces.size()) - 1;
    };

    addFace(bestA, bestB, bestC);
    addFace(bestA, bestC, bestD);
    addFace(bestA, bestD, bestB);
    addFace(bestB, bestD, bestC);

    // 6. Assign all other points to the face they're furthest above
    std::vector<bool> onHull(n, false);
    onHull[bestA] = onHull[bestB] = onHull[bestC] = onHull[bestD] = true;

    for (int i = 0; i < n; i++) {
        if (onHull[i]) continue;

        float bestFaceDist = 0.0f;
        int bestFace = -1;
        for (int f = 0; f < static_cast<int>(faces.size()); f++) {
            if (!faces[f].alive) continue;
            float d = PointFaceDist(inputPoints[i], faces[f]);
            if (d > 1e-6f && d > bestFaceDist) {
                bestFaceDist = d;
                bestFace = f;
            }
        }
        if (bestFace >= 0) {
            faces[bestFace].outsideSet.push_back(i);
        }
    }

    // 7. Iterative expansion
    const int maxIter = 200;
    for (int iter = 0; iter < maxIter; iter++) {
        // Find face with the furthest outside point
        int targetFace = -1;
        float furthestDist = 0.0f;
        int furthestPoint = -1;

        for (int f = 0; f < static_cast<int>(faces.size()); f++) {
            if (!faces[f].alive || faces[f].outsideSet.empty()) continue;

            for (int pi : faces[f].outsideSet) {
                float d = PointFaceDist(inputPoints[pi], faces[f]);
                if (d > furthestDist) {
                    furthestDist = d;
                    furthestPoint = pi;
                    targetFace = f;
                }
            }
        }

        if (targetFace < 0) break; // No more outside points

        Vector3 eye = inputPoints[furthestPoint];

        // Find all faces visible from the eye point
        std::vector<int> visibleFaces;
        for (int f = 0; f < static_cast<int>(faces.size()); f++) {
            if (!faces[f].alive) continue;
            if (PointFaceDist(eye, faces[f]) > 1e-6f) {
                visibleFaces.push_back(f);
            }
        }

        // Extract horizon edges (boundary of visible region)
        std::vector<Edge> horizon;
        for (int fi : visibleFaces) {
            QHFace& face = faces[fi];
            // Each face has 3 edges
            Edge edges[3] = {
                {face.v[0], face.v[1]},
                {face.v[1], face.v[2]},
                {face.v[2], face.v[0]}
            };

            for (auto& edge : edges) {
                // Check if the adjacent face (sharing this edge) is NOT visible
                bool edgeShared = false;
                for (int fj : visibleFaces) {
                    if (fj == fi) continue;
                    QHFace& other = faces[fj];
                    // Check if other face shares this edge
                    int sharedVerts = 0;
                    for (int a = 0; a < 3; a++) {
                        if (other.v[a] == edge.a || other.v[a] == edge.b) sharedVerts++;
                    }
                    if (sharedVerts == 2) { edgeShared = true; break; }
                }
                if (!edgeShared) {
                    horizon.push_back(edge);
                }
            }
        }

        // Collect orphaned outside points from visible faces
        std::vector<int> orphanedPoints;
        for (int fi : visibleFaces) {
            for (int pi : faces[fi].outsideSet) {
                if (pi != furthestPoint) {
                    orphanedPoints.push_back(pi);
                }
            }
            faces[fi].alive = false;
            faces[fi].outsideSet.clear();
        }

        // Create new faces from each horizon edge to the eye point
        // Update centroid with the new hull vertex
        onHull[furthestPoint] = true;

        // Recompute centroid from all hull vertices
        Vector3 newCentroid = {0, 0, 0};
        int hullCount = 0;
        for (int i = 0; i < n; i++) {
            if (onHull[i]) { newCentroid += inputPoints[i]; hullCount++; }
        }
        if (hullCount > 0) centroid = newCentroid * (1.0f / static_cast<float>(hullCount));

        std::vector<int> newFaceIndices;
        for (auto& edge : horizon) {
            int fi = addFace(edge.a, edge.b, furthestPoint);
            newFaceIndices.push_back(fi);
        }

        // Reassign orphaned points to new faces
        for (int pi : orphanedPoints) {
            float bestD = 0.0f;
            int bestF = -1;
            for (int fi : newFaceIndices) {
                if (!faces[fi].alive) continue;
                float d = PointFaceDist(inputPoints[pi], faces[fi]);
                if (d > 1e-6f && d > bestD) {
                    bestD = d;
                    bestF = fi;
                }
            }
            if (bestF >= 0) {
                faces[bestF].outsideSet.push_back(pi);
            }
        }
    }

    // 8. Collect unique hull vertex indices from surviving faces
    std::set<int> hullIndices;
    for (auto& f : faces) {
        if (!f.alive) continue;
        hullIndices.insert(f.v[0]);
        hullIndices.insert(f.v[1]);
        hullIndices.insert(f.v[2]);
    }

    std::vector<Vector3> result;
    result.reserve(hullIndices.size());
    for (int idx : hullIndices) {
        result.push_back(inputPoints[idx]);
    }
    return result;
}

// ============================================================================
// Update cluster hulls from current particle positions
// ============================================================================

void UpdateClusterHulls(
    std::vector<ConvexCluster>& clusters,
    const std::vector<Point>& points)
{
    for (auto& cluster : clusters) {
        // Gather current positions for this cluster's particles
        std::vector<Vector3> currentPositions;
        currentPositions.reserve(cluster.pointIndices.size());
        for (int idx : cluster.pointIndices) {
            currentPositions.push_back(points[idx].position);
        }

        // Compute convex hull
        cluster.hullVertices = QuickHull3D(currentPositions);

        // Rebuild ConvexShape pointing into the new hull data
        if (!cluster.hullVertices.empty()) {
            cluster.shape = ConvexShapeFromPoints(
                cluster.hullVertices.data(),
                static_cast<int>(cluster.hullVertices.size())
            );
        } else {
            cluster.shape = {nullptr, 0, {0,0,0}};
        }
    }
}

} // namespace Collision
