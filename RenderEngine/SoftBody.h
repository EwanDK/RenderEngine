#pragma once
#include <vector>

#include "raylib.h"

struct Point{
    Vector3 position;
    Vector3 speed;
    Vector3 force;
    float mass;
    const float radius = 1.f;
    int index;
};

struct Spring{
    Point* points[2];
    float baseDistance;
    float stiffness;
    float damping;
    int virtualDistance=0;
};

// Include collision after Point is fully defined (these headers forward-declare Point)
#include "Collision/CollisionSystem.h"
#include "Collision/ConvexClustering.h"

class SoftBody
{
public:
    SoftBody();
    void SolveSpring(Spring& spring);
    void ClampSpringForce(Spring& spring);
    void ApplyShapeMatching(float stiffness, float dt);
    void Solve(float dt);
    void Draw();
    void RecomputeNormals(Mesh& mesh);
    void Update(float dt);
    Model model;

    // Collision
    void SetGroundPlane(const Collision::GroundPlane& ground);
    void AddStaticCollider(const Collision::ConvexShape& shape);
    void ClearStaticColliders();
    void Translate(Vector3 offset);
    void BuildClusters(int k = 6);
    void UpdateClusterHulls();

    std::vector<Point>& GetPoints() { return points; }
    const std::vector<Point>& GetPoints() const { return points; }

private:
    std::vector<Spring> springs;
    std::vector<Point> points;
    float* restPositions;

    // Collision data
    bool hasGroundPlane = false;
    Collision::GroundPlane groundPlane{{0,1,0}, 0.0f};
    std::vector<Collision::ConvexShape> staticColliders;
    std::vector<Collision::ConvexCluster> clusters;
};
