#pragma once
#include <vector>
#include <set>

#include "raylib.h"

// ---- Definition structs (index-based, no Point pointers) ----

struct SpringDef {
    int index1, index2;
    float baseDistance;
    float stiffness;
    float damping;
    float muscleForce = 0.f;
    int   muscleGroup = -1;
};

struct StrutDef {
    int index1, index2;
    float distance; // enforced exact distance
};

// ---- Runtime structs (store raw Point pointers after body is built) ----

struct Point{
    Vector3 position;
    Vector3 speed;
    Vector3 force;
    float mass;
    const float radius = 1.f;
    int index;
    bool isStatic = false;
};

struct Strut {
    Point* a;
    Point* b;
    float distance;

    Strut() = default;
    Strut(const StrutDef& def, std::vector<Point>& pts);
};

struct Spring{
    Point* points[2];
    float baseDistance;
    float stiffness;
    float damping;
    int virtualDistance = 0;
    // Muscle fields (0 / -1 = passive spring)
    float muscleForce = 0.f; // compression force applied when group is active (>0 compress, <0 extend)
    int muscleGroup = -1;  // -1 = not a muscle

    Spring() = default;
    Spring(const SpringDef& def, std::vector<Point>& pts);
};

// Include collision after Point is fully defined (these headers forward-declare Point)
#include "Collision/CollisionSystem.h"
#include "Collision/ConvexClustering.h"

class SoftBody
{
public:
    enum class ConstraintMode { ShapeMatching, VolumePreservation };

    // Default: generates a UV sphere
    SoftBody(ConstraintMode mode = ConstraintMode::ShapeMatching);

    // Custom: caller supplies mesh, springs, struts, and optional rest/volume config.
    // staticPoints[i] = true pins point i in place.
    // restMesh: optional target pose for ShapeMatching (falls back to baseMesh positions).
    // compressionFactor: restVolume = currentVolume * factor (1 = identity, 2 = body is half inflated).
    SoftBody(Mesh baseMesh, const std::vector<SpringDef>& springDefs, const std::vector<StrutDef>& strutDefs, ConstraintMode mode, const std::vector<bool>& staticPoints = {}, const Mesh* restMesh = nullptr, float compressionFactor = 1.f);
    void SolveSpring(Spring& spring);
    void ClampSpringForce(Spring& spring);
    void SolveStrut(Strut& strut);
    void ApplyShapeMatching(float stiffness, float dt);
    void ApplyVolumePreservation(float stiffness, float dt);
    void AddStrut(int a, int b);
    void AddMuscle(int a, int b, float stiffness, float damping, float muscleForce, int group);
    void SetMuscleGroup(int group, bool active);
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

    ConstraintMode GetConstraintMode() const { return constraintMode; }

    std::vector<Point>& GetPoints() { return points; }
    const std::vector<Point>& GetPoints() const { return points; }
    const std::vector<Spring>& GetSprings() const { return springs; }
    const std::vector<Strut>&  GetStruts()  const { return struts; }
    const std::vector<Collision::ConvexCluster>& GetClusters() const { return clusters; }
    const std::vector<Collision::DebugContact>& GetDebugContacts() const { return debugContacts; }

private:
    void ApplyMuscleForces();

    std::vector<Spring> springs;
    std::vector<Strut> struts;
    std::vector<Point> points;
    float* restPositions;
    std::set<int> activeMuscleGroups;

    ConstraintMode constraintMode;
    float restVolume = 0.f;
    std::vector<Vector3> gradientBuffer;

    // Collision data
    bool hasGroundPlane = false;
    Collision::GroundPlane groundPlane{{0,1,0}, 0.0f};
    std::vector<Collision::ConvexShape> staticColliders;
    std::vector<Collision::ConvexCluster> clusters;
    std::vector<Collision::DebugContact> debugContacts;
};
