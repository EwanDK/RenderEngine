#include "SoftBody.h"

#include <iostream>
#include <set>

#include "raymath.h"
#include "MathUtils.h"

SoftBody::SoftBody(){
    
    Mesh restMesh = { 0 };

    GenerateUvSphere(10, 10, 60.0f,&restMesh.vertices,&restMesh.normals,&restMesh.indices,&restMesh.vertexCount,&restMesh.triangleCount);
    restMesh.triangleCount /= 3;
    
    Mesh baseMesh = { 0 };
    GenerateUvSphere(10, 10, 50.0f,&baseMesh.vertices,&baseMesh.normals,&baseMesh.indices,&baseMesh.vertexCount,&baseMesh.triangleCount);
    baseMesh.triangleCount /= 3;
    UploadMesh(&baseMesh, false);
    model = LoadModelFromMesh(baseMesh);

    for(int i=0;i<baseMesh.vertexCount;i++){
        points.emplace_back(Point{{baseMesh.vertices[i*3],baseMesh.vertices[i*3+1],baseMesh.vertices[i*3+2]},Vector3(),Vector3(),1.f,1.f,i});
    }

    restPositions = restMesh.vertices;
    unsigned short* restTriangles = restMesh.indices;
    std::set<std::pair<int,int>> usedEdges;

    for (int i = 0; i < restMesh.triangleCount; i++)
    {
        int i0 = restTriangles[i*3];
        int i1 = restTriangles[i*3+1];
        int i2 = restTriangles[i*3+2];

        auto addSpring = [&](int a, int b){
            int lo = std::min(a, b);
            int hi = std::max(a, b);

            if (usedEdges.insert({lo, hi}).second){
                Spring s;
                s.points[0] = &points[lo];
                s.points[1] = &points[hi];

                s.baseDistance = Vector3Distance(
                    Vector3{restPositions[lo*3], restPositions[lo*3+1], restPositions[lo*3+2]},
                    Vector3{restPositions[hi*3], restPositions[hi*3+1], restPositions[hi*3+2]}
                );

                s.stiffness = 100.f;
                s.damping   = 0.5f;
                springs.emplace_back(s);
            }
        };

        addSpring(i0, i1);
        addSpring(i0, i2);
        addSpring(i1, i2);
        
    }
    



    // Build convex clusters for broadphase collision
    BuildClusters(6);

    //UnloadMesh(restMesh);
}

void SoftBody::SolveSpring(Spring& spring){
    Vector3 ab = Vector3(spring.points[1]->position-spring.points[0]->position);
    float len = Vector3Length(ab);
    if (len < 1e-6f) return;
    Vector3 abNorm = Vector3Normalize(ab);
    
    float springForce = (len - spring.baseDistance) * spring.stiffness;

    Vector3 velDiff = spring.points[1]->speed-spring.points[0]->speed;

    float dot =Vector3DotProduct(abNorm,velDiff);

    float dampingForce = dot*spring.damping;

    float totalForce = springForce + dampingForce;

    Vector3 aForce = abNorm*totalForce;
    Vector3 baNorm = ab*-1.f;
    baNorm=Vector3Normalize(baNorm);
    Vector3 bForce = baNorm*totalForce;
    // >0 attraction <0 repulsion

    spring.points[0]->force+=aForce;
    spring.points[1]->force+=bForce;
}

void SoftBody::ClampSpringForce(Spring& spring) {
    Vector3 delta = spring.points[1]->position - spring.points[0]->position;
    float dist = Vector3Length(delta);
    if (dist < 1e-6f) return;

    float minDist = spring.baseDistance * 0.5f;
    float maxDist = spring.baseDistance * 1.5f;

    if (dist < minDist || dist > maxDist) {
        Vector3 dir = delta / dist;
        float clampedDist = Clamp(dist, minDist, maxDist);
        float correction = clampedDist - dist;

        float wA = spring.points[0]->isStatic ? 0.f : 1.f / spring.points[0]->mass;
        float wB = spring.points[1]->isStatic ? 0.f : 1.f / spring.points[1]->mass;
        float wSum = wA + wB;
        if (wSum < 1e-10f) return;

        spring.points[0]->position -= dir * correction * (wA / wSum);
        spring.points[1]->position += dir * correction * (wB / wSum);

        if (!spring.points[0]->isStatic) spring.points[0]->force = Vector3();
        if (!spring.points[1]->isStatic) spring.points[1]->force = Vector3();
    }
}

void SoftBody::SolveStrut(Strut& strut) {
    Vector3 delta = strut.b->position - strut.a->position;
    float len = Vector3Length(delta);
    if (len < 1e-6f) return;

    float err = (len - strut.distance) / len; // signed: >0 too far, <0 too close

    float wA = strut.a->isStatic ? 0.f : 1.f / strut.a->mass;
    float wB = strut.b->isStatic ? 0.f : 1.f / strut.b->mass;
    float wSum = wA + wB;
    if (wSum < 1e-10f) return;

    // Positional correction
    Vector3 correction = delta * err;
    strut.a->position += correction * (wA / wSum);
    strut.b->position -= correction * (wB / wSum);

    // Kill relative velocity along the constraint axis (prevents pumping)
    Vector3 n = delta * (1.f / len);
    float relVel = Vector3DotProduct(strut.b->speed - strut.a->speed, n);
    Vector3 velCorr = n * relVel;
    strut.a->speed += velCorr * (wA / wSum);
    strut.b->speed -= velCorr * (wB / wSum);
}

void SoftBody::AddStrut(int a, int b) {
    Strut s;
    s.a = &points[a];
    s.b = &points[b];
    s.distance = Vector3Distance(points[a].position, points[b].position);
    struts.emplace_back(s);
}

void SoftBody::AddMuscle(int a, int b, float stiffness, float damping, float muscleForce, int group) {
    Spring s;
    s.points[0]   = &points[a];
    s.points[1]   = &points[b];
    s.baseDistance = Vector3Distance(points[a].position, points[b].position);
    s.stiffness   = stiffness;
    s.damping     = damping;
    s.muscleForce = muscleForce;
    s.muscleGroup = group;
    springs.emplace_back(s);
}

void SoftBody::SetMuscleGroup(int group, bool active) {
    if (active) activeMuscleGroups.insert(group);
    else        activeMuscleGroups.erase(group);
}

void SoftBody::ApplyMuscleForces() {
    for (auto& s : springs) {
        if (s.muscleGroup < 0) continue;
        if (activeMuscleGroups.find(s.muscleGroup) == activeMuscleGroups.end()) continue;

        Vector3 ab = s.points[1]->position - s.points[0]->position;
        float len = Vector3Length(ab);
        if (len < 1e-6f) continue;
        Vector3 dir = ab * (1.f / len);

        // Positive muscleForce = compression (pull endpoints toward each other)
        s.points[0]->force += dir *  s.muscleForce;
        s.points[1]->force += dir * -s.muscleForce;
    }
}

void SoftBody::ApplyShapeMatching(float stiffness, float dt){
    if (points.empty() || model.meshes[0].vertexCount != points.size())return;

    // Compute current and rest centers of mass
    Vector3 centerNow=Vector3();
    Vector3 centerRest=Vector3();
    for (size_t i = 0; i < points.size(); ++i) {
        centerNow += points[i].position;
        centerRest += Vector3(restPositions[i*3],restPositions[i*3+1],restPositions[i*3+2]);
    }
    centerNow *= 1.f/static_cast<float>(points.size());
    centerRest *= 1.f/static_cast<float>(model.meshes[0].vertexCount);

    // Compute covariance matrix A = Σ(p_i - c_p)(q_i - c_q)^T
    Matrix3 A;
    A.zero();
    for (size_t i = 0; i < points.size(); ++i) {
        Vector3 p = points[i].position - centerNow;
        Vector3 q = Vector3(restPositions[i*3],restPositions[i*3+1],restPositions[i*3+2]) - centerRest;
        A += Matrix3::outerProduct(p, q); // A += pqᵗ
    }

    // Compute optimal rotation matrix
    Matrix3 R = A.orthonormalized();

    float effectiveStiffness = 1.0f - pow(1.0f - stiffness, dt * 60.0f);
    // Pull current points toward rotated rest pose
    for (size_t i = 0; i < points.size(); ++i) {
        Vector3 q = Vector3(restPositions[i*3],restPositions[i*3+1],restPositions[i*3+2]) - centerRest;
        Vector3 goal = centerNow + R.getColumn(0) * q.x + R.getColumn(1) * q.y + R.getColumn(2) * q.z;
        Vector3 correction = (goal - points[i].position) * effectiveStiffness;
        points[i].position+=correction;
    }
}



void SoftBody::Solve(float dt){
    // Clamp dt to prevent tunneling on frame stutters
    dt = std::min(dt, 1.0f / 30.0f);

    // 1. Zero forces
    for(int i=0;i<points.size();i++){
        points[i].force = Vector3();
    }

    // 2. Spring forces
    for(int i=0;i<springs.size();i++){
        SolveSpring(springs[i]);
    }

    // 2b. Muscle activation forces
    ApplyMuscleForces();

    // 3. Integrate velocity
    const float damping = 0.98f; // Per-frame damping at 60fps baseline
    const float dampingFactor = powf(damping, dt * 60.0f); // Frame-rate independent
    const Vector3 gravity = {0.0f, -9.81f, 0.0f};
    for(auto& p : points){
        if (p.isStatic) continue;
        p.speed += p.force * dt * (1.0f / p.mass); // Spring forces only
        p.speed *= dampingFactor;                    // Damp spring oscillation
        p.speed += gravity * dt;                     // Gravity added undamped
    }

    // 4. Integrate position
    for(auto& p : points){
        if (p.isStatic) continue;
        p.position += p.speed * dt;
    }

    // 5. Position-based constraints (iterate multiple times for stability)
    debugContacts.clear();
    const int constraintIterations = 3;
    for(int iter = 0; iter < constraintIterations; iter++){
        bool lastIter = (iter == constraintIterations - 1);

        // Spring length constraints
        for(auto& spring : springs){
            ClampSpringForce(spring);
        }

        // Rigid struts (exact distance constraints)
        for(auto& strut : struts){
            SolveStrut(strut);
        }

        // Shape matching (with dt scaling)
        ApplyShapeMatching(0.01f, dt);

        // Ground collision
        if (hasGroundPlane) {
            Collision::ResolveGroundCollision(points, groundPlane, 0.3f, 0.85f);

            // Capture ground contacts on last iteration
            if (lastIter) {
                for (auto& p : points) {
                    float signedDist = Vector3DotProduct(groundPlane.normal, p.position) - groundPlane.height;
                    if (signedDist < 2.0f) {
                        debugContacts.push_back({p.position, groundPlane.normal, std::max(0.0f, 2.0f - signedDist)});
                    }
                }
            }
        }

        // Static colliders (cluster broadphase + per-particle narrowphase)
        UpdateClusterHulls();
        for (auto& collider : staticColliders) {
            for (auto& cluster : clusters) {
                if (cluster.shape.count > 0 && Collision::Intersect(cluster.shape, collider)) {
                    Collision::ResolveSoftVsStaticSubset(
                        points, cluster.pointIndices, collider, 0.3f, 0.5f,
                        lastIter ? &debugContacts : nullptr);
                }
            }
        }
    }
}

void SoftBody::SetGroundPlane(const Collision::GroundPlane& ground) {
    groundPlane = ground;
    hasGroundPlane = true;
}

void SoftBody::AddStaticCollider(const Collision::ConvexShape& shape) {
    staticColliders.push_back(shape);
}

void SoftBody::ClearStaticColliders() {
    staticColliders.clear();
}

void SoftBody::BuildClusters(int k) {
    clusters = Collision::BuildClusters(restPositions, static_cast<int>(points.size()), k);
    Collision::UpdateClusterHulls(clusters, points);
}

void SoftBody::UpdateClusterHulls() {
    Collision::UpdateClusterHulls(clusters, points);
}

void SoftBody::Translate(Vector3 offset) {
    for (size_t i = 0; i < points.size(); i++) {
        points[i].position += offset;
    }
    // Also shift rest positions so shape matching targets the new location
    for (int i = 0; i < (int)points.size(); i++) {
        restPositions[i * 3]     += offset.x;
        restPositions[i * 3 + 1] += offset.y;
        restPositions[i * 3 + 2] += offset.z;
    }
}

void SoftBody::Draw(){
    for (int i = 0; i < model.meshes[0].vertexCount; i++)
    {
        model.meshes[0].vertices[i*3] = points[i].position.x;
        model.meshes[0].vertices[i*3+1] = points[i].position.y;
        model.meshes[0].vertices[i*3+2] = points[i].position.z;
    }

    RecomputeNormals(model.meshes[0]);

    UpdateMeshBuffer(model.meshes[0],0,model.meshes[0].vertices,model.meshes[0].vertexCount*3*sizeof(float),0);
    UpdateMeshBuffer(model.meshes[0], 2, model.meshes[0].normals,  model.meshes[0].vertexCount * 3 * sizeof(float), 0);

    DrawModelWires(model,Vector3(0.f,0.f,0.f),1,BLUE);
}

void SoftBody::RecomputeNormals(Mesh& mesh)
{
    // Clear normals
    for (int i = 0; i < mesh.vertexCount * 3; i++)
    {
        mesh.normals[i] = 0.0f;
    }

    // For each triangle
    for (int i = 0; i < mesh.triangleCount; i++)
    {
        int i0 = mesh.indices[i*3];
        int i1 = mesh.indices[i*3+1];
        int i2 = mesh.indices[i*3+2];

        Vector3 v0 = {mesh.vertices[i0*3],mesh.vertices[i0*3+1],mesh.vertices[i0*3+2]};
        Vector3 v1 = {mesh.vertices[i1*3],mesh.vertices[i1*3+1],mesh.vertices[i1*3+2]};
        Vector3 v2 = {mesh.vertices[i2*3],mesh.vertices[i2*3+1],mesh.vertices[i2*3+2]};

        // Compute face normal
        Vector3 e1 = Vector3Subtract(v1, v0);
        Vector3 e2 = Vector3Subtract(v2, v0);
        Vector3 faceNormal = Vector3CrossProduct(e1, e2);

        // Accumulate into vertex normals
        mesh.normals[i0*3] += faceNormal.x;
        mesh.normals[i0*3+1] += faceNormal.y;
        mesh.normals[i0*3+2] += faceNormal.z;

        mesh.normals[i1*3] += faceNormal.x;
        mesh.normals[i1*3+1] += faceNormal.y;
        mesh.normals[i1*3+2] += faceNormal.z;

        mesh.normals[i2*3] += faceNormal.x;
        mesh.normals[i2*3+1] += faceNormal.y;
        mesh.normals[i2*3+2] += faceNormal.z;
    }

    // Normalize all normals
    for (int i = 0; i < mesh.vertexCount; i++)
    {
        Vector3 n = {mesh.normals[i*3],mesh.normals[i*3+1],mesh.normals[i*3+2]};

        n = Vector3Normalize(n);

        mesh.normals[i*3] = n.x;
        mesh.normals[i*3+1] = n.y;
        mesh.normals[i*3+2] = n.z;
    }
}

void SoftBody::Update(float dt){
    Solve(dt);
    Draw();
}