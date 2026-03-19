#include "Slime.h"
#include "MathUtils.h"

Slime::Slime(){

    Mesh baseMesh = { 0 };
    GenerateUvSphere(10, 10, 50.0f,&baseMesh.vertices,&baseMesh.normals,&baseMesh.indices,&baseMesh.vertexCount,&baseMesh.triangleCount);
    baseMesh.triangleCount /= 3;
    UploadMesh(&baseMesh, false);

    core = SoftBody(baseMesh,{{0,baseMesh.vertexCount-1,100.f,100.f,0.5f,20.f,1}},{},SoftBody::ConstraintMode::VolumePreservation,{},{},1.f);

}

void Slime::Update(const std::vector<Input::InputAction>& actions,float dt, Vector3 force){
    
    core.Update(dt, force);

    // Compute core centroid (pseudo center)
    const auto& pts = core.GetPoints();
    Vector3 center = { 0, 0, 0 };
    for (const auto& p : pts) center = center + p.position;
    center = center * (1.0f / (float)pts.size());

    // Pin skin's first vertex (static anchor) to the core center
    skin.model.meshes[0].vertices[0] = center.x;
    skin.model.meshes[0].vertices[1] = center.y;
    skin.model.meshes[0].vertices[2] = center.z;
}

