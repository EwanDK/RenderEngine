#include "SoftBody.h"

#include "raymath.h"
#include "MathUtils.h"

SoftBody::SoftBody(){
    
    Mesh restMesh = { 0 };

    GenerateUvSphere(10, 10, 100.0f,&restMesh.vertices,&restMesh.normals,&restMesh.indices,&restMesh.vertexCount,&restMesh.triangleCount);
    restMesh.triangleCount /= 3;
    //UploadMesh(&restMesh, false);
    
    Mesh baseMesh = { 0 };
    GenerateUvSphere(10, 10, 50.0f,&baseMesh.vertices,&baseMesh.normals,&baseMesh.indices,&baseMesh.vertexCount,&baseMesh.triangleCount);
    baseMesh.triangleCount /= 3;
    UploadMesh(&baseMesh, true);
    model = LoadModelFromMesh(baseMesh);

    for(int i=0;i<baseMesh.vertexCount;i++){
        points.emplace_back(Point{{baseMesh.vertices[i*3],baseMesh.vertices[i*3+1],baseMesh.vertices[i*3+2]},Vector3(),Vector3(),1.f,1.f,i});
    }

    restPositions = restMesh.vertices;
    unsigned short* restTriangles = restMesh.indices;

    for(int i=0;i<restMesh.triangleCount*3;i+=3){
        Spring s1;
        s1.points[0]=&points[i];
        s1.points[1]=&points[i+1];
        s1.baseDistance = Vector3Distance(Vector3{restPositions[restTriangles[i]*3],restPositions[restTriangles[i]*3+1],restPositions[restTriangles[i]*3+2]},Vector3{restPositions[restTriangles[i+1]*3],restPositions[restTriangles[i+1]*3+1],restPositions[restTriangles[i+1]*3+2]});
        s1.stiffness=2.f;
        s1.damping=.5f;
        springs.emplace_back(s1);

        Spring s2;
        s2.points[0]=&points[i];
        s2.points[1]=&points[i+2];
        s2.baseDistance = Vector3Distance(Vector3{restPositions[restTriangles[i]*3],restPositions[restTriangles[i]*3+1],restPositions[restTriangles[i]*3+2]},Vector3{restPositions[restTriangles[i+2]*3],restPositions[restTriangles[i+2]*3+1],restPositions[restTriangles[i+2]*3+2]});
        s2.stiffness=2.f;
        s2.damping=.5f;
        springs.emplace_back(s2);

        Spring s3;
        s3.points[0]=&points[i+2];
        s3.points[1]=&points[i+1];
        s3.baseDistance = Vector3Distance(Vector3{restPositions[restTriangles[i+2]*3],restPositions[restTriangles[i+2]*3+1],restPositions[restTriangles[i+2]*3+2]},Vector3{restPositions[restTriangles[i+1]*3],restPositions[restTriangles[i+1]*3+1],restPositions[restTriangles[i+1]*3+2]});
        s3.stiffness=2.f;
        s3.damping=.5f;
        springs.emplace_back(s3);
    }
    



    //UnloadMesh(restMesh);
}

void SoftBody::SolveSpring(Spring spring){
    Vector3 ab = Vector3(spring.points[1]->position-spring.points[0]->position);
    Vector3 abNorm = ab;
    Vector3Normalize(abNorm);
    
    float springForce = (Vector3Length(ab) - spring.baseDistance) * spring.stiffness;

    Vector3 velDiff = spring.points[1]->speed-spring.points[0]->speed;

    float dot =Vector3DotProduct(abNorm,velDiff);

    float dampingForce = dot*spring.damping;

    float totalForce = springForce + dampingForce;

    Vector3 aForce = abNorm*totalForce;
    Vector3 baNorm = ab*-1.f;
    Vector3Normalize(baNorm);
    Vector3 bForce = baNorm*totalForce;
    // >0 attraction <0 repulsion

    spring.points[0]->force+=aForce;
    spring.points[1]->force+=bForce;
}

void SoftBody::ClampSpringForce(Spring& spring) {
    Vector3 delta = spring.points[1]->position - spring.points[0]->position;
    float dist = Vector3Length(delta);

    float minDist = 2.0f * spring.points[0]->radius;
    float maxDist = 2.0f * spring.baseDistance;

    if (dist < minDist || dist > maxDist) {
        Vector3Normalize(delta);

        float clampedDist = Clamp(dist, minDist, maxDist);
        Vector3 target = spring.points[0]->position + delta * clampedDist;

        // Correction vector (split between points, inversely to mass)
        float totalMass = spring.points[0]->mass + spring.points[1]->mass;
        float ratioA = spring.points[1]->mass / totalMass;
        float ratioB = spring.points[0]->mass / totalMass;

        Vector3 correction = (target - spring.points[1]->position);

        spring.points[0]->position-=correction*ratioA;
        spring.points[0]->position-=correction*ratioB;

        // Optional: zero spring force to prevent snapback
        spring.points[0]->force = Vector3();
        spring.points[1]->force = Vector3();
    }
}

void SoftBody::ApplyShapeMatching(float stiffness)
{
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

    // Pull current points toward rotated rest pose
    for (size_t i = 0; i < points.size(); ++i) {
        Vector3 q = Vector3(restPositions[i*3],restPositions[i*3+1],restPositions[i*3+2]) - centerRest;
        Vector3 goal = centerNow + R.getColumn(0) * q.x + R.getColumn(1) * q.y + R.getColumn(2) * q.z;
        Vector3 correction = (goal - points[i].position) * stiffness;
        points[i].position+=correction;
    }
}



void SoftBody::Solve(float dt){
    for(int i=0;i<points.size();i++){
        //if(!pointMassGroundCollisions(&points[i]))points[i].force=Vector3::unitZ*-9*points[i].mass; //This line for gravity
        points[i].force=Vector3(); //This line for no gravity 
    }
    for(int i=0;i<springs.size();i++){
        SolveSpring(springs[i]);
    }
    for(int i=0;i<springs.size();i++){
        ClampSpringForce(springs[i]);
    }

    ApplyShapeMatching(0.0001f);
    
    const float maxSpeed = 100.0f;
    const float groundFriction=.85f;
    const float lowSpeedThreshold=2.f;
    
    for(int i=0;i<points.size();i++){
        // Zero low force
        if(Vector3Length(points[i].force)<lowSpeedThreshold)points[i].force=Vector3();
        
        points[i].speed+=points[i].force*dt * (1/points[i].mass);
        
        if (Vector3Length(points[i].speed) > maxSpeed) {
            Vector3Normalize(points[i].speed);
            points[i].speed *= maxSpeed;
        }
        Vector3 tSpeed = points[i].speed*dt;
        points[i].position+=tSpeed;
        // After movement, apply ground friction *if the point is on the ground*
        //auto& groundPlane = getGame().getPlanes()[0];
        //const AABB& planeBox = groundPlane->getBox()->getWorldBox();

        // Check if we're within "contact" range (z within 1 unit of planeBox.max.z)
        /*if (points[i].position.z - 5.0f <= planeBox.max.z + 0.1f) {
            points[i].speed.x *= groundFriction;
            points[i].speed.y *= groundFriction;
        }*/
    }
    //float tmp = (points[0].position-points[2].position).length();
    //std::cout<<tmp<<std::endl;
}
