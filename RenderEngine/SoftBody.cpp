#include "SoftBody.h"

struct Spring{
    unsigned short points[2];
    float baseDistance;
    float stiffness;
    float damping;
    int virtualDistance=0;
};

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
    

    restPositions = restMesh.vertices;
    unsigned short* restTriangles = restMesh.indices;

    for(int i=0;i<restMesh.triangleCount*3;i+=3){
        Spring s1;
        s1.points[0]=i;
        s1.points[1]=i+1;
        s1.baseDistance = Vector3Distance(Vector3{restPositions[restTriangles[i]*3],restPositions[restTriangles[i]*3+1],restPositions[restTriangles[i]*3+2]},Vector3{restPositions[restTriangles[i+1]*3],restPositions[restTriangles[i+1]*3+1],restPositions[restTriangles[i+1]*3+2]});
        s1.stiffness=2.f;
        s1.damping=.5f;
        springs.emplace_back(s1);

        Spring s2;
        s2.points[0]=i;
        s2.points[1]=i+2;
        s2.baseDistance = Vector3Distance(Vector3{restPositions[restTriangles[i]*3],restPositions[restTriangles[i]*3+1],restPositions[restTriangles[i]*3+2]},Vector3{restPositions[restTriangles[i+2]*3],restPositions[restTriangles[i+2]*3+1],restPositions[restTriangles[i+2]*3+2]});
        s2.stiffness=2.f;
        s2.damping=.5f;
        springs.emplace_back(s2);

        Spring s3;
        s3.points[0]=i+2;
        s3.points[1]=i+1;
        s3.baseDistance = Vector3Distance(Vector3{restPositions[restTriangles[i+2]*3],restPositions[restTriangles[i+2]*3+1],restPositions[restTriangles[i+2]*3+2]},Vector3{restPositions[restTriangles[i+1]*3],restPositions[restTriangles[i+1]*3+1],restPositions[restTriangles[i+1]*3+2]});
        s3.stiffness=2.f;
        s3.damping=.5f;
        springs.emplace_back(s3);
    }
    



    //UnloadMesh(restMesh);
}
