#include "CaveGen.h"
#include <raylib.h>
#include "MathUtils.h"

void CaveGen::GenerateMesh(){

    Mesh ceiling = {0};
    GeneratePlane(100.f,100.f,10,10,{0,1,0},&ceiling.vertices,&ceiling.normals,&ceiling.indices,&ceiling.vertexCount,&ceiling.triangleCount);
    
}
