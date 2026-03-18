#include "CaveGen.h"
#include <raylib.h>
#include "MathUtils.h"
#include "PerlinNoise.hpp"
#include <algorithm>
#include <cstring>

void CaveGen::GenerateMesh(){

    constexpr float sizeX = 100.f;
    constexpr float sizeY = 100.f;
    constexpr int subdivX = 10;
    constexpr int subdivY = 10;
    const Vector3 planeNormal = {0, -1, 0};

    Mesh ceiling = {0};
    GeneratePlane(sizeX, sizeY, subdivX, subdivY, planeNormal,&ceiling.vertices, &ceiling.normals, &ceiling.indices,&ceiling.vertexCount, &ceiling.triangleCount);

    // Initial displacement through Perlin Noise
    const float maxDisplacement = std::max(sizeX, sizeY) / 10.f;
    constexpr float noiseScale = 0.05f;

    siv::PerlinNoise noise;

    for (int i=0;i<ceiling.vertexCount;i++) {
        float& vx = ceiling.vertices[i*3+0];
        float& vy = ceiling.vertices[i*3+1];
        float& vz = ceiling.vertices[i*3+2];

        float disp = (float)noise.noise2D_01(vx*noiseScale,vz*noiseScale)*maxDisplacement;
        vx += planeNormal.x*disp;
        vy += planeNormal.y*disp;
        vz += planeNormal.z*disp;
    }
    
    memset(ceiling.normals, 0, ceiling.vertexCount * 3 * sizeof(float));

    constexpr int triangleCount = subdivX*subdivY*2;
    for (int t=0;t<triangleCount;t++){
        int i0 = ceiling.indices[t*3+0];
        int i1 = ceiling.indices[t*3+1];
        int i2 = ceiling.indices[t*3+2];

        float ax = ceiling.vertices[i0*3], ay = ceiling.vertices[i0*3+1], az = ceiling.vertices[i0*3+2];
        float bx = ceiling.vertices[i1*3], by = ceiling.vertices[i1*3+1], bz = ceiling.vertices[i1*3+2];
        float cx = ceiling.vertices[i2*3], cy = ceiling.vertices[i2*3+1], cz = ceiling.vertices[i2*3+2];

        float e1x = bx-ax, e1y = by-ay, e1z = bz-az;
        float e2x = cx-ax, e2y = cy-ay, e2z = cz-az;

        float fnx = e1y*e2z - e1z*e2y;
        float fny = e1z*e2x - e1x*e2z;
        float fnz = e1x*e2y - e1y*e2x;

        ceiling.normals[i0*3] += fnx; ceiling.normals[i0*3+1] += fny; ceiling.normals[i0*3+2] += fnz;
        ceiling.normals[i1*3] += fnx; ceiling.normals[i1*3+1] += fny; ceiling.normals[i1*3+2] += fnz;
        ceiling.normals[i2*3] += fnx; ceiling.normals[i2*3+1] += fny; ceiling.normals[i2*3+2] += fnz;
    }

    for (int i=0;i<ceiling.vertexCount;i++) {
        float nx = ceiling.normals[i*3], ny = ceiling.normals[i*3+1], nz = ceiling.normals[i*3+2];
        float len = sqrtf(nx*nx + ny*ny + nz*nz);
        if (len > 1e-6f) {
            ceiling.normals[i*3] = nx/len;
            ceiling.normals[i*3+1] = ny/len;
            ceiling.normals[i*3+2] = nz/len;
        }
    }
}
