#include "CaveGen.h"
#include <raylib.h>
#include "MathUtils.h"
#include "PerlinNoise.hpp"
#include <algorithm>
#include <cstring>

#include "Renderer.h"

void CaveGen::GenerateMesh(Shader litShader){

    constexpr float sizeX = 1000.f;
    constexpr float sizeY = 1000.f;
    constexpr int subdivX = 200;
    constexpr int subdivY = 200;
    const Vector3 planeNormal = {0, -1, 0};

    ceiling = {0};
    int indexCount = 0;
    GeneratePlane(sizeX, sizeY, subdivX, subdivY, planeNormal,&ceiling.vertices, &ceiling.normals, &ceiling.indices,&ceiling.vertexCount, &indexCount);
    ceiling.triangleCount = indexCount / 3;

    // Initial displacement through Perlin Noise
    const float maxDisplacement = std::max(sizeX, sizeY) / 10.f;
    constexpr float noiseScale = 0.01f;

    siv::PerlinNoise noise;

    for (int i=0;i<ceiling.vertexCount;i++) {
        float& vx = ceiling.vertices[i*3+0];
        float& vy = ceiling.vertices[i*3+1];
        float& vz = ceiling.vertices[i*3+2];

        // FBM: 5 octaves to break parallel ridges and create isolated local minima
        float disp = 0.f, amp = 1.f, freq = noiseScale, totalAmp = 0.f;
        for (int o = 0; o < 5; o++) {
            disp     += (float)noise.noise2D_01(vx*freq, vz*freq) * amp;
            totalAmp += amp;
            amp  *= 0.5f;
            freq *= 2.f;
        }
        disp = (disp / totalAmp) * maxDisplacement;

        vx += planeNormal.x*disp;
        vy += planeNormal.y*disp;
        vz += planeNormal.z*disp;
    }

    // Build adjacency map
    adjacency.assign(ceiling.vertexCount, {});
    constexpr int triangleCount = subdivX*subdivY*2;
    for (int t=0;t<triangleCount;t++){
        int i0 = ceiling.indices[t*3+0];
        int i1 = ceiling.indices[t*3+1];
        int i2 = ceiling.indices[t*3+2];
        adjacency[i0].push_back(i1); adjacency[i0].push_back(i2);
        adjacency[i1].push_back(i0); adjacency[i1].push_back(i2);
        adjacency[i2].push_back(i0); adjacency[i2].push_back(i1);
    }
    // Deduplicate each neighbor list
    for (auto& neighbors : adjacency) {
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    }

    memset(ceiling.normals, 0, ceiling.vertexCount * 3 * sizeof(float));

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
    
    UploadMesh(&ceiling, false);
    model = LoadModelFromMesh(ceiling);
    model.materials[0].shader = litShader;
    AddToLit(EventBus::Get());
}

void CaveGen::Draw(){
    DrawModel(model,{0,200,0},1,WHITE);
}

void CaveGen::WaterDeposition(){
    const float waterScaleFactor = 2.f;
    const int   spreadHops       = 8;
    const float spreadFalloff    = 0.4f;

    float* verts = model.meshes[0].vertices;
    SortHeight();

    // Accumulate water flow from high to low
    std::vector<float> water(ceiling.vertexCount, 1.f);
    std::vector<float> slopeSum(ceiling.vertexCount, 0.f);
    std::vector<std::vector<std::pair<int,float>>> outflow(ceiling.vertexCount);
    for (int index : orderedVertices) {
        float hy = verts[index*3+1];
        for (int nb : adjacency[index]) {
            float slope = hy - verts[nb*3+1];
            if (slope > 0.f) {
                slopeSum[index] += slope;
                outflow[index].push_back({nb, slope});
            }
        }
    }
    for (int index : orderedVertices) {
        if (slopeSum[index] < 1e-6f) continue;
        for (auto [nb, slope] : outflow[index])
            water[nb] += water[index] * (slope / slopeSum[index]);
    }

    // Spread water outward hop by hop with falloff.
    // Each vertex takes the max of itself and decayed neighbors,
    // creating a ring-shaped displacement zone around each convergence point.
    std::vector<float> spread = water;
    std::vector<float> next(ceiling.vertexCount);
    for (int hop = 0; hop < spreadHops; hop++) {
        for (int i = 0; i < ceiling.vertexCount; i++) {
            next[i] = spread[i];
            for (int nb : adjacency[i])
                next[i] = std::max(next[i], spread[nb] * spreadFalloff);
        }
        std::swap(spread, next);
    }

    // Only displace above the baseline (1.0 = every vertex's starting water).
    // Flat areas where nothing converged stay at spread ≈ 1, so displacement ≈ 0.
    for (int i = 0; i < ceiling.vertexCount; i++) {
        float excess = std::max(0.f, spread[i] - 1.f);
        float displacement = powf(excess, 1.f/3.f) * waterScaleFactor;
        float ny = model.meshes[0].normals[3*i+1];
        float cosTheta = std::max(0.f, -ny);
        verts[i*3+1] -= displacement * cosTheta;
    }

    UpdateMeshBuffer(model.meshes[0], 0, verts, ceiling.vertexCount * 3 * sizeof(float), 0);
}

void CaveGen::SortHeight(){
    // Build ordered vertex list (decreasing Y) from current mesh state
    const float* verts = model.meshes[0].vertices;
    orderedVertices.resize(ceiling.vertexCount);
    for (int i=0;i<ceiling.vertexCount;i++) orderedVertices[i] = i;
    std::sort(orderedVertices.begin(), orderedVertices.end(), [&](int a, int b){
        return verts[a*3+1] > verts[b*3+1];
    });
}