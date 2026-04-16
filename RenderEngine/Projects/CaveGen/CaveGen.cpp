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
    constexpr int subdivX = 50;
    constexpr int subdivY = 50;
    constexpr float noiseScale = 0.01f;
    const float maxDisplacement = std::max(sizeX, sizeY) / 10.f;

    BuildMeshSection(ceiling, ceilingModel, ceilingAdjacency, {0, -1, 0}, sizeX, sizeY, subdivX, subdivY, noiseScale, maxDisplacement, litShader);
    BuildMeshSection(floor, floorModel, floorAdjacency, {0, 1, 0}, sizeX, sizeY, subdivX, subdivY, noiseScale, maxDisplacement, litShader);

    AddToLit(EventBus::Get());
}

void CaveGen::BuildMeshSection(Mesh& outMesh, Model& outModel, std::vector<std::vector<int>>& outAdjacency, Vector3 normal, float sizeX, float sizeY, int subdivX, int subdivY, float noiseScale, float maxDisplacement, Shader litShader){
    outMesh = {0};
    int indexCount = 0;
    GeneratePlane(sizeX, sizeY, subdivX, subdivY, normal, &outMesh.vertices, &outMesh.normals, &outMesh.indices, &outMesh.vertexCount, &indexCount);
    outMesh.triangleCount = indexCount / 3;

    // Initial displacement through Perlin Noise (FBM: 5 octaves)
    siv::PerlinNoise noise;
    for (int i = 0; i < outMesh.vertexCount; i++){
        float& vx = outMesh.vertices[i * 3 + 0];
        float& vy = outMesh.vertices[i * 3 + 1];
        float& vz = outMesh.vertices[i * 3 + 2];

        float disp = 0.f, amp = 1.f, freq = noiseScale, totalAmp = 0.f;
        for (int o = 0; o < 5; o++){
            disp += (float)noise.noise2D_01(vx * freq, vz * freq) * amp;
            totalAmp += amp;
            amp *= 0.5f;
            freq *= 2.f;
        }
        disp = (disp / totalAmp) * maxDisplacement;

        vx += normal.x * disp;
        vy += normal.y * disp;
        vz += normal.z * disp;
    }

    // Build adjacency map
    int triangleCount = subdivX * subdivY * 2;
    outAdjacency.assign(outMesh.vertexCount, {});
    for (int t = 0; t < triangleCount; t++){
        int i0 = outMesh.indices[t * 3 + 0];
        int i1 = outMesh.indices[t * 3 + 1];
        int i2 = outMesh.indices[t * 3 + 2];
        outAdjacency[i0].push_back(i1);
        outAdjacency[i0].push_back(i2);
        outAdjacency[i1].push_back(i0);
        outAdjacency[i1].push_back(i2);
        outAdjacency[i2].push_back(i0);
        outAdjacency[i2].push_back(i1);
    }
    for (auto& neighbors : outAdjacency){
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    }

    // Recompute normals
    memset(outMesh.normals, 0, outMesh.vertexCount * 3 * sizeof(float));
    for (int t = 0; t < triangleCount; t++){
        int i0 = outMesh.indices[t * 3 + 0];
        int i1 = outMesh.indices[t * 3 + 1];
        int i2 = outMesh.indices[t * 3 + 2];

        float ax = outMesh.vertices[i0 * 3], ay = outMesh.vertices[i0 * 3 + 1], az = outMesh.vertices[i0 * 3 + 2];
        float bx = outMesh.vertices[i1 * 3], by = outMesh.vertices[i1 * 3 + 1], bz = outMesh.vertices[i1 * 3 + 2];
        float cx = outMesh.vertices[i2 * 3], cy = outMesh.vertices[i2 * 3 + 1], cz = outMesh.vertices[i2 * 3 + 2];

        float e1x = bx - ax, e1y = by - ay, e1z = bz - az;
        float e2x = cx - ax, e2y = cy - ay, e2z = cz - az;

        float fnx = e1y * e2z - e1z * e2y;
        float fny = e1z * e2x - e1x * e2z;
        float fnz = e1x * e2y - e1y * e2x;

        outMesh.normals[i0 * 3] += fnx;
        outMesh.normals[i0 * 3 + 1] += fny;
        outMesh.normals[i0 * 3 + 2] += fnz;
        outMesh.normals[i1 * 3] += fnx;
        outMesh.normals[i1 * 3 + 1] += fny;
        outMesh.normals[i1 * 3 + 2] += fnz;
        outMesh.normals[i2 * 3] += fnx;
        outMesh.normals[i2 * 3 + 1] += fny;
        outMesh.normals[i2 * 3 + 2] += fnz;
    }
    for (int i = 0; i < outMesh.vertexCount; i++){
        float nx = outMesh.normals[i * 3], ny = outMesh.normals[i * 3 + 1], nz = outMesh.normals[i * 3 + 2];
        float len = sqrtf(nx * nx + ny * ny + nz * nz);
        if (len > 1e-6f){
            outMesh.normals[i * 3] = nx / len;
            outMesh.normals[i * 3 + 1] = ny / len;
            outMesh.normals[i * 3 + 2] = nz / len;
        }
    }

    UploadMesh(&outMesh, false);
    outModel = LoadModelFromMesh(outMesh);
    outModel.materials[0].shader = litShader;
}

void CaveGen::Draw(){
    DrawModel(ceilingModel, {0, 200, 0}, 1,WHITE);
    DrawModel(floorModel, {0, -200, 0}, 1,WHITE);
}

void CaveGen::WaterDeposition(){
    const float waterScaleFactor = 2.f;
    const int spreadHops = 8;
    const float spreadFalloff = 0.4f;

    float* verts = ceilingModel.meshes[0].vertices;
    SortHeight(false, ceilingModel, ceiling);

    // Accumulate water flow from high to low
    water = std::vector<float>(ceiling.vertexCount, 1.f);
    std::vector<float> slopeSum(ceiling.vertexCount, 0.f);
    std::vector<std::vector<std::pair<int, float>>> outflow(ceiling.vertexCount);
    for (int index : orderedVertices){
        float hy = verts[index * 3 + 1];
        for (int nb : ceilingAdjacency[index]){
            float slope = hy - verts[nb * 3 + 1];
            if (slope > 0.f){
                slopeSum[index] += slope;
                outflow[index].push_back({nb, slope});
            }
        }
    }
    for (int index : orderedVertices){
        if (slopeSum[index] < 1e-6f) continue;
        for (auto [nb, slope] : outflow[index])
            water[nb] += water[index] * (slope / slopeSum[index]);
    }

    // Spread water outward hop by hop with falloff.
    // Each vertex takes the max of itself and decayed neighbors,
    // creating a ring-shaped displacement zone around each convergence point.
    std::vector<float> spread = water;
    std::vector<float> next(ceiling.vertexCount);
    for (int hop = 0; hop < spreadHops; hop++){
        for (int i = 0; i < ceiling.vertexCount; i++){
            next[i] = spread[i];
            for (int nb : ceilingAdjacency[i])
                next[i] = std::max(next[i], spread[nb] * spreadFalloff);
        }
        std::swap(spread, next);
    }

    // Only displace above the baseline (1.0 = every vertex's starting water).
    // Flat areas where nothing converged stay at spread ≈ 1, so displacement ≈ 0.
    for (int i = 0; i < ceiling.vertexCount; i++){
        float excess = std::max(0.f, spread[i] - 1.f);
        float displacement = powf(excess, 1.f / 3.f) * waterScaleFactor;
        float ny = ceilingModel.meshes[0].normals[3 * i + 1];
        float cosTheta = std::max(0.f, -ny);
        verts[i * 3 + 1] -= displacement * cosTheta;
    }

    UpdateMeshBuffer(ceilingModel.meshes[0], 0, verts, ceiling.vertexCount * 3 * sizeof(float), 0);
}

std::vector<int> CaveGen::FindLocalMinima(const Mesh& mesh, const std::vector<std::vector<int>>& adjacency){
    std::vector<int> minima;
    const float* verts = mesh.vertices;
    for (int i = 0; i < mesh.vertexCount; i++){
        float y = verts[i*3+1];
        bool isMin = true;
        for (int nb : adjacency[i]){
            if (verts[nb*3+1] <= y){
                isMin = false;
                break;
            }
        }
        if (isMin) minima.push_back(i);
    }
    return minima;
}

void CaveGen::SortHeight(bool decreasing, Model inModel, Mesh inMesh){
    // Build ordered vertex list from current mesh state
    const float* verts = inModel.meshes[0].vertices;
    orderedVertices.resize(inMesh.vertexCount);
    for (int i = 0; i < inMesh.vertexCount; i++) orderedVertices[i] = i;
    std::sort(orderedVertices.begin(), orderedVertices.end(), [&](int a, int b){
        return decreasing ? (verts[a * 3 + 1] > verts[b * 3 + 1]) : (verts[a * 3 + 1] < verts[b * 3 + 1]);
    });
}

void CaveGen::WaterDrip(){
    std::vector<int> stalactiteIndices = FindLocalMinima(ceiling, ceilingAdjacency);
    
    std::vector<float> floorWater(floor.vertexCount, 0.f);
    for (int i : stalactiteIndices)
        floorWater[i] = std::max(0.f, water[i] - 1.f);

    // Spread outward from each drip point (models the thin radial water film).
    // Lower falloff than ceiling (0.3 vs 0.4) → faster decay → more conical/pointed
    // shape, consistent with a higher Damköhler number typical of slow drip rates.
    const int spreadHops = 10;
    const float spreadFalloff = 0.3f;
    std::vector<float> spread = floorWater;
    std::vector<float> next(floor.vertexCount);
    for (int hop = 0; hop < spreadHops; hop++){
        for (int i = 0; i < floor.vertexCount; i++){
            next[i] = spread[i];
            for (int nb : floorAdjacency[i])
                next[i] = std::max(next[i], spread[nb] * spreadFalloff);
        }
        std::swap(spread, next);
    }

    // Displace floor vertices upward. Growth rate ∝ water^(1/3) follows the
    // Franke-Dreybrodt precipitation model (F = α(c − c_eq) under laminar flow).
    // floor.vertices retains pre-WaterDrip positions (separate alloc from the model).
    const float waterScaleFactor = 2.f;
    float* verts = floorModel.meshes[0].vertices;
    float* norms = floorModel.meshes[0].normals;
    std::vector<int> poolIndices = FindLocalMinima(floor, floorAdjacency);

    for (int i = 0; i < floor.vertexCount; i++){
        if (spread[i] < 1e-6f) continue;
        float cosTheta = std::max(0.f, norms[3 * i + 1]);
        verts[i * 3 + 1] += powf(spread[i], 1.f / 3.f) * waterScaleFactor * cosTheta;
    }

    // Safety: a floor pool (local minimum) must not become a peak.
    // If it was pushed above its lowest displaced neighbor, clamp it to that
    // neighbor's new Y and hand the overshoot as extra displacement to all
    // neighbors, then do a second upward pass with it.
    std::vector<float> extraDisp(floor.vertexCount, 0.f);
    for (int i : poolIndices){
        float lowestNbY = FLT_MAX;
        for (int nb : floorAdjacency[i])
            lowestNbY = std::min(lowestNbY, verts[nb * 3 + 1]);
        if (lowestNbY == FLT_MAX) continue;

        float overshoot = verts[i * 3 + 1] - lowestNbY;
        if (overshoot <= 0.f) continue;

        verts[i * 3 + 1] = lowestNbY;

        // Split overshoot equally between the pool itself and every neighbour
        // sitting at lowestNbY — 1/(levelNeighbours + 1) each.
        int levelCount = 0;
        for (int nb : floorAdjacency[i])
            if (verts[nb * 3 + 1] <= lowestNbY + 1e-4f) levelCount++;
        float share = overshoot / (float)(levelCount + 1);
        extraDisp[i] += share;
        for (int nb : floorAdjacency[i])
            if (verts[nb * 3 + 1] <= lowestNbY + 1e-4f) extraDisp[nb] += share;
    }

    for (int i = 0; i < floor.vertexCount; i++){
        if (extraDisp[i] < 1e-6f) continue;
        verts[i * 3 + 1] += extraDisp[i];
    }

    UpdateMeshBuffer(floorModel.meshes[0], 0, verts, floor.vertexCount * 3 * sizeof(float), 0);
}
