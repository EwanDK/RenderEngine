#include "CaveGen.h"
#include <raylib.h>
#include "MathUtils.h"
#include "PerlinNoise.hpp"
#include <algorithm>
#include <cstring>
#include <unordered_set>

#include "Renderer.h"

void CaveGen::GenerateMesh(Shader litShader){
    constexpr float sizeX = 1000.f;
    constexpr float sizeY = 1000.f;
    constexpr int subdivX = 100;
    constexpr int subdivY = 100;
    constexpr float noiseScale = 0.01f;
    const float maxDisplacement = std::max(sizeX, sizeY) / 10.f;

    planeSubdivX = subdivX;
    BuildMeshSection(ceiling, ceilingModel, ceilingAdjacency, {0, -1, 0}, sizeX, sizeY, subdivX, subdivY, noiseScale, maxDisplacement, litShader);
    BuildMeshSection(floor, floorModel, floorAdjacency, {0, 1, 0}, sizeX, sizeY, subdivX, subdivY, noiseScale, maxDisplacement, litShader);

    // For the equilateral layout each "band" of two rows has (2*subdivX+1) vertices.
    // Within a band, even rows have (subdivX+1) entries and odd rows have subdivX.
    // The floor is generated with normal flipped on Z, so ceiling vertex i mirrors
    // to the floor vertex in the same row with column (subdivX - col) for even rows
    // and (subdivX - 1 - col) for odd rows.
    int W = 2 * subdivX + 1;
    ceilToFloor.resize(ceiling.vertexCount);
    for (int i = 0; i < ceiling.vertexCount; i++) {
        int band = i / W;
        int pos = i % W;
        ceilToFloor[i] = pos <= subdivX
            ? band * W + (subdivX - pos)
            : band * W + (3 * subdivX + 1 - pos);
    }

    AddToLit(EventBus::Get());
}

void CaveGen::BuildMeshSection(Mesh& outMesh, Model& outModel, std::vector<std::vector<int>>& outAdjacency, Vector3 normal, float sizeX, float sizeY, int subdivX, int subdivY, float noiseScale, float maxDisplacement, Shader litShader){
    outMesh = {0};
    int indexCount = 0;
    //GeneratePlane(sizeX, sizeY, subdivX, subdivY, normal, &outMesh.vertices, &outMesh.normals, &outMesh.indices, &outMesh.vertexCount, &indexCount);
    GeneratePlaneEquilateral(sizeX,subdivX,subdivY,normal, &outMesh.vertices, &outMesh.normals, &outMesh.indices, &outMesh.vertexCount, &indexCount);
    outMesh.triangleCount = indexCount / 3;

    // Initial displacement through Perlin Noise (FBM: 5 octaves)
    siv::PerlinNoise noise;
    for (int i = 0; i < outMesh.vertexCount; i++){
        float& vx = outMesh.vertices[i * 3 + 0];
        float& vy = outMesh.vertices[i * 3 + 1];
        float& vz = outMesh.vertices[i * 3 + 2];

        float disp = 0.f, amp = 1.f, freq = noiseScale, totalAmp = 0.f;
        for (int o = 0; o < 3; o++){
            disp += (float)noise.noise2D_01(vx * freq, vz * freq) * amp;
            totalAmp += amp;
            amp *= 0.3f;
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

void CaveGen::DebugNudge(){
    constexpr int targetIdx = 935;
    constexpr float nudgeAmount = 100.f;

    auto nudgeMesh = [&](Model& model, const std::vector<std::vector<int>>& adjacency, float normalY){
        float* verts = model.meshes[0].vertices;
        int vertCount = model.meshes[0].vertexCount;
        if (targetIdx >= vertCount) return;

        verts[targetIdx * 3 + 1] += normalY * nudgeAmount;
        for (int nb : adjacency[targetIdx])
            verts[nb * 3 + 1] += normalY * nudgeAmount;

        UpdateMeshBuffer(model.meshes[0], 0, verts, vertCount * 3 * sizeof(float), 0);
    };

    nudgeMesh(ceilingModel, ceilingAdjacency, -1.f);
    nudgeMesh(floorModel, floorAdjacency, 1.f);
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
    // The first hop seeds ring-1 neighbours at full value (no decay) so that
    // the immediate ring approximates the minima, producing a circular base.
    // Falloff begins at ring-2 outward.
    constexpr float firstHopFalloff = 0.9f;
    std::vector<float> spread = water;
    std::vector<float> next(ceiling.vertexCount);
    for (int i = 0; i < ceiling.vertexCount; i++){
        next[i] = spread[i];
        for (int nb : ceilingAdjacency[i])
            next[i] = std::max(next[i], spread[nb] * firstHopFalloff);
    }
    std::swap(spread, next);
    for (int hop = 0; hop < spreadHops; hop++){
        for (int i = 0; i < ceiling.vertexCount; i++){
            next[i] = spread[i];
            for (int nb : ceilingAdjacency[i])
                next[i] = std::max(next[i], spread[nb] * spreadFalloff);
        }
        std::swap(spread, next);
    }

    std::unordered_set<int> pillarCeilSet;
    for (const auto& p : pillars){
        pillarCeilSet.insert(p.ceilingIdx);
        for (const auto& rv : p.ceilingRing) pillarCeilSet.insert(rv.meshIdx);
    }

    // Only displace above the baseline (1.0 = every vertex's starting water).
    // Flat areas where nothing converged stay at spread ≈ 1, so displacement ≈ 0.
    for (int i = 0; i < ceiling.vertexCount; i++){
        if (pillarCeilSet.count(i)) continue;
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

static void ExtendMesh(Mesh& mesh, int extraVerts, int extraTris){
    int oldVC = mesh.vertexCount;
    int oldTC = mesh.triangleCount;
    mesh.vertices = (float*)realloc(mesh.vertices, (oldVC + extraVerts) * 3 * sizeof(float));
    mesh.normals = (float*)realloc(mesh.normals, (oldVC + extraVerts) * 3 * sizeof(float));
    mesh.indices = (unsigned short*)realloc(mesh.indices, (oldTC + extraTris) * 3 * sizeof(unsigned short));
    memset(mesh.vertices + oldVC * 3, 0, extraVerts * 3 * sizeof(float));
    memset(mesh.normals + oldVC * 3, 0, extraVerts * 3 * sizeof(float));
    mesh.vertexCount = oldVC + extraVerts;
    mesh.triangleCount = oldTC + extraTris;
}

static void RebuildMeshOnGPU(Mesh& mesh, Model& model){
    float* v = mesh.vertices;
    float* n = mesh.normals;
    unsigned short* idx = mesh.indices;
    int vc = mesh.vertexCount;
    int tc = mesh.triangleCount;
    mesh.vertices = nullptr;
    mesh.normals = nullptr;
    mesh.indices = nullptr;
    UnloadMesh(mesh); // frees GPU VAO/VBOs; CPU ptrs are null so not freed
    mesh.vertices = v;
    mesh.normals = n;
    mesh.indices = idx;
    mesh.vertexCount = vc;
    mesh.triangleCount = tc;
    mesh.vaoId = 0;
    mesh.vboId = nullptr;
    UploadMesh(&mesh, false);
    model.meshes[0] = mesh;
}

void CaveGen::DetectAndInitPillars(){
    // Cover ring verts added in previous calls so FindLocalMinima won't go out of bounds
    ceilingAdjacency.resize(ceiling.vertexCount);
    floorAdjacency.resize(floor.vertexCount);

    std::unordered_set<int> pillarSet;
    std::unordered_set<int> ringCeilSet;
    for (const auto& p : pillars){
        pillarSet.insert(p.ceilingIdx);
        for (const auto& rv : p.ceilingRing) ringCeilSet.insert(rv.meshIdx);
    }

    std::vector<int> ceilMinima = FindLocalMinima(ceiling, ceilingAdjacency);
    bool anyNew = false;

    for (int ci : ceilMinima){
        if (pillarSet.count(ci) || ringCeilSet.count(ci)) continue;

        // Refresh after each iteration: prior ExtendMesh may have moved the arrays
        float* cVerts = ceiling.vertices;
        float* fVerts = floor.vertices;

        int fi = ceilToFloor[ci];

        float cWorldY = cVerts[ci * 3 + 1] + 200.f;
        float fWorldY = fVerts[fi * 3 + 1] - 200.f;
        if (cWorldY > fWorldY) continue;

        float avgWorldY = (cWorldY + fWorldY) * 0.5f;
        float lostHeight = (fWorldY - cWorldY) * 0.5f;

        cVerts[ci * 3 + 1] = avgWorldY - 200.f;
        fVerts[fi * 3 + 1] = avgWorldY + 200.f;

        float cx = cVerts[ci * 3], cy = cVerts[ci * 3 + 1], cz = cVerts[ci * 3 + 2];
        float fx = fVerts[fi * 3], fy = fVerts[fi * 3 + 1], fz = fVerts[fi * 3 + 2];

        std::vector<int> cNbs = ceilingAdjacency[ci];
        std::vector<int> fNbs = floorAdjacency[fi];
        std::sort(cNbs.begin(), cNbs.end(), [&](int a, int b){
            return atan2f(cVerts[a*3+2]-cz, cVerts[a*3]-cx) < atan2f(cVerts[b*3+2]-cz, cVerts[b*3]-cx);
        });
        std::sort(fNbs.begin(), fNbs.end(), [&](int a, int b){
            return atan2f(fVerts[a*3+2]-fz, fVerts[a*3]-fx) < atan2f(fVerts[b*3+2]-fz, fVerts[b*3]-fx);
        });

        int cN = (int)cNbs.size(), fN = (int)fNbs.size();
        int cOldVC = ceiling.vertexCount, cOldTC = ceiling.triangleCount;
        int fOldVC = floor.vertexCount, fOldTC = floor.triangleCount;

        ExtendMesh(ceiling, cN, cN);
        ExtendMesh(floor, fN, fN);
        ceilingAdjacency.resize(ceiling.vertexCount); // empty adjacency for new ring verts
        floorAdjacency.resize(floor.vertexCount);

        // Refresh after realloc; re-read center positions (data unchanged, ptr may differ)
        cVerts = ceiling.vertices;
        fVerts = floor.vertices;
        cx = cVerts[ci*3]; cy = cVerts[ci*3+1]; cz = cVerts[ci*3+2];
        fx = fVerts[fi*3]; fy = fVerts[fi*3+1]; fz = fVerts[fi*3+2];

        PillarInfo p;
        p.ceilingIdx = ci;
        p.floorIdx = fi;
        p.lostHeight = lostHeight;
        p.growthStopped = false;

        for (int k = 0; k < cN; k++){
            int nb = cNbs[k];
            float dx = cVerts[nb*3] - cx, dz = cVerts[nb*3+2] - cz;
            float d = sqrtf(dx*dx + dz*dz);
            if (d < 1e-6f){ dx = 1.f; dz = 0.f; } else { dx /= d; dz /= d; }
            int ri = cOldVC + k;
            ceiling.vertices[ri*3] = cx + dx * lostHeight;
            ceiling.vertices[ri*3+1] = cy;
            ceiling.vertices[ri*3+2] = cz + dz * lostHeight;
            ceiling.normals[ri*3] = dx;
            ceiling.normals[ri*3+1] = 0.f;
            ceiling.normals[ri*3+2] = dz;
            p.ceilingRing.push_back({ri, nb, dx, dz});
            // ceiling winding: ring[k] → nb[k] → ring[(k+1)%N] → outward XZ normal
            int ri_next = cOldVC + (k + 1) % cN;
            int ti = (cOldTC + k) * 3;
            ceiling.indices[ti] = (unsigned short)ri;
            ceiling.indices[ti+1] = (unsigned short)nb;
            ceiling.indices[ti+2] = (unsigned short)ri_next;
        }

        for (int k = 0; k < fN; k++){
            int nb = fNbs[k];
            float dx = fVerts[nb*3] - fx, dz = fVerts[nb*3+2] - fz;
            float d = sqrtf(dx*dx + dz*dz);
            if (d < 1e-6f){ dx = 1.f; dz = 0.f; } else { dx /= d; dz /= d; }
            int ri = fOldVC + k;
            floor.vertices[ri*3] = fx + dx * lostHeight;
            floor.vertices[ri*3+1] = fy;
            floor.vertices[ri*3+2] = fz + dz * lostHeight;
            floor.normals[ri*3] = dx;
            floor.normals[ri*3+1] = 0.f;
            floor.normals[ri*3+2] = dz;
            p.floorRing.push_back({ri, nb, dx, dz});
            // floor winding reversed: ring[k] → ring[(k+1)%N] → nb[k]
            int ri_next = fOldVC + (k + 1) % fN;
            int ti = (fOldTC + k) * 3;
            floor.indices[ti] = (unsigned short)ri;
            floor.indices[ti+1] = (unsigned short)ri_next;
            floor.indices[ti+2] = (unsigned short)nb;
        }

        pillars.push_back(std::move(p));
        anyNew = true;
    }

    if (anyNew){
        RebuildMeshOnGPU(ceiling, ceilingModel);
        RebuildMeshOnGPU(floor, floorModel);
    }
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

void CaveGen::GrowPillars(){
    if (pillars.empty()) return;

    const float waterScaleFactor = 2.f;
    const float stopThreshold = 0.5f; // XZ dist to base neighbor below this → nearly vertical

    float* cVerts = ceiling.vertices;
    float* fVerts = floor.vertices;
    bool any = false;

    for (auto& p : pillars){
        if (p.growthStopped) continue;

        // Same water-driven growth rate as vertical, but redirected to XZ
        float excess = std::max(0.f, water[p.ceilingIdx] - 1.f);
        float growStep = powf(excess, 1.f / 3.f) * waterScaleFactor;
        if (growStep < 1e-6f) continue;

        any = true;
        bool stop = false;

        for (const auto& rv : p.ceilingRing){
            int ri = rv.meshIdx, nb = rv.neighborIdx;
            cVerts[ri*3] += rv.dirX * growStep;
            cVerts[ri*3+2] += rv.dirZ * growStep;
            float dx = cVerts[nb*3] - cVerts[ri*3];
            float dz = cVerts[nb*3+2] - cVerts[ri*3+2];
            if (sqrtf(dx*dx + dz*dz) < stopThreshold) stop = true;
        }

        for (const auto& rv : p.floorRing){
            int ri = rv.meshIdx, nb = rv.neighborIdx;
            fVerts[ri*3] += rv.dirX * growStep;
            fVerts[ri*3+2] += rv.dirZ * growStep;
            float dx = fVerts[nb*3] - fVerts[ri*3];
            float dz = fVerts[nb*3+2] - fVerts[ri*3+2];
            if (sqrtf(dx*dx + dz*dz) < stopThreshold) stop = true;
        }

        if (stop) p.growthStopped = true;
    }

    if (any){
        UpdateMeshBuffer(ceilingModel.meshes[0], 0, cVerts, ceiling.vertexCount * 3 * sizeof(float), 0);
        UpdateMeshBuffer(floorModel.meshes[0], 0, fVerts, floor.vertexCount * 3 * sizeof(float), 0);
    }
}

void CaveGen::WaterDrip(){
    std::vector<int> stalactiteIndices = FindLocalMinima(ceiling, ceilingAdjacency);
    
    std::vector<float> floorWater(floor.vertexCount, 0.f);
    for (int i : stalactiteIndices){
        int floorIdx = ceilToFloor[i];
        floorWater[floorIdx] = std::max(0.f, water[i] - 1.f);
    }

    // Spread outward from each drip point (models the thin radial water film).
    // Lower falloff than ceiling (0.3 vs 0.4) → faster decay → more conical/pointed
    // shape, consistent with a higher Damköhler number typical of slow drip rates.
    // First hop uses a gentle decay so ring-1 sits just below the minima
    // (prevents ring-1 verts becoming local minima) while still being much
    // closer to the peak than the steep per-hop falloff, giving a circular base.
    constexpr float firstHopFalloff = 0.9f;
    const int spreadHops = 10;
    const float spreadFalloff = 0.3f;
    std::vector<float> spread = floorWater;
    std::vector<float> next(floor.vertexCount);
    for (int i = 0; i < floor.vertexCount; i++){
        next[i] = spread[i];
        for (int nb : floorAdjacency[i])
            next[i] = std::max(next[i], spread[nb] * firstHopFalloff);
    }
    std::swap(spread, next);
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

    /*std::unordered_set<int> pillarFloorSet;
    for (const auto& p : pillars){
        pillarFloorSet.insert(p.floorIdx);
        for (const auto& rv : p.floorRing) pillarFloorSet.insert(rv.meshIdx);
    }*/

    for (int i = 0; i < floor.vertexCount; i++){
        //if (pillarFloorSet.count(i)) continue;
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
        //if (pillarFloorSet.count(i)) continue;
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
        //if (pillarFloorSet.count(i)) continue;
        if (extraDisp[i] < 1e-6f) continue;
        verts[i * 3 + 1] += extraDisp[i];
    }

    UpdateMeshBuffer(floorModel.meshes[0], 0, verts, floor.vertexCount * 3 * sizeof(float), 0);
    //GrowPillars();
    //DetectAndInitPillars();
}
