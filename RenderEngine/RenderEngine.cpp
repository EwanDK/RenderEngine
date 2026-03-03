#include "raylib.h"
#include "raymath.h"
#include "SoftBody.h"
#include "Collision/CollisionSystem.h"
#include "Collision/ConvexClustering.h"
#include "Collision/GJK.h"
#include "Input/InputSystem.h"
#include "SpectatorCamera.h"
#include <vector>
#include <cmath>

int main(int, char**)
{
    InitWindow(1280,720,"Soft Body Simulation");

    SpectatorCamera camera({ 0.0f, 150.0f, 0.0f });

    SoftBody soft = SoftBody();
    soft.Translate({0.0f, 100.0f, 0.0f}); // Start above the cube so it falls onto it

    // Ground plane well below the cube
    Collision::GroundPlane ground({0, 1, 0}, -200.0f);
    soft.SetGroundPlane(ground);

    // Static cube obstacle — top face at y=10, bottom at y=-10
    Vector3 cubePos = {0.0f, 0.0f, 0.0f};
    Vector3 cubeSize = {80.0f, 20.0f, 80.0f};
    Mesh cubeMesh = GenMeshCube(cubeSize.x, cubeSize.y, cubeSize.z);
    UploadMesh(&cubeMesh, false);
    Model cubeModel = LoadModelFromMesh(cubeMesh);

    // Extract cube verts in world space and register as static collider
    std::vector<Vector3> cubeVerts;
    Matrix cubeTransform = MatrixTranslate(cubePos.x, cubePos.y, cubePos.z);
    Collision::ConvexShape cubeShape = Collision::ConvexShapeFromMesh(cubeMesh, cubeTransform, cubeVerts);
    soft.AddStaticCollider(cubeShape);
    //SetTargetFPS(60);

    // Debug visualization flags (toggled by F1-F4)
    bool showClusters  = false;
    bool showParticles = false;
    bool showContacts  = false;
    bool showSprings   = false;

    const Color clusterColors[] = {RED, GREEN, BLUE, YELLOW, PURPLE, ORANGE};
    const int numClusterColors = 6;

    Input::InputSystem inputSystem;

    while (!WindowShouldClose())
    {
        // Poll input system and process actions
        auto actions = inputSystem.Poll();
        float dt = GetFrameTime();

        camera.Update(actions, dt);

        for (const auto& a : actions) {
            if (a.type != Input::InputEvent::Pressed) continue;
            switch (a.action) {
                case Input::ActionID::ToggleClusters:  showClusters  = !showClusters;  break;
                case Input::ActionID::ToggleParticles: showParticles = !showParticles; break;
                case Input::ActionID::ToggleContacts:  showContacts  = !showContacts;  break;
                case Input::ActionID::ToggleSprings:   showSprings   = !showSprings;   break;
                default: break;
            }
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera.GetCamera());

        soft.Update(dt);

        // Draw static cube obstacle
        DrawModel(cubeModel, cubePos, 1.0f, GRAY);
        DrawModelWires(cubeModel, cubePos, 1.0f, DARKGRAY);

        // Draw ground plane visual
        DrawPlane(Vector3{0.0f, -60.0f, 0.0f}, Vector2{400.0f, 400.0f}, LIGHTGRAY);
        DrawGrid(20, 10.0f);

        // --- Debug visualizations ---
        const auto& pts = soft.GetPoints();

        // F1: Cluster membership + hull vertices
        if (showClusters) {
            const auto& clusters = soft.GetClusters();
            for (int c = 0; c < (int)clusters.size(); c++) {
                Color col = clusterColors[c % numClusterColors];
                // Particles colored by cluster
                for (int idx : clusters[c].pointIndices) {
                    DrawSphere(pts[idx].position, 1.5f, col);
                }
                // Hull vertices as wireframe spheres
                for (const auto& hv : clusters[c].hullVertices) {
                    DrawSphereWires(hv, 2.5f, 4, 4, col);
                }
            }
        }

        // F2: All particle spheres
        if (showParticles) {
            for (const auto& p : pts) {
                DrawSphere(p.position, p.radius, Fade(RED, 0.5f));
            }
        }

        // F3: Contact normals
        if (showContacts) {
            const auto& contacts = soft.GetDebugContacts();
            for (const auto& c : contacts) {
                Vector3 end = c.position + c.normal * 5.0f;
                DrawLine3D(c.position, end, MAGENTA);
                DrawSphere(c.position, 0.5f, MAGENTA);
            }
        }

        // F4: Springs colored by stretch ratio, struts in cyan
        if (showSprings) {
            const auto& springs = soft.GetSprings();
            for (const auto& s : springs) {
                float dist = Vector3Distance(s.points[0]->position, s.points[1]->position);
                float ratio = dist / s.baseDistance;
                float stretch = fabsf(ratio - 1.0f);
                float t = fminf(stretch / 0.5f, 1.0f);
                unsigned char r = (unsigned char)(t * 255.0f);
                unsigned char g = (unsigned char)((1.0f - t) * 255.0f);
                Color col = {r, g, 0, 255};
                DrawLine3D(s.points[0]->position, s.points[1]->position, col);
            }
            const auto& struts = soft.GetStruts();
            for (const auto& st : struts) {
                DrawLine3D(st.a->position, st.b->position, SKYBLUE);
            }
        }

        EndMode3D();

        // HUD
        DrawText("Soft Body + GJK Collision", 10, 10, 20, DARKGRAY);
        DrawFPS(10, 40);

        DrawText(TextFormat("[F1] Clusters:  %s", showClusters  ? "ON" : "OFF"), 10, 70,  15, showClusters  ? GREEN : GRAY);
        DrawText(TextFormat("[F2] Particles: %s", showParticles ? "ON" : "OFF"), 10, 90,  15, showParticles ? GREEN : GRAY);
        DrawText(TextFormat("[F3] Contacts:  %s", showContacts  ? "ON" : "OFF"), 10, 110, 15, showContacts  ? GREEN : GRAY);
        DrawText(TextFormat("[F4] Springs:   %s", showSprings   ? "ON" : "OFF"), 10, 130, 15, showSprings   ? GREEN : GRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
