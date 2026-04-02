#include <raylib.h>
#include <iostream>

#include "pkff.h"
#include "sim.h"

int main() {
    PKF pkf;
    read_pkf("sim.pkff", pkf);

    const usize N = pkf.particles.size();
    const u32 play_dt = 1;
    u32 current_time = 0;
    Vec<usize> keyframe_idx(N, 0);

    const int screenWidth = 1200;
    const int screenHeight = 800;
    InitWindow(screenWidth, screenHeight, "N-Body PKF Visualizer");
    SetTargetFPS(60);

    Camera3D camera;
    camera.position = {-20.0f, 0.0f, 0.0f};
    camera.target = {0.0f, 0.0f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 90.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        UpdateCamera(&camera, CAMERA_ORBITAL);

        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode3D(camera);

        //std::cout << ">>> " << current_time << ", " << pkf.num_dt << std::endl;

        current_time += play_dt;
        if (current_time > pkf.num_dt) {
            current_time = 0;
            for (usize i = 0; i < N; i++) {
                keyframe_idx[i] = 0;
            }
        }

        for (usize i = 0; i < N; i++) {
            const Vec<Keyframe>& track = pkf.particles[i];
            if (track.empty()) continue;

            while (keyframe_idx[i] + 1 < track.size() && track[keyframe_idx[i] + 1].dt_skip <= current_time) {
                keyframe_idx[i]++;
            }

            R3 pos3 = track[keyframe_idx[i]].s;

            const Vector3 pos = {
                static_cast<f32>(pos3.x / init_d),
                static_cast<f32>(pos3.y / init_d),
                static_cast<f32>(pos3.z / init_d)
            };

            if (i == 0) {
                std::cout << keyframe_idx[i] << ": " << pos.x << ", " << pos.y << ", "  << pos.z << std::endl;
            }
            DrawSphere(pos, 0.1f, WHITE);
        }

        EndMode3D();

        DrawText(TextFormat("N-Body Visualization: time %u/%u (step %u)", current_time, pkf.num_dt, play_dt), 10, 10, 20, RAYWHITE);
        DrawText("Use mouse to orbit, scroll to zoom", 10, 34, 16, RAYWHITE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}