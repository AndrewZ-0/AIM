#include <raylib.h>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "pkff.h"
#include "sim.h"

inline R3s interpolate_keyframes(const Keyframe& kf0, const Keyframe& kf1, u32 current_time) {
    if (kf1.dt_skip <= kf0.dt_skip) return kf0.s;
    const f64 alpha = (current_time - (f64)kf0.dt_skip) / ((f64)kf1.dt_skip - (f64)kf0.dt_skip);
    const f64 t = std::max(0.0, std::min(1.0, alpha));
    return {
        static_cast<f32>(kf0.s.x + (kf1.s.x - kf0.s.x) * t),
        static_cast<f32>(kf0.s.y + (kf1.s.y - kf0.s.y) * t),
        static_cast<f32>(kf0.s.z + (kf1.s.z - kf0.s.z) * t)
    };
}

inline f32 get_velocity_magnitude(const Vec<Keyframe>& track, usize idx, f32 dt) {
    if (idx + 1 >= track.size()) return 0.0f;
    
    const R3s& pos0 = track[idx].s;
    const R3s& pos1 = track[idx + 1].s;
    
    f64 dx = pos1.x - pos0.x;
    f64 dy = pos1.y - pos0.y;
    f64 dz = pos1.z - pos0.z;
    
    f64 displacement_sq = dx * dx + dy * dy + dz * dz;
    f32 time_diff = (f32)(track[idx + 1].dt_skip - track[idx].dt_skip) * dt;
    
    if (time_diff > 0) {
        return std::sqrt(displacement_sq) / time_diff;
    }
    return 0.0f;
}

inline Color velocity_to_color(f32 v, f32 vmin, f32 vmax) {
    const f32 eps = 1e-6f;

    v = std::max(v, eps);
    vmin = std::max(vmin, eps);
    vmax = std::max(vmax, eps);

    f32 t = (v - vmin) / (vmax - vmin + 1e-10f);

    t = std::min(std::max(t, 0.0f), 1.0f);

    t = t * t;

    const f32 x = t * 4.0f;

    u8 r, g, b;

    if (x < 1.0f) {
        r = 0;
        g = (u8)(255 * x);
        b = 255;
    } else if (x < 2.0f) {
        r = 0;
        g = 255;
        b = (u8)(255 * (2.0f - x));
    } else if (x < 3.0f) {
        r = (u8)(255 * (x - 2.0f));
        g = 255;
        b = 0;
    } else {
        r = 255;
        g = (u8)(255 * (4.0f - x));
        b = 0;
    }

    return {r, g, b, 255};
}

inline R3s get_particle_pos(const Vec<Keyframe>& track, usize& idx, u32 current_time) {
    if (track.empty()) return {0, 0, 0};
    
    while (idx + 1 < track.size() && track[idx + 1].dt_skip <= current_time) {
        idx++;
    }
    
    if (idx + 1 < track.size()) {
        return interpolate_keyframes(track[idx], track[idx + 1], current_time);
    }
    return track[idx].s;
}

int main() {
    PKF pkf;
    read_pkf("sim2.pkff", pkf);

    const usize N = pkf.particles.size();
    const u32 play_dt = 1;
    u32 current_time = 0;
    Vec<usize> keyframe_idx(N, 0);

    InitWindow(1200, 800, "N-Body PKF Visualizer");
    SetTargetFPS(60);

    Camera3D camera = {{0.0f, 0.0f, 0.0f}, {1, 0, 0}, {0, 1, 0}, 60.0f, CAMERA_PERSPECTIVE};
    f32 cam_distance = 180.0f;
    f32 cam_yaw = 0.5f;
    f32 cam_pitch = 0.3f;

    while (!WindowShouldClose()) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 delta = GetMouseDelta();
            cam_yaw -= delta.x * 0.01f;
            cam_pitch -= delta.y * 0.01f;
            cam_pitch = std::max(-1.5f, std::min(1.5f, cam_pitch));
        }
        
        cam_distance -= GetMouseWheelMove() * 0.5f;
        cam_distance = std::max(1.0f, cam_distance);
        
        camera.position = {
            -std::cos(cam_yaw) * std::cos(cam_pitch) * cam_distance,
            -std::sin(cam_pitch) * cam_distance,
            std::sin(cam_yaw) * std::cos(cam_pitch) * cam_distance
        };

        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);

        current_time += play_dt;
        if (current_time > pkf.num_dt) {
            current_time = 0;
            for (usize i = 0; i < N; i++) keyframe_idx[i] = 0;
        }

        f32 vel_min = 1e10f, vel_max = 0.0f;
        for (usize i = 0; i < N; i++) {
            if (!pkf.particles[i].empty()) {
                usize idx = keyframe_idx[i];
                f32 v = get_velocity_magnitude(pkf.particles[i], idx, pkf.dt);
                vel_min = std::min(vel_min, v);
                vel_max = std::max(vel_max, v);
            }
        }
        if (vel_min > vel_max) vel_min = vel_max;

        for (usize i = 0; i < N; i++) {
            R3s pos3 = get_particle_pos(pkf.particles[i], keyframe_idx[i], current_time);
            Vector3 pos = {
                100 * static_cast<f32>(pos3.x / init_d), 
                100 * static_cast<f32>(pos3.y / init_d), 
                100 * static_cast<f32>(pos3.z / init_d)
            };
            
            f32 vel_mag = get_velocity_magnitude(pkf.particles[i], keyframe_idx[i], pkf.dt);
            Color col = velocity_to_color(vel_mag, vel_min, vel_max);
            
            //DrawCubeV(pos, {1.1, 1.1, 1.1}, col);
            DrawPoint3D(pos, col);
        }

        EndMode3D();
        DrawText(TextFormat("time %u/%u", current_time, pkf.num_dt), 10, 10, 16, RAYWHITE);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}