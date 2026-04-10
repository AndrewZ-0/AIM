#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <iostream>

#include "sim.h"

static Vec<R3> gaussian_centers;
static Vec<f64> gaussian_amplitudes;
static Vec<f64> gaussian_widths;
static bool field_initialized = false;

inline void init_random_field() {
    if (field_initialized) return;
    
    const int num_gaussians = 8;
    const f64 box_size = init_d * 2.0;
    
    gaussian_centers.resize(num_gaussians);
    gaussian_amplitudes.resize(num_gaussians);
    gaussian_widths.resize(num_gaussians);
    
    srand(static_cast<unsigned int>(time(NULL)));
    
    for (int i = 0; i < num_gaussians; ++i) {
        gaussian_centers[i].x = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * box_size;
        gaussian_centers[i].y = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * box_size;
        gaussian_centers[i].z = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * box_size;
        
        gaussian_amplitudes[i] = 0.5 + static_cast<f64>(rand()) / RAND_MAX;
        
        gaussian_widths[i] = init_d * (0.2 + static_cast<f64>(rand()) / RAND_MAX * 0.8);
    }
    
    field_initialized = true;
}

inline f64 density_field(const R3& pos) {
    init_random_field();
    
    f64 density = 0.0;
    for (size_t i = 0; i < gaussian_centers.size(); ++i) {
        const R3 delta = {pos.x - gaussian_centers[i].x, pos.y - gaussian_centers[i].y, pos.z - gaussian_centers[i].z};
        const f64 r_sq = len_sq(delta);
        const f64 width_sq = gaussian_widths[i] * gaussian_widths[i];
        density += gaussian_amplitudes[i] * std::exp(-r_sq / width_sq);
    }
    
    const f64 max_density = gaussian_centers.size() * 1.5;
    return std::min(1.0, density / max_density);
}

inline void init_particles(Particles& p, usize N) {
    srand(static_cast<unsigned int>(time(NULL)));

    p.s.resize(N);
    p.v.resize(N, {0.0, 0.0, 0.0});
    p.a.resize(N, {0.0, 0.0, 0.0});
    p.m.resize(N);

    const f64 half_d = init_d * 1.0;
    
    //sample particles from density field
    for (usize i = 0; i < N; i++) {
        f64 sampled_density = 0.0;
        R3 sampled_pos;
        
        sampled_pos.x = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * 2.0 * half_d;
        sampled_pos.y = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * 2.0 * half_d;
        sampled_pos.z = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * 2.0 * half_d;
        sampled_density = density_field(sampled_pos);
        
        p.s[i] = sampled_pos;
        
        //init vel inv prop to local density (low density = fast)
        const f64 vel_scale = (1.0 - sampled_density) * init_d * 0.5e-15;
        p.v[i].x = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * vel_scale;
        p.v[i].y = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * vel_scale;
        p.v[i].z = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * vel_scale;
        
        p.m[i] = 0.5 * M + (static_cast<f64>(rand()) / RAND_MAX) * M;
    }
}

inline void apply_grav(Particles& p, usize N) {
    for (usize i = 0; i < N; i++) {
        p.a[i] = {0.0, 0.0, 0.0};
        for (usize j = 0; j < N; j++) {
            if (i == j) continue;
            const R3 dr = p.s[j] - p.s[i];
            const f64 r_sq = len_sq(dr);
            const f64 r_sq_soft = r_sq + softening * softening;
            p.a[i] += (p.m[j] / (r_sq_soft * std::sqrt(r_sq_soft))) * dr;
        }
    }
}

inline void run_leapfrog(Particles& p, PKF& pkf, usize N) {
    pkf.particles.resize(N);

    for (usize i = 0; i < N; i++) {
        pkf.particles[i].push_back(Keyframe{toR3s(p.s[i]), 0});
    }

    const f64 k = 0.5 * G * dt;
    for (u32 nt = 0; nt < num_dt; nt++) {
        apply_grav(p, N);

        for (usize i = 0; i < N; i++) {
            p.v[i] += k * p.a[i];
            p.s[i] += dt * p.v[i];
        }

        apply_grav(p, N);

        for (usize i = 0; i < N; i++) {
            p.v[i] += k * p.a[i];

            const f64 vel_magnitude = std::sqrt(len_sq(p.v[i]));
            const f64 acc_magnitude = std::sqrt(len_sq(p.a[i]));
            const f64 direction_change_factor = acc_magnitude / (vel_magnitude + 1e-10); //avoid division by zero
            const f64 dynamic_epsilon = epsilon / (1.0 + direction_change_scale * direction_change_factor);
            const f64 dynamic_epsilon_sq = dynamic_epsilon * dynamic_epsilon;

            if (len_sq(p.s[i] - toR3(pkf.particles[i].back().s)) >= dynamic_epsilon_sq) {
                pkf.particles[i].push_back(Keyframe{toR3s(p.s[i]), nt});
            }
        }

        std::cout << nt << "/" << num_dt << std::endl; 
    }
}


int main() {
    Particles p; 
    PKF pkf;

    pkf.dt = dt;
    pkf.num_dt = num_dt;

    const usize N = n * n * n;
    init_particles(p, N);

    run_leapfrog(p, pkf, N);

    std::cout << pkf.particles[0].size() << std::endl;
    std::cout << pkf.particles[1].size() << std::endl;
    std::cout << pkf.particles[2].size() << std::endl;

    write_pkf("sim.pkff", pkf);

    return 0;
}