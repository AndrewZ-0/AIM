#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <iostream>

#include "sim.h"

inline void init_particles(Particles& p, usize N) {
    srand(static_cast<unsigned int>(time(NULL)));

    p.s.resize(N);
    p.v.resize(N, {0.0, 0.0, 0.0});
    p.a.resize(N, {0.0, 0.0, 0.0});
    p.m.resize(N);

    for (usize x = 0; x < n; x++) {
        for (usize y = 0; y < n; y++) {
            for (usize z = 0; z < n; z++) {
                const usize i = x * n * n + y * n + z;
                
                p.s[i] = {(f64)x * init_d, (f64)y * init_d, (f64)z * init_d};
                p.m[i] = 0.5 * M + (static_cast<f64>(rand()) / RAND_MAX) * M;
            }
        }
    }
}

inline void apply_grav(Particles& p, usize N) {
    for (usize i = 0; i < N; i++) {
        p.a[i] = {0.0, 0.0, 0.0}; //treated as both accumulator & acceleration
        for (usize j = 0; j < N; j++) {
            if (i == j) continue;
            const R3 dr = p.s[j] - p.s[i];
            const f64 r_sq = len_sq(dr);
            p.a[i] += (p.m[j] / (r_sq * std::sqrt(r_sq))) * dr;
        }
    }
}

inline void run_leapfrog(Particles& p, PKF& pkf, usize N) {
    pkf.particles.resize(N);

    for (usize i = 0; i < N; i++) {
        pkf.particles[i].push_back(Keyframe{p.s[i], 0});
    }

    const f32 epsilon_sq = epsilon * epsilon;
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

            if (len_sq(p.s[i] - pkf.particles[i].back().s) >= epsilon_sq) {
                pkf.particles[i].push_back(Keyframe{p.s[i], nt});
            }
        }
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

    std::cout << pkf.particles[0].size() << ", " << p.s.back().z;

    write_pkf("sim.pkff", pkf);

    return 0;
}