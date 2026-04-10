#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <iostream>
#include <cmath>

#include "sim.h"
#include "octree.h"

const f32 theta = 0.6;

static Vec<R3> gaussian_centers;
static Vec<f64> gaussian_amplitudes;
static Vec<f64> gaussian_widths_parallel;
static Vec<f64> gaussian_widths_perp;
static Vec<R3> gaussian_directions; 
static bool field_initialized = false;

inline void init_random_field() {
    if (field_initialized) return;
    
    const int num_filaments = 4;
    const f64 box_size = init_d * 2.0;
    
    gaussian_centers.resize(num_filaments);
    gaussian_amplitudes.resize(num_filaments);
    gaussian_widths_parallel.resize(num_filaments);
    gaussian_widths_perp.resize(num_filaments);
    gaussian_directions.resize(num_filaments);
    
    srand(static_cast<unsigned int>(time(NULL)));
    
    for (int i = 0; i < num_filaments; ++i) {
        gaussian_centers[i].x = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * box_size * 0.5;
        gaussian_centers[i].y = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * box_size * 0.5;
        gaussian_centers[i].z = (static_cast<f64>(rand()) / RAND_MAX - 0.5) * box_size * 0.5;
        
        f64 theta = (static_cast<f64>(rand()) / RAND_MAX) * 2.0 * M_PI;
        f64 phi = (static_cast<f64>(rand()) / RAND_MAX) * M_PI;
        gaussian_directions[i].x = std::sin(phi) * std::cos(theta);
        gaussian_directions[i].y = std::sin(phi) * std::sin(theta);
        gaussian_directions[i].z = std::cos(phi);
        
        gaussian_widths_parallel[i] = init_d * (0.3 + static_cast<f64>(rand()) / RAND_MAX * 0.7);
        gaussian_widths_perp[i] = init_d * (0.05 + static_cast<f64>(rand()) / RAND_MAX * 0.1);
        gaussian_amplitudes[i] = 0.8 + static_cast<f64>(rand()) / RAND_MAX * 0.2;
    }
    
    field_initialized = true;
}

inline f64 anisotropic_gaussian(const R3& pos, const R3& center, const R3& dir, f64 w_par, f64 w_perp) {
    const R3 delta = pos - center;
    const f64 par = delta.x * dir.x + delta.y * dir.y + delta.z * dir.z;
    const f64 par_sq = par * par;
    const f64 perp_sq = len_sq(delta) - par_sq;
    
    return std::exp(-par_sq / (w_par * w_par) - perp_sq / (w_perp * w_perp));
}

inline f64 density_field(const R3& pos) {
    init_random_field();
    
    f64 density = 0.0;
    for (size_t i = 0; i < gaussian_centers.size(); ++i) {
        density += gaussian_amplitudes[i] * anisotropic_gaussian(pos, gaussian_centers[i], gaussian_directions[i], gaussian_widths_parallel[i], gaussian_widths_perp[i]);
    }
    
    const f64 max_density = gaussian_centers.size();
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

inline void build_octree(Octree& oct, const Particles& p, usize N) {
    //find bounds
    R3 min_pos = p.s[0];
    R3 max_pos = p.s[0];
    
    for (usize i = 1; i < N; i++) {
        min_pos.x = std::min(min_pos.x, p.s[i].x);
        min_pos.y = std::min(min_pos.y, p.s[i].y);
        min_pos.z = std::min(min_pos.z, p.s[i].z);
        max_pos.x = std::max(max_pos.x, p.s[i].x);
        max_pos.y = std::max(max_pos.y, p.s[i].y);
        max_pos.z = std::max(max_pos.z, p.s[i].z);
    }
    
    R3 center = 0.5 * (min_pos + max_pos);
    f64 half_size = 0.5 * (max_pos.x - min_pos.x);
    half_size = std::max(half_size, 0.5 * (max_pos.y - min_pos.y));
    half_size = std::max(half_size, 0.5 * (max_pos.z - min_pos.z));
    half_size *= 1.5; //margin
    
    oct.scale = half_size;
    oct.root = new Node{{nullptr}, nullptr, 0.0, center};
    
    for (usize i = 0; i < N; i++) {
        push_node(oct, p.s[i], p.m[i]);
    }
}

inline void compute_force(const Node* node, const R3& pos, f64 mass, R3& force, const R3& node_center, f64 node_scale) {
    if (!node) return;
    if (node->m == 0.0) return;
    
    const R3 delta = node->pos - pos;
    const f64 r_sq = len_sq(delta);
    
    if (r_sq < 1e3) return; 
    
    const f64 r = std::sqrt(r_sq);
    
    if (is_leaf(node) || (2.0 * node_scale / r) < theta) {
        const f64 r_sq_soft = r_sq + softening * softening;
        force += (node->m / (r_sq_soft * std::sqrt(r_sq_soft))) * delta;
    } else {
        for (int i = 0; i < 8; i++) {
            if (node->children[i]) {
                f64 child_scale = node_scale / 2.0;
                R3 child_center = node_center;
                child_center.x += (i & 4 ? 1 : -1) * child_scale;
                child_center.y += (i & 2 ? 1 : -1) * child_scale;
                child_center.z += (i & 1 ? 1 : -1) * child_scale;
                compute_force(node->children[i], pos, mass, force, child_center, child_scale);
            }
        }
    }
}

inline void apply_grav_bh(Particles& p, const Octree& oct, usize N) {
    for (usize i = 0; i < N; i++) {
        p.a[i] = {0.0, 0.0, 0.0};
    }
    for (usize i = 0; i < N; i++) {
        R3 force = {0.0, 0.0, 0.0};
        if (oct.root) {
            compute_force(oct.root, p.s[i], p.m[i], force, oct.root->pos, oct.scale);
        }

        p.a[i] = force;
    }
}

inline void free_octree(Node* node) {
    if (!node) return;
    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            free_octree(node->children[i]);
        }
    }
    delete node;
}

inline void run_leapfrog_bh(Particles& p, PKF& pkf, usize N) {
    pkf.particles.resize(N);

    for (usize i = 0; i < N; i++) {
        pkf.particles[i].push_back(Keyframe{toR3s(p.s[i]), 0});
    }

    Octree oct;
    build_octree(oct, p, N);
    apply_grav_bh(p, oct, N);
    
    Vec<R3> old_pos = p.s;
    const f64 k = 0.5 * G * dt;
    const f64 leaf_drift_threshold = oct.scale * 0.1;
    
    for (u32 nt = 0; nt < num_dt; nt++) {
        old_pos = p.s;
        
        for (usize i = 0; i < N; i++) {
            p.v[i] += k * p.a[i];
            p.s[i] += dt * p.v[i];
        }

        for (usize i = 0; i < N; i++) {
            update_particle(oct, old_pos[i], p.s[i], p.m[i]);
        }
        
        apply_grav_bh(p, oct, N);

        for (usize i = 0; i < N; i++) {
            p.v[i] += k * p.a[i];

            const f64 vel_magnitude = std::sqrt(len_sq(p.v[i]));
            const f64 acc_magnitude = std::sqrt(len_sq(p.a[i]));
            const f64 direction_change_factor = acc_magnitude / (vel_magnitude + 1e-10);
            const f64 dynamic_epsilon = epsilon / (1.0 + direction_change_scale * direction_change_factor);
            const f64 dynamic_epsilon_sq = dynamic_epsilon * dynamic_epsilon;

            if (len_sq(p.s[i] - toR3(pkf.particles[i].back().s)) >= dynamic_epsilon_sq) {
                pkf.particles[i].push_back(Keyframe{toR3s(p.s[i]), nt});
            }
        }
        if ((nt + 1) % 100 == 0) {
            free_octree(oct.root);
            build_octree(oct, p, N);
        }
        
        std::cout << nt << "/" << num_dt << std::endl; 
    }
    
    free_octree(oct.root);
}

int main() {
    Particles p; 
    PKF pkf;

    pkf.dt = dt;
    pkf.num_dt = num_dt;

    const usize N = n * n * n;
    init_particles(p, N);

    run_leapfrog_bh(p, pkf, N);

    std::cout << pkf.particles[0].size() << std::endl;
    std::cout << pkf.particles[1].size() << std::endl;
    std::cout << pkf.particles[2].size() << std::endl;

    write_pkf("sim2.pkff", pkf);

    return 0;
}
