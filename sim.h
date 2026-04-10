#pragma once

#include "pkff.h"

const f64 G = 6.6743015e-11;
const f64 M = 1e42;
const usize n = 50;
const f64 init_d = 2.5e21; 
const f32 dt = 3.156e12; // 10M years
const u32 num_dt = 5e2; //13.8 billion years

const f64 epsilon = init_d * 0.1;
const f64 direction_change_scale = 0.8;
const f64 softening = init_d * 0.5; // gravitational softening length

struct Particles {
    Vec<R3> s;
    Vec<R3> v;
    Vec<R3> a;
    Vec<f64> m;
};