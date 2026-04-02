#pragma once

#include "pkff.h"

const f64 G = 6.6743015e-11;
const f64 M = 1.5e39;
const usize n = 8;
const f64 init_d = 1e21; 
const f32 dt = 3.156e14; // 10M years
const u32 num_dt = 1.38e2 * 20; //13.8 * 20 billion years
const f32 epsilon = init_d * 0.015;

struct Particles {
    Vec<R3> s;
    Vec<R3> v;
    Vec<R3> a;
    Vec<f64> m;
};