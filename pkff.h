#pragma once

#include <cstdint>

#include "rusty.h"

struct Keyframe {
    R3s s;
    u32 dt_skip;
};

struct PKF {
    f32 dt; //time step
    u32 num_dt;

    Vec<Vec<Keyframe>> particles;
};

template<typename T>
inline void write_raw(std::ofstream& os, const T& data);
void write_pkf(const std::string& filename, const PKF& pkf);

template<typename T>
inline void read_raw(std::ifstream& is, T& data);
void read_pkf(const std::string& filename, PKF& pkf);