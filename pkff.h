#pragma once

#include <cstdint>
#include <vector>

#define u32 uint32_t
#define u16 uint16_t
#define f32 float
#define Vec std::vector

struct Keyframe {
    f32 x, y, z;
    u16 dt_skip;
};

struct PKF {
    f32 dt; //time step
    u32 num_dt;

    Vec<Vec<Keyframe>> particles;
};

template<typename T>
inline void write_raw(std::ofstream& os, const T& data);
void write_pkf(std::ofstream& os, const std::string& filename, const PKF& pkf);

template<typename T>
inline void read_raw(std::ifstream& is, T& data);
void read_pkf(std::ifstream& is, const std::string& filename, PKF& pkf);