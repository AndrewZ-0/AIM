#pragma once

#include <vector>

//rust-style type names
#define u32 uint32_t
#define u16 uint16_t
#define u8 uint8_t
#define usize size_t
#define f64 double
#define f32 float
#define Vec std::vector

struct R3 {
    f64 x, y, z;
};

inline R3 operator*(const f64 k, R3 v) {
    v.x *= k;
    v.y *= k;
    v.z *= k;
    return v;
}
inline R3 operator+(R3 v1, const R3& v2) {
    v1.x += v2.x;
    v1.y += v2.y;
    v1.z += v2.z;
    return v1;
}
inline R3 operator-(R3 v1, const R3& v2) {
    v1.x -= v2.x;
    v1.y -= v2.y;
    v1.z -= v2.z;
    return v1;
}
inline void operator+=(R3& v1, const R3& v2) {
    v1.x += v2.x;
    v1.y += v2.y;
    v1.z += v2.z;
}
inline void operator*=(R3& v, const f64 k) {
    v.x *= k;
    v.y *= k;
    v.z *= k;
}
inline void operator/=(R3& v, const f64 k) {
    v.x /= k;
    v.y /= k;
    v.z /= k;
}
inline bool operator==(const R3& v1, const R3& v2) {
    return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}

inline f64 len_sq(const R3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

struct R3s {
    f32 x, y, z;
};

inline const R3 toR3(const R3s& v) {
    return {
        static_cast<f64>(v.x), 
        static_cast<f64>(v.y), 
        static_cast<f64>(v.z)
    };
}
inline const R3s toR3s(const R3& v) {
    return {
        static_cast<f32>(v.x), 
        static_cast<f32>(v.y), 
        static_cast<f32>(v.z)
    };
}


inline f32 len_sq(const R3s& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}