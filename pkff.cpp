#include <fstream>

#include "pkff.h"

template<typename T>
void write_raw(std::ofstream& os, const T& data) {
    os.write(reinterpret_cast<const char*>(&data), sizeof(data));
}
void write_pkf(std::ofstream& os, const std::string& filename, const PKF& pkf) {
    os.open(filename, std::ios::binary);
    u32 num_particles = pkf.particles.size();
    write_raw(os, num_particles);
    write_raw(os, pkf.dt);
    write_raw(os, pkf.num_dt);

    for (const Vec<Keyframe>& p : pkf.particles) {
        u16 num_kf = static_cast<u16>(p.size());
        write_raw(os, num_kf);
        os.write(reinterpret_cast<const char*>(p.data()), num_kf * sizeof(Keyframe));
    }
    os.close();
}

template<typename T>
void read_raw(std::ifstream& is, T& data) {
    is.read(reinterpret_cast<char*>(&data), sizeof(data));
}
void read_pkf(std::ifstream& is, const std::string& filename, PKF& pkf) {
    is.open(filename, std::ios::binary);
    u32 num_particles;
    read_raw(is, num_particles);
    read_raw(is, pkf.dt);
    read_raw(is, pkf.num_dt);

    pkf.particles.resize(num_particles);
    for (Vec<Keyframe>& p : pkf.particles) {
        u16 num_kf;
        read_raw(is, num_kf);
        p.resize(num_kf);
        is.read(reinterpret_cast<char*>(p.data()), num_kf * sizeof(Keyframe));
    }
    is.close();
}

