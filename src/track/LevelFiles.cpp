#include "track/LevelFiles.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iterator>

namespace tf {

namespace {

// Next header token, skipping white space and # comments.
bool token(const std::string& s, std::size_t& pos, std::string& out) {
    while (pos < s.size()) {
        if (s[pos] == '#') {
            while (pos < s.size() && s[pos] != '\n') ++pos;
        } else if (std::isspace(static_cast<unsigned char>(s[pos]))) {
            ++pos;
        } else {
            break;
        }
    }
    const std::size_t start = pos;
    while (pos < s.size() && !std::isspace(static_cast<unsigned char>(s[pos])) && s[pos] != '#') ++pos;
    out = s.substr(start, pos - start);
    return !out.empty();
}

} // namespace

bool readPgm(const std::string& path, PgmImage& out, std::string& error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        error = "cannot open " + path;
        return false;
    }
    const std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::size_t pos = 0;
    std::string magic, w, h, m;
    if (!token(s, pos, magic) || magic != "P5" || !token(s, pos, w) || !token(s, pos, h) || !token(s, pos, m)) {
        error = path + ": not a binary PGM (P5)";
        return false;
    }
    out.width = std::atoi(w.c_str());
    out.height = std::atoi(h.c_str());
    out.maxValue = std::atoi(m.c_str());
    if (out.width <= 0 || out.height <= 0 || out.width > 16384 || out.height > 16384 || out.maxValue <= 0 ||
        out.maxValue > 65535) {
        error = path + ": bad PGM header";
        return false;
    }
    ++pos; // the single white space character after maxval
    const std::size_t bytesPer = out.maxValue > 255 ? 2 : 1;
    const std::size_t count = std::size_t(out.width) * std::size_t(out.height);
    if (s.size() < pos + count * bytesPer) {
        error = path + ": truncated PGM data";
        return false;
    }
    out.data.resize(count);
    const unsigned char* p = reinterpret_cast<const unsigned char*>(s.data() + pos);
    for (std::size_t i = 0; i < count; ++i)
        out.data[i] = bytesPer == 2 ? std::uint16_t((p[2 * i] << 8) | p[2 * i + 1]) : std::uint16_t(p[i]);
    return true;
}

bool writePgm(const std::string& path, const PgmImage& image, std::string& error) {
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        error = "cannot write " + path;
        return false;
    }
    f << "P5\n" << image.width << " " << image.height << "\n" << image.maxValue << "\n";
    const bool wide = image.maxValue > 255;
    std::string bytes;
    bytes.reserve(image.data.size() * (wide ? 2 : 1));
    for (std::uint16_t v : image.data) {
        if (wide) bytes.push_back(char(v >> 8));
        bytes.push_back(char(v & 0xff));
    }
    f.write(bytes.data(), std::streamsize(bytes.size()));
    if (!f) {
        error = "write failed: " + path;
        return false;
    }
    return true;
}

const char* levelSurfaceClassName(LevelSurfaceClass c) {
    static const char* const names[] = {"asphalt", "gravel", "dirt", "mud", "grass", "sand", "rock", "snow"};
    static_assert(std::size(names) == std::size_t(LevelSurfaceClass::Count));
    return names[std::min(std::size_t(c), std::size(names) - 1)];
}

} // namespace tf
