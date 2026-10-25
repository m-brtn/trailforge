#include "app/GameSettings.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace tf {
namespace {

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

bool parseNumber(const std::string& v, double& out) {
    if (v.empty()) return false;
    char* end = nullptr;
    out = std::strtod(v.c_str(), &end);
    return end && *end == '\0' && std::isfinite(out);
}

float volume(float v) { return std::isfinite(v) ? std::clamp(v, 0.0f, 1.0f) : 1.0f; }

} // namespace

GameSettings sanitized(GameSettings s) {
    s.quality = std::clamp(s.quality, 0, 3);
    if (!std::isfinite(s.renderScale) || s.renderScale < 0.5f) s.renderScale = 0.0f;
    s.renderScale = std::min(s.renderScale, 1.0f);
    s.masterVolume = volume(s.masterVolume);
    s.musicVolume = volume(s.musicVolume);
    s.effectsVolume = volume(s.effectsVolume);
    s.uiVolume = volume(s.uiVolume);
    s.traction = std::clamp(s.traction, 0, 2);
    s.diffLocks = std::clamp(s.diffLocks, 0, 3);
    s.driveKeys = std::clamp(s.driveKeys, 0, 1);
    s.padGlyphs = std::clamp(s.padGlyphs, 0, 2);
    const bool idOk = s.vehicle.size() <= 64 && std::all_of(s.vehicle.begin(), s.vehicle.end(), [](char c) {
        return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
    });
    if (!idOk) s.vehicle.clear();
    return s;
}

std::string serializeSettings(const GameSettings& in) {
    const GameSettings s = sanitized(in);
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
                  "# TrailForge settings (written by the game; unknown keys are ignored)\n"
                  "quality = %d\n"
                  "render_scale = %.2f\n"
                  "vsync = %d\n"
                  "volume_master = %.2f\n"
                  "volume_music = %.2f\n"
                  "volume_effects = %.2f\n"
                  "volume_ui = %.2f\n"
                  "abs = %d\n"
                  "traction = %d\n"
                  "diff_locks = %d\n"
                  "auto_reverse = %d\n"
                  "drive_keys = %d\n"
                  "pad_glyphs = %d\n",
                  s.quality, double(s.renderScale), s.vsync ? 1 : 0, double(s.masterVolume), double(s.musicVolume),
                  double(s.effectsVolume), double(s.uiVolume), s.abs ? 1 : 0, s.traction, s.diffLocks,
                  s.autoReverse ? 1 : 0, s.driveKeys, s.padGlyphs);
    std::string out = buf;
    if (!s.vehicle.empty()) out += "vehicle = " + s.vehicle + "\n";
    return out;
}

GameSettings parseSettings(const std::string& text, int* skipped) {
    GameSettings s;
    int bad = 0;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        const auto hash = line.find('#');
        if (hash != std::string::npos) line.resize(hash);
        line = trim(line);
        if (line.empty()) continue;
        const auto eq = line.find('=');
        if (eq != std::string::npos && trim(line.substr(0, eq)) == "vehicle") {
            s.vehicle = trim(line.substr(eq + 1));
            if (sanitized(s).vehicle != s.vehicle) {
                s.vehicle.clear();
                ++bad;
            }
            continue;
        }
        double v = 0.0;
        if (eq == std::string::npos || !parseNumber(trim(line.substr(eq + 1)), v)) {
            ++bad;
            continue;
        }
        const std::string k = trim(line.substr(0, eq));
        const int i = int(std::lround(std::clamp(v, -1e6, 1e6)));
        if (k == "quality") s.quality = i;
        else if (k == "render_scale") s.renderScale = float(v);
        else if (k == "vsync") s.vsync = i != 0;
        else if (k == "volume_master") s.masterVolume = float(v);
        else if (k == "volume_music") s.musicVolume = float(v);
        else if (k == "volume_effects") s.effectsVolume = float(v);
        else if (k == "volume_ui") s.uiVolume = float(v);
        else if (k == "abs") s.abs = i != 0;
        else if (k == "traction") s.traction = i;
        else if (k == "diff_locks") s.diffLocks = i;
        else if (k == "auto_reverse") s.autoReverse = i != 0;
        else if (k == "drive_keys") s.driveKeys = i;
        else if (k == "pad_glyphs") s.padGlyphs = i;
        else ++bad;
    }
    if (skipped) *skipped = bad;
    return sanitized(s);
}

bool saveSettingsFile(const std::string& path, const GameSettings& s, std::string* error) {
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) {
            if (error) *error = "cannot write " + tmp;
            return false;
        }
        out << serializeSettings(s);
        if (!out.flush()) {
            if (error) *error = "write failed: " + tmp;
            return false;
        }
    }
    std::remove(path.c_str()); // rename() does not replace an existing file on Windows
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        if (error) *error = "cannot rename " + tmp + " -> " + path;
        return false;
    }
    return true;
}

bool loadSettingsFile(const std::string& path, GameSettings& out, int* skipped) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::ostringstream ss;
    ss << in.rdbuf();
    out = parseSettings(ss.str(), skipped);
    return true;
}

} // namespace tf
