#include "core/TerrainTextureSet.h"

#include <quench/core/Json.h>

#include <fstream>
#include <sstream>

namespace tf {

namespace json = quench::json;

namespace {
const char* const kLayerOrder[TerrainTextureSet::kLayers] = {"grass", "soil", "road", "rock", "sand", "snow"};
}

bool loadTerrainTextureSet(const std::string& dir, TerrainTextureSet& out, std::string& error) {
    const std::string path = dir + "/terrain.json";
    std::ifstream in(path);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    const quench::Result<json::Value> parsed = json::parse(ss.str());
    if (!parsed) {
        error = path + ": " + parsed.error();
        return false;
    }
    const json::Value& doc = *parsed;
    out = TerrainTextureSet{};
    out.albedoSize = std::uint32_t(doc["albedoSize"].numberOr(0));
    out.nraSize = std::uint32_t(doc["nraSize"].numberOr(0));
    const json::Value& layers = doc["layers"];
    if (!layers.isArray() || layers.array.size() != TerrainTextureSet::kLayers || out.albedoSize == 0 || out.nraSize == 0) {
        error = path + ": needs albedoSize, nraSize and " + std::to_string(TerrainTextureSet::kLayers) + " layers";
        return false;
    }
    for (std::size_t i = 0; i < layers.array.size(); ++i) {
        TerrainTextureSet::Layer l;
        l.id = layers.array[i]["id"].stringOr("");
        l.tile = float(layers.array[i]["tile"].numberOr(0));
        if (l.id != kLayerOrder[i] || !(l.tile > 0.05f)) {
            error = path + ": layer " + std::to_string(i) + " must be '" + kLayerOrder[i] + "' with a tile > 0.05 m";
            return false;
        }
        const json::Value& tint = layers.array[i]["tint"];
        if (tint.isArray() && tint.array.size() == 3)
            for (int c = 0; c < 3; ++c) l.tint[c] = float(tint.array[std::size_t(c)].numberOr(1.0));
        l.albedoPath = dir + "/" + l.id + "_albedo.ktx2";
        l.nraPath = dir + "/" + l.id + "_nra.ktx2";
        out.layers.push_back(std::move(l));
    }
    return true;
}

std::uint32_t terrainTextureSkip(std::uint32_t stored, std::uint32_t wanted) {
    std::uint32_t skip = 0;
    while ((stored >> skip) > wanted && (stored >> (skip + 1)) >= 4) ++skip;
    return skip;
}

bool loadTerrainTextureMap(const std::string& path, bool srgb, std::uint32_t skip, ktx2::Image& out,
                           std::string& error) {
    if (!ktx2::readFile(path, out, error, skip)) return false;
    const std::uint32_t want = srgb ? ktx2::kVkFormatBc7Srgb : ktx2::kVkFormatBc7Unorm;
    if (out.vkFormat != want || out.width != out.height) {
        error = path + ": expected a square BC7 " + (srgb ? "sRGB" : "UNORM") + " texture";
        return false;
    }
    return true;
}

} // namespace tf
