#include "core/ModelAsset.h"

#include "core/Json.h"

#define CGLTF_IMPLEMENTATION
#include <cgltf.h>

#include <cstring>
#include <memory>

namespace tf {

namespace {

struct DataDeleter {
    void operator()(cgltf_data* d) const { cgltf_free(d); }
};

const cgltf_accessor* attribute(const cgltf_primitive& p, cgltf_attribute_type type, const char* name = nullptr) {
    for (cgltf_size a = 0; a < p.attributes_count; ++a) {
        const cgltf_attribute& at = p.attributes[a];
        if (at.type == type && (!name || (at.name && std::strcmp(at.name, name) == 0))) return at.data;
    }
    return nullptr;
}

std::string imageName(const cgltf_texture_view& v) {
    if (!v.texture || !v.texture->image || !v.texture->image->uri) return {};
    return v.texture->image->uri;
}

std::array<float, 3> vec3(const json::Value& v) {
    std::array<float, 3> out{};
    for (std::size_t i = 0; i < 3 && i < v.array.size(); ++i) out[i] = float(v.array[i].number);
    return out;
}

} // namespace

bool ModelAsset::load(const std::string& path, bool withMeshes, std::string* error) {
    auto fail = [&](const std::string& msg) {
        if (error) *error = path + ": " + msg;
        return false;
    };
    cgltf_options opts{};
    cgltf_data* raw = nullptr;
    if (cgltf_parse_file(&opts, path.c_str(), &raw) != cgltf_result_success) return fail("cannot parse");
    std::unique_ptr<cgltf_data, DataDeleter> data(raw);
    if (cgltf_load_buffers(&opts, raw, path.c_str()) != cgltf_result_success) return fail("cannot load buffers");

    json::Value extras;
    std::string err;
    if (!raw->extras.data || !json::parse(raw->extras.data, extras, &err)) return fail("no model extras " + err);
    kind = extras["kind"].stringOr("");
    height = float(extras["height"].numberOr(0.0));
    radius = float(extras["radius"].numberOr(0.0));
    boundsMin = vec3(extras["boundsMin"]);
    boundsMax = vec3(extras["boundsMax"]);
    trunkRadius = float(extras["trunkRadius"].numberOr(0.0));
    trunkTop = vec3(extras["trunkTop"]);
    lodError.clear();
    for (const json::Value& v : extras["lodError"].array) lodError.push_back(float(v.number));
    collisionLod = int(extras["collisionLod"].numberOr(-1.0));
    surface = extras["surface"].stringOr("");
    impostorAlbedo = extras["impostor"]["albedo"].stringOr("");
    impostorNra = extras["impostor"]["nra"].stringOr("");
    impostorViews = int(extras["impostor"]["views"].numberOr(0.0));
    id = raw->nodes_count > 0 && raw->nodes[0].name ? raw->nodes[0].name : "";
    if (height <= 0.0f || radius <= 0.0f) return fail("missing height / radius");

    materials.clear();
    for (cgltf_size m = 0; m < raw->materials_count; ++m) {
        const cgltf_material& src = raw->materials[m];
        ModelMaterial mat;
        mat.name = src.name ? src.name : "";
        mat.albedo = imageName(src.pbr_metallic_roughness.base_color_texture);
        mat.nra = imageName(src.normal_texture);
        mat.alphaTest = src.alpha_mode == cgltf_alpha_mode_mask;
        if (mat.albedo.empty() || mat.nra.empty()) return fail("material " + mat.name + " without maps");
        materials.push_back(mat);
    }

    lods.clear();
    collisionPositions.clear();
    collisionIndices.clear();
    for (cgltf_size mi = 0; mi < raw->meshes_count; ++mi) {
        const cgltf_mesh& mesh = raw->meshes[mi];
        const std::string name = mesh.name ? mesh.name : "";
        const bool collision = name == "collision";
        if (!collision && (name.rfind("lod", 0) != 0 || !withMeshes)) continue;
        if (!collision && name != "lod" + std::to_string(lods.size())) return fail("meshes out of order: " + name);
        ModelLod lod;
        for (cgltf_size pi = 0; pi < mesh.primitives_count; ++pi) {
            const cgltf_primitive& prim = mesh.primitives[pi];
            const cgltf_accessor* pos = attribute(prim, cgltf_attribute_type_position);
            if (!pos || !prim.indices) return fail(name + ": primitive without positions or indices");
            const std::size_t n = pos->count;
            std::vector<float> P(n * 3);
            cgltf_accessor_unpack_floats(pos, P.data(), P.size());
            if (collision) {
                const auto base = std::uint32_t(collisionPositions.size() / 3);
                collisionPositions.insert(collisionPositions.end(), P.begin(), P.end());
                for (cgltf_size k = 0; k < prim.indices->count; ++k)
                    collisionIndices.push_back(base + std::uint32_t(cgltf_accessor_read_index(prim.indices, k)));
                continue;
            }
            const cgltf_accessor* nrm = attribute(prim, cgltf_attribute_type_normal);
            const cgltf_accessor* uv = attribute(prim, cgltf_attribute_type_texcoord);
            const cgltf_accessor* wind = attribute(prim, cgltf_attribute_type_custom, "_WIND");
            if (!nrm || !uv || !wind || !prim.material) return fail(name + ": primitive without attributes");
            std::vector<float> N(n * 3), UV(n * 2), W(n * 2);
            cgltf_accessor_unpack_floats(nrm, N.data(), N.size());
            cgltf_accessor_unpack_floats(uv, UV.data(), UV.size());
            cgltf_accessor_unpack_floats(wind, W.data(), W.size());
            const auto base = std::uint32_t(lod.vertices.size());
            for (std::size_t i = 0; i < n; ++i) {
                ModelVertex v{};
                std::memcpy(v.pos, &P[i * 3], sizeof v.pos);
                std::memcpy(v.normal, &N[i * 3], sizeof v.normal);
                std::memcpy(v.uv, &UV[i * 2], sizeof v.uv);
                std::memcpy(v.wind, &W[i * 2], sizeof v.wind);
                lod.vertices.push_back(v);
            }
            ModelPrimitive p;
            p.firstIndex = std::uint32_t(lod.indices.size());
            p.indexCount = std::uint32_t(prim.indices->count);
            p.material = int(prim.material - raw->materials);
            for (cgltf_size k = 0; k < prim.indices->count; ++k)
                lod.indices.push_back(base + std::uint32_t(cgltf_accessor_read_index(prim.indices, k)));
            lod.primitives.push_back(p);
        }
        if (!collision) lods.push_back(std::move(lod));
    }
    if (withMeshes && lods.empty()) return fail("no lod meshes");
    return true;
}

} // namespace tf
