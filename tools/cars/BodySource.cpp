#include "BodySource.h"

#include "core/Json.h"

#include <cgltf.h>
#include <meshoptimizer.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>

namespace tf {
namespace {

struct DataDeleter {
    void operator()(cgltf_data* d) const { cgltf_free(d); }
};

json::Value extrasOf(const cgltf_extras& e) {
    json::Value v;
    if (e.data) json::parse(e.data, v);
    return v;
}

bool startsWith(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }

const cgltf_accessor* attribute(const cgltf_primitive& p, cgltf_attribute_type type) {
    for (cgltf_size a = 0; a < p.attributes_count; ++a)
        if (p.attributes[a].type == type && p.attributes[a].index == 0) return p.attributes[a].data;
    return nullptr;
}

bool hasToken(const std::string& name, std::initializer_list<const char*> tokens) {
    for (const char* t : tokens)
        if (name.find(t) != std::string::npos) return true;
    return false;
}

class Reader {
public:
    Reader(const cgltf_data* d, BodyModel& m, std::vector<std::string>& e, std::vector<std::string>& w)
        : data(d), model(m), errors(e), warnings(w) {}

    void visit(const cgltf_node& node, const std::string& parentPart) {
        const std::string name = node.name ? node.name : "";
        for (const char* p : {"REF_", "PROXY_", "CUT_"})
            if (startsWith(name, p)) {
                warnings.push_back("skipped helper node '" + name + "' (not exported to the game)");
                return;
            }
        float M[16];
        cgltf_node_transform_world(&node, M);
        const json::Value x = extrasOf(node.extras);
        std::string childParent = parentPart;
        if (startsWith(name, "COL_")) {
            if (!node.mesh) errors.push_back(name + ": collision piece without a mesh");
            else hull(name, *node.mesh, M);
            return;
        }
        if (startsWith(name, "SPARE_")) {
            BodySpare s;
            s.name = name.substr(6);
            s.parent = parentPart;
            s.seat = {M[12], M[13], M[14]};
            // The empty's local +Y (Blender's local +Z, its arrow) points away from the body.
            const float o[3] = {M[4], M[5], M[6]};
            const float len = std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
            if (len > 1e-6f) s.outward = {o[0] / len, o[1] / len, o[2] / len};
            model.spares.push_back(s);
            return;
        }
        if (node.mesh) {
            std::string base = name;
            int lod = 0;
            for (int k : {1, 2}) {
                const std::string suffix = "_LOD" + std::to_string(k);
                if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
                    base = name.substr(0, name.size() - suffix.size());
                    lod = k;
                }
            }
            BodyPart& part = partNamed(base, parentPart, x);
            if (part.lods.size() <= std::size_t(lod)) part.lods.resize(std::size_t(lod) + 1);
            mesh(*node.mesh, M, part.lods[std::size_t(lod)], name);
            if (lod == 0) childParent = base;
        } else if (!name.empty() && node.children_count == 0) {
            warnings.push_back("empty node '" + name + "' ignored");
        }
        for (cgltf_size c = 0; c < node.children_count; ++c) visit(*node.children[c], childParent);
    }

    const cgltf_data* data;
    BodyModel& model;
    std::vector<std::string>& errors;
    std::vector<std::string>& warnings;
    std::map<const cgltf_material*, int> materials;

private:
    BodyPart& partNamed(const std::string& name, const std::string& parent, const json::Value& x) {
        for (BodyPart& p : model.parts)
            if (p.name == name) return p;
        BodyPart p;
        p.name = name;
        p.parent = parent;
        const bool fixed = name == "body" || name == "body_shell" || name == "chassis" || name == "underbody" ||
                           name == "cabin" || name == "liner";
        p.deformable = x["tf_deformable"].boolOr(!hasToken(name, {"glass", "light", "lamp", "lens", "mirror"}));
        p.detachable = x["tf_detachable"].boolOr(!fixed);
        p.collides = x["tf_collides"].boolOr(!hasToken(name, {"mirror", "antenna"}));
        attachGiven.push_back(x["tf_attach"].isArray() && x["tf_attach"].array.size() == 3);
        if (attachGiven.back())
            for (std::size_t i = 0; i < 3; ++i) p.attach[i] = float(x["tf_attach"].array[i].numberOr(0));
        model.parts.push_back(p);
        return model.parts.back();
    }

public:
    std::vector<bool> attachGiven;

private:
    int material(const cgltf_material* src) {
        if (auto it = materials.find(src); it != materials.end()) return it->second;
        BodyMaterial m;
        m.name = src && src->name ? src->name : "";
        const json::Value x = src ? extrasOf(src->extras) : json::Value();
        std::string role = x["tf_role"].stringOr("");
        if (role.empty()) role = m.name.substr(0, m.name.find('.'));
        if (!bodyRoleFromName(role, m.role)) {
            errors.push_back("material '" + m.name + "': no role (name it, or set the custom property tf_role, to one of: " +
                             bodyRoleList() + ")");
        }
        if (src) {
            const cgltf_pbr_metallic_roughness& pbr = src->pbr_metallic_roughness;
            m.color = {pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2]};
            m.alpha = pbr.base_color_factor[3];
            m.roughness = pbr.roughness_factor;
            m.metallic = pbr.metallic_factor;
            m.emissive = {src->emissive_factor[0], src->emissive_factor[1], src->emissive_factor[2]};
            if (src->has_emissive_strength) m.emissiveStrength = src->emissive_strength.emissive_strength;
            if (src->has_clearcoat) {
                m.clearcoat = src->clearcoat.clearcoat_factor;
                m.clearcoatRoughness = src->clearcoat.clearcoat_roughness_factor;
            }
            if (src->has_transmission) m.transmission = src->transmission.transmission_factor;
            if (src->has_ior) m.ior = src->ior.ior;
            if (pbr.base_color_texture.texture || pbr.metallic_roughness_texture.texture || src->normal_texture.texture)
                warnings.push_back("material '" + m.name + "': textures are ignored (vehicle bodies use material and vertex colours)");
        }
        m.mud = float(x["tf_mud"].numberOr(bodyRoleDefaultMud(m.role)));
        model.materials.push_back(m);
        return materials[src] = int(model.materials.size()) - 1;
    }

    static void transform(const float* M, const float* p, float* out, float w) {
        for (int r = 0; r < 3; ++r) out[r] = M[r] * p[0] + M[4 + r] * p[1] + M[8 + r] * p[2] + M[12 + r] * w;
    }

    void mesh(const cgltf_mesh& mesh, const float* M, BodyMesh& out, const std::string& node) {
        // Normal matrix: inverse transpose of the upper 3x3 (cofactors; the scale does not matter after normalizing).
        const float a[9] = {M[0], M[1], M[2], M[4], M[5], M[6], M[8], M[9], M[10]}; // columns
        float nm[9];
        nm[0] = a[4] * a[8] - a[5] * a[7], nm[1] = a[5] * a[6] - a[3] * a[8], nm[2] = a[3] * a[7] - a[4] * a[6];
        nm[3] = a[2] * a[7] - a[1] * a[8], nm[4] = a[0] * a[8] - a[2] * a[6], nm[5] = a[1] * a[6] - a[0] * a[7];
        nm[6] = a[1] * a[5] - a[2] * a[4], nm[7] = a[2] * a[3] - a[0] * a[5], nm[8] = a[0] * a[4] - a[1] * a[3];
        const float det = a[0] * nm[0] + a[1] * nm[1] + a[2] * nm[2];
        for (cgltf_size pi = 0; pi < mesh.primitives_count; ++pi) {
            const cgltf_primitive& prim = mesh.primitives[pi];
            if (prim.type != cgltf_primitive_type_triangles) {
                errors.push_back(node + ": only triangle primitives are supported");
                continue;
            }
            const cgltf_accessor* pos = attribute(prim, cgltf_attribute_type_position);
            if (!pos) {
                errors.push_back(node + ": primitive without positions");
                continue;
            }
            const cgltf_accessor* nrm = attribute(prim, cgltf_attribute_type_normal);
            const cgltf_accessor* col = attribute(prim, cgltf_attribute_type_color);
            const std::size_t n = pos->count;
            std::vector<float> P(n * 3), N(n * 3, 0.0f);
            cgltf_accessor_unpack_floats(pos, P.data(), P.size());
            std::vector<std::uint32_t> idx;
            if (prim.indices)
                for (cgltf_size k = 0; k < prim.indices->count; ++k) idx.push_back(std::uint32_t(cgltf_accessor_read_index(prim.indices, k)));
            else
                for (std::size_t k = 0; k < n; ++k) idx.push_back(std::uint32_t(k));
            if (nrm) {
                cgltf_accessor_unpack_floats(nrm, N.data(), N.size());
            } else { // area-weighted smooth normals
                for (std::size_t t = 0; t + 2 < idx.size(); t += 3) {
                    const float* p0 = &P[idx[t] * 3];
                    const float* p1 = &P[idx[t + 1] * 3];
                    const float* p2 = &P[idx[t + 2] * 3];
                    const float u[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]}, v[3] = {p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]};
                    const float c[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
                    for (int k = 0; k < 3; ++k)
                        for (int j = 0; j < 3; ++j) N[idx[t + std::size_t(k)] * 3 + std::size_t(j)] += c[j];
                }
            }
            const auto base = std::uint32_t(out.vertexCount());
            for (std::size_t i = 0; i < n; ++i) {
                float p[3], q[3];
                transform(M, &P[i * 3], p, 1.0f);
                const float* m = &N[i * 3];
                for (int r = 0; r < 3; ++r) q[r] = nm[r] * m[0] + nm[3 + r] * m[1] + nm[6 + r] * m[2];
                const float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
                for (float& c : q) c = len > 1e-12f ? c / len : 0.0f;
                if (det < 0.0f)
                    for (float& c : q) c = -c;
                out.positions.insert(out.positions.end(), p, p + 3);
                out.normals.insert(out.normals.end(), q, q + 3);
            }
            if (col || !out.colors.empty()) {
                if (out.colors.empty()) out.colors.assign(std::size_t(base) * 3, 1.0f);
                for (std::size_t i = 0; i < n; ++i) {
                    float c[4] = {1, 1, 1, 1};
                    if (col) cgltf_accessor_read_float(col, i, c, cgltf_num_components(col->type));
                    out.colors.insert(out.colors.end(), c, c + 3);
                }
            }
            BodyMesh::Primitive p;
            p.firstIndex = std::uint32_t(out.indices.size());
            p.indexCount = std::uint32_t(idx.size() / 3 * 3);
            p.material = material(prim.material);
            for (std::size_t t = 0; t + 2 < idx.size(); t += 3) {
                out.indices.push_back(base + idx[t]);
                out.indices.push_back(base + (det < 0.0f ? idx[t + 2] : idx[t + 1]));
                out.indices.push_back(base + (det < 0.0f ? idx[t + 1] : idx[t + 2]));
            }
            out.primitives.push_back(p);
        }
    }

    void hull(const std::string& name, const cgltf_mesh& mesh, const float* M) {
        BodyHull h;
        h.name = name;
        for (cgltf_size pi = 0; pi < mesh.primitives_count; ++pi) {
            const cgltf_primitive& prim = mesh.primitives[pi];
            const cgltf_accessor* pos = attribute(prim, cgltf_attribute_type_position);
            if (!pos || !prim.indices || prim.type != cgltf_primitive_type_triangles) {
                errors.push_back(name + ": collision primitive must be indexed triangles");
                continue;
            }
            const auto base = std::uint32_t(h.points.size());
            std::vector<float> P(pos->count * 3);
            cgltf_accessor_unpack_floats(pos, P.data(), P.size());
            for (std::size_t i = 0; i < pos->count; ++i) {
                std::array<float, 3> p;
                transform(M, &P[i * 3], p.data(), 1.0f);
                h.points.push_back(p);
            }
            for (cgltf_size k = 0; k < prim.indices->count; ++k)
                h.indices.push_back(base + std::uint32_t(cgltf_accessor_read_index(prim.indices, k)));
        }
        model.hulls.push_back(std::move(h));
    }
};

} // namespace

bool readBodySource(const std::string& path, BodyModel& out, std::vector<std::string>& errors,
                    std::vector<std::string>& warnings) {
    cgltf_options opts{};
    cgltf_data* raw = nullptr;
    if (cgltf_parse_file(&opts, path.c_str(), &raw) != cgltf_result_success) {
        errors.push_back(path + ": cannot parse");
        return false;
    }
    std::unique_ptr<cgltf_data, DataDeleter> data(raw);
    if (cgltf_load_buffers(&opts, raw, path.c_str()) != cgltf_result_success) {
        errors.push_back(path + ": cannot load buffers");
        return false;
    }
    const std::size_t before = errors.size();
    out = BodyModel{};
    const std::size_t slash = path.find_last_of("/\\");
    out.source = slash == std::string::npos ? path : path.substr(slash + 1);
    Reader r(raw, out, errors, warnings);
    const cgltf_scene* scene = raw->scene ? raw->scene : (raw->scenes_count ? &raw->scenes[0] : nullptr);
    if (scene) {
        for (cgltf_size i = 0; i < scene->nodes_count; ++i) r.visit(*scene->nodes[i], "");
    } else {
        for (cgltf_size i = 0; i < raw->nodes_count; ++i)
            if (!raw->nodes[i].parent) r.visit(raw->nodes[i], "");
    }
    for (std::size_t i = 0; i < out.parts.size(); ++i) {
        BodyPart& p = out.parts[i];
        if (p.lods.empty() || p.lods[0].triangles() == 0) {
            errors.push_back("part '" + p.name + "': LOD node without a base mesh");
            continue;
        }
        if (!r.attachGiven[i]) {
            float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
            const std::vector<float>& P = p.lods[0].positions;
            for (std::size_t k = 0; k < P.size(); ++k) lo[k % 3] = std::min(lo[k % 3], P[k]), hi[k % 3] = std::max(hi[k % 3], P[k]);
            for (int k = 0; k < 3; ++k) p.attach[std::size_t(k)] = 0.5f * (lo[k] + hi[k]);
        }
    }
    out.computeBounds();
    return errors.size() == before;
}

void buildBodyLods(BodyModel& model) {
    const float ratios[kBodyLodCount] = {1.0f, 0.5f, 0.25f};
    const float limits[kBodyLodCount] = {0.0f, 0.010f, 0.030f};
    model.lodError.assign(std::size_t(kBodyLodCount), 0.0f);
    for (BodyPart& part : model.parts) {
        if (part.lods.empty()) continue;
        if (part.lods.size() < std::size_t(kBodyLodCount)) part.lods.resize(std::size_t(kBodyLodCount));
        const BodyMesh& src = part.lods[0];
        for (int l = 1; l < kBodyLodCount; ++l) {
            BodyMesh& dst = part.lods[std::size_t(l)];
            if (!dst.indices.empty()) continue; // authored LOD
            dst.positions = src.positions;
            dst.normals = src.normals;
            dst.colors = src.colors;
            for (const BodyMesh::Primitive& p : src.primitives) {
                std::vector<std::uint32_t> out(p.indexCount);
                const std::size_t target = std::max<std::size_t>(3, std::size_t(float(p.indexCount) * ratios[l]) / 3 * 3);
                float err = 0.0f;
                const std::size_t n = meshopt_simplify(out.data(), src.indices.data() + p.firstIndex, p.indexCount,
                                                       src.positions.data(), src.vertexCount(), 3 * sizeof(float), target,
                                                       limits[l], meshopt_SimplifyLockBorder | meshopt_SimplifySparse |
                                                                      meshopt_SimplifyErrorAbsolute,
                                                       &err);
                if (n == 0) continue;
                BodyMesh::Primitive q;
                q.firstIndex = std::uint32_t(dst.indices.size());
                q.indexCount = std::uint32_t(n);
                q.material = p.material;
                dst.indices.insert(dst.indices.end(), out.begin(), out.begin() + std::ptrdiff_t(n));
                dst.primitives.push_back(q);
                model.lodError[std::size_t(l)] = std::max(model.lodError[std::size_t(l)], err);
            }
        }
    }
}

} // namespace tf
