#include "vehicle/VehicleBody.h"

#include "vehicle/VehicleConfig.h"
#include "vehicle/VehicleDefinition.h"
#include "vehicle/VehicleSetup.h"
#include "vehicle/WheelSweep.h"

#include <Jolt/Jolt.h>

#include <Jolt/Geometry/ConvexHullBuilder.h>
#include <cgltf.h>
#include <quench/core/Json.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <set>

namespace tf {
namespace {

struct RoleInfo {
    BodyRole role;
    const char* name;
    float mud;
};
constexpr RoleInfo kRoles[] = {
    {BodyRole::PaintPrimary, "paint_primary", 1.0f},  {BodyRole::PaintSecondary, "paint_secondary", 1.0f},
    {BodyRole::PaintAccent, "paint_accent", 1.0f},    {BodyRole::Cladding, "cladding", 1.0f},
    {BodyRole::Plastic, "plastic", 1.0f},             {BodyRole::Chrome, "chrome", 0.6f},
    {BodyRole::Metal, "metal", 0.8f},                 {BodyRole::Glass, "glass", 0.15f},
    {BodyRole::Rubber, "rubber", 1.0f},               {BodyRole::Lens, "lens", 0.2f},
    {BodyRole::LightHead, "light_head", 0.1f},        {BodyRole::LightTail, "light_tail", 0.1f},
    {BodyRole::LightBrake, "light_brake", 0.1f},      {BodyRole::LightReverse, "light_reverse", 0.1f},
    {BodyRole::LightIndicator, "light_indicator", 0.1f}, {BodyRole::Liner, "liner", 1.0f},
    {BodyRole::Underbody, "underbody", 1.0f},
};
static_assert(sizeof(kRoles) / sizeof(kRoles[0]) == std::size_t(BodyRole::Count));

struct DataDeleter {
    void operator()(cgltf_data* d) const { cgltf_free(d); }
};

const cgltf_accessor* attribute(const cgltf_primitive& p, cgltf_attribute_type type) {
    for (cgltf_size a = 0; a < p.attributes_count; ++a)
        if (p.attributes[a].type == type && p.attributes[a].index == 0) return p.attributes[a].data;
    return nullptr;
}

json::Value extrasOf(const cgltf_extras& e) {
    if (!e.data) return {};
    quench::Result<json::Value> parsed = json::parse(e.data);
    return parsed ? std::move(*parsed) : json::Value{};
}

std::array<float, 3> vec3Of(const json::Value& v, std::array<float, 3> fallback) {
    if (!v.isArray() || v.array.size() != 3) return fallback;
    for (std::size_t i = 0; i < 3; ++i) fallback[i] = float(v.array[i].numberOr(fallback[i]));
    return fallback;
}

json::Value vec(std::initializer_list<float> v) {
    json::Value a = json::Value::makeArray();
    for (float f : v) a.array.push_back(json::Value::makeNumber(double(f)));
    return a;
}
json::Value vec3Value(const std::array<float, 3>& v) { return vec({v[0], v[1], v[2]}); }

// Appends the primitives of a converted mesh to `out` (absolute indices).
bool readMesh(const cgltf_data* data, const cgltf_mesh& mesh, BodyMesh& out, std::string& err) {
    for (cgltf_size pi = 0; pi < mesh.primitives_count; ++pi) {
        const cgltf_primitive& prim = mesh.primitives[pi];
        const cgltf_accessor* pos = attribute(prim, cgltf_attribute_type_position);
        const cgltf_accessor* nrm = attribute(prim, cgltf_attribute_type_normal);
        const cgltf_accessor* col = attribute(prim, cgltf_attribute_type_color);
        if (!pos || !nrm || !prim.indices || !prim.material) {
            err = std::string(mesh.name ? mesh.name : "?") + ": primitive without positions, normals, indices or material";
            return false;
        }
        const auto base = std::uint32_t(out.vertexCount());
        const std::size_t n = pos->count;
        std::vector<float> P(n * 3), N(n * 3);
        cgltf_accessor_unpack_floats(pos, P.data(), P.size());
        cgltf_accessor_unpack_floats(nrm, N.data(), N.size());
        if (col || !out.colors.empty()) {
            if (out.colors.empty()) out.colors.assign(out.positions.size(), 1.0f);
            std::vector<float> C(n * 3, 1.0f);
            if (col) {
                const cgltf_size k = cgltf_num_components(col->type);
                for (std::size_t i = 0; i < n; ++i) {
                    float c[4] = {1, 1, 1, 1};
                    cgltf_accessor_read_float(col, i, c, k);
                    for (int j = 0; j < 3; ++j) C[i * 3 + std::size_t(j)] = c[j];
                }
            }
            out.colors.insert(out.colors.end(), C.begin(), C.end());
        }
        out.positions.insert(out.positions.end(), P.begin(), P.end());
        out.normals.insert(out.normals.end(), N.begin(), N.end());
        BodyMesh::Primitive p;
        p.firstIndex = std::uint32_t(out.indices.size());
        p.indexCount = std::uint32_t(prim.indices->count);
        p.material = int(prim.material - data->materials);
        for (cgltf_size k = 0; k < prim.indices->count; ++k)
            out.indices.push_back(base + std::uint32_t(cgltf_accessor_read_index(prim.indices, k)));
        out.primitives.push_back(p);
    }
    return true;
}

// ---- GLB writer
class GlbBuilder {
public:
    json::Value root = json::Value::makeObject();
    std::vector<unsigned char> bin;

    GlbBuilder() {
        for (const char* k : {"accessors", "bufferViews", "meshes", "nodes", "materials"})
            root.object[k] = json::Value::makeArray();
    }
    int view(const void* data, std::size_t bytes, int target) {
        while (bin.size() % 4) bin.push_back(0);
        json::Value v = json::Value::makeObject();
        v.object["buffer"] = json::Value::makeNumber(0);
        v.object["byteOffset"] = json::Value::makeNumber(double(bin.size()));
        v.object["byteLength"] = json::Value::makeNumber(double(bytes));
        if (target) v.object["target"] = json::Value::makeNumber(target);
        const auto* p = static_cast<const unsigned char*>(data);
        bin.insert(bin.end(), p, p + bytes);
        return push("bufferViews", v);
    }
    int floats(const std::vector<float>& f, int comps, bool minMax) {
        json::Value a = json::Value::makeObject();
        a.object["bufferView"] = json::Value::makeNumber(view(f.data(), f.size() * sizeof(float), 34962));
        a.object["componentType"] = json::Value::makeNumber(5126);
        a.object["count"] = json::Value::makeNumber(double(f.size() / std::size_t(comps)));
        a.object["type"] = json::Value::makeString(comps == 3 ? "VEC3" : "VEC2");
        if (minMax) {
            json::Value lo = json::Value::makeArray(), hi = json::Value::makeArray();
            for (int c = 0; c < comps; ++c) {
                float l = 1e30f, h = -1e30f;
                for (std::size_t i = std::size_t(c); i < f.size(); i += std::size_t(comps)) l = std::min(l, f[i]), h = std::max(h, f[i]);
                lo.array.push_back(json::Value::makeNumber(l));
                hi.array.push_back(json::Value::makeNumber(h));
            }
            a.object["min"] = lo;
            a.object["max"] = hi;
        }
        return push("accessors", a);
    }
    int indices(const std::uint32_t* idx, std::size_t count) {
        json::Value a = json::Value::makeObject();
        a.object["bufferView"] = json::Value::makeNumber(view(idx, count * sizeof(std::uint32_t), 34963));
        a.object["componentType"] = json::Value::makeNumber(5125);
        a.object["count"] = json::Value::makeNumber(double(count));
        a.object["type"] = json::Value::makeString("SCALAR");
        return push("accessors", a);
    }
    int push(const char* array, json::Value v) {
        auto& a = root.object[array].array;
        a.push_back(std::move(v));
        return int(a.size()) - 1;
    }
    // One glTF primitive per body primitive; each gets its own compacted vertex range.
    int mesh(const std::string& name, const BodyMesh& m) {
        json::Value mesh = json::Value::makeObject();
        mesh.object["name"] = json::Value::makeString(name);
        json::Value prims = json::Value::makeArray();
        for (const BodyMesh::Primitive& p : m.primitives) {
            std::vector<std::uint32_t> remap(m.vertexCount(), UINT32_MAX), idx;
            std::vector<float> P, N, C;
            for (std::uint32_t k = 0; k < p.indexCount; ++k) {
                const std::uint32_t v = m.indices[p.firstIndex + k];
                if (remap[v] == UINT32_MAX) {
                    remap[v] = std::uint32_t(P.size() / 3);
                    for (int j = 0; j < 3; ++j) {
                        P.push_back(m.positions[v * 3 + std::uint32_t(j)]);
                        N.push_back(m.normals[v * 3 + std::uint32_t(j)]);
                        if (!m.colors.empty()) C.push_back(m.colors[v * 3 + std::uint32_t(j)]);
                    }
                }
                idx.push_back(remap[v]);
            }
            json::Value attrs = json::Value::makeObject();
            attrs.object["POSITION"] = json::Value::makeNumber(floats(P, 3, true));
            attrs.object["NORMAL"] = json::Value::makeNumber(floats(N, 3, false));
            if (!C.empty()) attrs.object["COLOR_0"] = json::Value::makeNumber(floats(C, 3, false));
            json::Value prim = json::Value::makeObject();
            prim.object["attributes"] = attrs;
            prim.object["indices"] = json::Value::makeNumber(indices(idx.data(), idx.size()));
            prim.object["material"] = json::Value::makeNumber(p.material);
            prims.array.push_back(prim);
        }
        mesh.object["primitives"] = prims;
        return push("meshes", mesh);
    }
    bool write(const std::string& path, std::string* error) {
        root.object["buffers"] = json::Value::makeArray();
        json::Value buf = json::Value::makeObject();
        while (bin.size() % 4) bin.push_back(0);
        buf.object["byteLength"] = json::Value::makeNumber(double(bin.size()));
        root.object["buffers"].array.push_back(buf);
        std::string js = json::write(root);
        while (js.size() % 4) js.push_back(' ');
        std::ofstream f(path, std::ios::binary);
        if (!f) {
            if (error) *error = path + ": cannot write";
            return false;
        }
        auto u32 = [&](std::uint32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); };
        u32(0x46546C67);
        u32(2);
        u32(std::uint32_t(12 + 8 + js.size() + 8 + bin.size()));
        u32(std::uint32_t(js.size()));
        u32(0x4E4F534A);
        f.write(js.data(), std::streamsize(js.size()));
        u32(std::uint32_t(bin.size()));
        u32(0x004E4942);
        f.write(reinterpret_cast<const char*>(bin.data()), std::streamsize(bin.size()));
        return bool(f);
    }
};

// Convex hull of a piece's points (Jolt's builder, robust against the slivers of authored hulls): face planes with
// unit normals. Empty when the points span no volume.
struct HullPlanes {
    std::vector<std::array<float, 4>> planes;
    explicit HullPlanes(const BodyHull& h) {
        if (h.points.size() < 4) return;
        if (JPH::Allocate == nullptr) JPH::RegisterDefaultAllocator();
        JPH::ConvexHullBuilder::Positions pos;
        for (const auto& p : h.points) pos.push_back(JPH::Vec3(p[0], p[1], p[2]));
        JPH::ConvexHullBuilder builder(pos);
        const char* err = nullptr;
        const auto r = builder.Initialize(INT_MAX, 1e-4f, err);
        if (r != JPH::ConvexHullBuilder::EResult::Success && r != JPH::ConvexHullBuilder::EResult::MaxVerticesReached) return;
        for (const JPH::ConvexHullBuilder::Face* f : builder.GetFaces()) {
            if (f->mRemoved || f->mNormal.LengthSq() < 1e-20f) continue;
            const JPH::Vec3 n = f->mNormal.Normalized();
            planes.push_back({n.GetX(), n.GetY(), n.GetZ(), -n.Dot(f->mCentroid)});
        }
    }
    // > 0 outside (lower bound of the distance), < 0 inside (depth below the nearest face).
    float distance(const float* p) const {
        float d = -1e9f;
        for (const auto& q : planes) d = std::max(d, q[0] * p[0] + q[1] * p[1] + q[2] * p[2] + q[3]);
        return d;
    }
};

std::string point(const float* p) {
    char buf[96];
    std::snprintf(buf, sizeof buf, "(%.3f, %.3f, %.3f)", p[0], p[1], p[2]);
    return buf;
}

} // namespace

const char* bodyRoleName(BodyRole r) {
    for (const RoleInfo& i : kRoles)
        if (i.role == r) return i.name;
    return "?";
}

bool bodyRoleFromName(const std::string& name, BodyRole& out) {
    for (const RoleInfo& i : kRoles)
        if (name == i.name) {
            out = i.role;
            return true;
        }
    return false;
}

std::string bodyRoleList() {
    std::string s;
    for (const RoleInfo& i : kRoles) s += (s.empty() ? "" : ", ") + std::string(i.name);
    return s;
}

float bodyRoleDefaultMud(BodyRole r) {
    for (const RoleInfo& i : kRoles)
        if (i.role == r) return i.mud;
    return 1.0f;
}

bool bodyNameIsReserved(const std::string& name) {
    static const std::set<std::string> kReserved = {"wheel", "wheels", "tire",  "tires",  "tyre",       "tyres",
                                                    "rim",   "rims",   "spring", "springs", "shock",     "shocks",
                                                    "strut", "struts", "damper", "coil",  "suspension", "axle"};
    std::string token;
    for (std::size_t i = 0; i <= name.size(); ++i) {
        const char c = i < name.size() ? char(std::tolower(static_cast<unsigned char>(name[i]))) : '_';
        if (c == '_' || c == '.' || c == '-' || c == ' ') {
            if (kReserved.count(token)) return true;
            token.clear();
        } else {
            token += c;
        }
    }
    return false;
}

const BodyPart* BodyModel::find(const std::string& name) const {
    for (const BodyPart& p : parts)
        if (p.name == name) return &p;
    return nullptr;
}

void BodyModel::computeBounds() {
    boundsMin = {1e9f, 1e9f, 1e9f};
    boundsMax = {-1e9f, -1e9f, -1e9f};
    for (const BodyPart& p : parts) {
        if (p.lods.empty()) continue;
        const std::vector<float>& P = p.lods[0].positions;
        for (std::size_t i = 0; i < P.size(); ++i) {
            boundsMin[i % 3] = std::min(boundsMin[i % 3], P[i]);
            boundsMax[i % 3] = std::max(boundsMax[i % 3], P[i]);
        }
    }
}

bool loadBodyModel(const std::string& path, BodyModel& out, bool withMeshes, std::string* error) {
    auto fail = [&](const std::string& msg) {
        if (error) *error = path + ": " + msg;
        return false;
    };
    cgltf_options opts{};
    cgltf_data* raw = nullptr;
    if (cgltf_parse_file(&opts, path.c_str(), &raw) != cgltf_result_success) return fail("cannot parse");
    std::unique_ptr<cgltf_data, DataDeleter> data(raw);
    if (cgltf_load_buffers(&opts, raw, path.c_str()) != cgltf_result_success) return fail("cannot load buffers");
    const json::Value extras = extrasOf(raw->extras);
    if (extras["kind"].stringOr("") != "vehicle_body")
        return fail("not a converted vehicle body (run trailforge_vehiclec on the Blender export)");
    if (int(extras["version"].numberOr(0)) != 1) return fail("unsupported vehicle body version");
    out = BodyModel{};
    out.source = extras["source"].stringOr("");
    out.boundsMin = vec3Of(extras["boundsMin"], {});
    out.boundsMax = vec3Of(extras["boundsMax"], {});
    for (const json::Value& e : extras["lodError"].array) out.lodError.push_back(float(e.numberOr(0)));

    for (cgltf_size m = 0; m < raw->materials_count; ++m) {
        const cgltf_material& src = raw->materials[m];
        const json::Value mx = extrasOf(src.extras);
        BodyMaterial mat;
        mat.name = src.name ? src.name : "";
        if (!bodyRoleFromName(mx["role"].stringOr(""), mat.role)) return fail("material " + mat.name + ": unknown role");
        const cgltf_pbr_metallic_roughness& pbr = src.pbr_metallic_roughness;
        mat.color = {pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2]};
        mat.alpha = pbr.base_color_factor[3];
        mat.roughness = pbr.roughness_factor;
        mat.metallic = pbr.metallic_factor;
        mat.emissive = {src.emissive_factor[0], src.emissive_factor[1], src.emissive_factor[2]};
        if (src.has_emissive_strength) mat.emissiveStrength = src.emissive_strength.emissive_strength;
        if (src.has_clearcoat) {
            mat.clearcoat = src.clearcoat.clearcoat_factor;
            mat.clearcoatRoughness = src.clearcoat.clearcoat_roughness_factor;
        }
        if (src.has_transmission) mat.transmission = src.transmission.transmission_factor;
        if (src.has_ior) mat.ior = src.ior.ior;
        mat.mud = float(mx["mud"].numberOr(bodyRoleDefaultMud(mat.role)));
        out.materials.push_back(mat);
    }

    std::string err;
    for (cgltf_size ni = 0; ni < raw->nodes_count; ++ni) {
        const cgltf_node& node = raw->nodes[ni];
        const std::string name = node.name ? node.name : "";
        const json::Value nx = extrasOf(node.extras);
        if (name.rfind("COL_", 0) == 0) {
            if (!node.mesh) return fail(name + ": collision piece without a mesh");
            BodyMesh m;
            for (cgltf_size pi = 0; pi < node.mesh->primitives_count; ++pi) {
                const cgltf_primitive& prim = node.mesh->primitives[pi];
                const cgltf_accessor* pos = attribute(prim, cgltf_attribute_type_position);
                if (!pos || !prim.indices) return fail(name + ": collision primitive without positions or indices");
                const auto base = std::uint32_t(m.positions.size() / 3);
                std::vector<float> P(pos->count * 3);
                cgltf_accessor_unpack_floats(pos, P.data(), P.size());
                m.positions.insert(m.positions.end(), P.begin(), P.end());
                for (cgltf_size k = 0; k < prim.indices->count; ++k)
                    m.indices.push_back(base + std::uint32_t(cgltf_accessor_read_index(prim.indices, k)));
            }
            BodyHull h;
            h.name = name;
            for (std::size_t i = 0; i + 2 < m.positions.size(); i += 3)
                h.points.push_back({m.positions[i], m.positions[i + 1], m.positions[i + 2]});
            h.indices = std::move(m.indices);
            out.hulls.push_back(std::move(h));
            continue;
        }
        if (name.rfind("SPARE_", 0) == 0) {
            BodySpare s;
            s.name = name.substr(6);
            s.parent = nx["parent"].stringOr("");
            s.seat = vec3Of(nx["seat"], {});
            s.outward = vec3Of(nx["outward"], s.outward);
            out.spares.push_back(s);
            continue;
        }
        BodyPart part;
        part.name = name;
        part.parent = nx["parent"].stringOr("");
        part.attach = vec3Of(nx["attach"], {});
        part.deformable = nx["deformable"].boolOr(false);
        part.detachable = nx["detachable"].boolOr(false);
        part.collides = nx["collides"].boolOr(true);
        if (withMeshes) {
            for (const json::Value& li : nx["lods"].array) {
                const auto mi = std::size_t(li.numberOr(-1));
                if (mi >= raw->meshes_count) return fail(name + ": bad LOD mesh index");
                BodyMesh m;
                if (!readMesh(raw, raw->meshes[mi], m, err)) return fail(err);
                part.lods.push_back(std::move(m));
            }
            if (part.lods.empty()) return fail(name + ": part without meshes");
        }
        out.parts.push_back(std::move(part));
    }
    for (const BodyHull& h : out.hulls)
        if (const float e = hullConvexityError(h); e > kBodyConvexTolerance) {
            char buf[64];
            std::snprintf(buf, sizeof buf, " is not convex (%.1f mm)", 1000.0f * std::min(e, 9.9f));
            return fail(h.name + buf);
        }
    if (out.parts.empty()) return fail("no parts");
    return true;
}

bool writeBodyModel(const std::string& path, const BodyModel& model, std::string* error) {
    GlbBuilder g;
    json::Value asset = json::Value::makeObject();
    asset.object["version"] = json::Value::makeString("2.0");
    asset.object["generator"] = json::Value::makeString("trailforge_vehiclec");
    g.root.object["asset"] = asset;
    json::Value ex = json::Value::makeObject();
    ex.object["kind"] = json::Value::makeString("vehicle_body");
    ex.object["version"] = json::Value::makeNumber(1);
    ex.object["source"] = json::Value::makeString(model.source);
    ex.object["boundsMin"] = vec3Value(model.boundsMin);
    ex.object["boundsMax"] = vec3Value(model.boundsMax);
    json::Value le = json::Value::makeArray();
    for (float e : model.lodError) le.array.push_back(json::Value::makeNumber(e));
    ex.object["lodError"] = le;
    g.root.object["extras"] = ex;

    std::set<std::string> used;
    for (const BodyMaterial& m : model.materials) {
        json::Value mat = json::Value::makeObject();
        mat.object["name"] = json::Value::makeString(m.name);
        json::Value pbr = json::Value::makeObject();
        pbr.object["baseColorFactor"] = vec({m.color[0], m.color[1], m.color[2], m.alpha});
        pbr.object["roughnessFactor"] = json::Value::makeNumber(m.roughness);
        pbr.object["metallicFactor"] = json::Value::makeNumber(m.metallic);
        mat.object["pbrMetallicRoughness"] = pbr;
        mat.object["emissiveFactor"] = vec3Value(m.emissive);
        json::Value exts = json::Value::makeObject();
        auto ext = [&](const char* name, const char* key, float v, const char* key2 = nullptr, float v2 = 0.0f) {
            json::Value e = json::Value::makeObject();
            e.object[key] = json::Value::makeNumber(v);
            if (key2) e.object[key2] = json::Value::makeNumber(v2);
            exts.object[name] = e;
            used.insert(name);
        };
        if (m.clearcoat > 0.0f)
            ext("KHR_materials_clearcoat", "clearcoatFactor", m.clearcoat, "clearcoatRoughnessFactor", m.clearcoatRoughness);
        if (m.transmission > 0.0f) ext("KHR_materials_transmission", "transmissionFactor", m.transmission);
        if (m.ior != 1.5f) ext("KHR_materials_ior", "ior", m.ior);
        if (m.emissiveStrength != 1.0f) ext("KHR_materials_emissive_strength", "emissiveStrength", m.emissiveStrength);
        if (!exts.object.empty()) mat.object["extensions"] = exts;
        if (m.alpha < 1.0f) mat.object["alphaMode"] = json::Value::makeString("BLEND");
        json::Value mx = json::Value::makeObject();
        mx.object["role"] = json::Value::makeString(bodyRoleName(m.role));
        mx.object["mud"] = json::Value::makeNumber(m.mud);
        mat.object["extras"] = mx;
        g.push("materials", mat);
    }
    if (!used.empty()) {
        json::Value u = json::Value::makeArray();
        for (const std::string& s : used) u.array.push_back(json::Value::makeString(s));
        g.root.object["extensionsUsed"] = u;
    }

    json::Value sceneNodes = json::Value::makeArray();
    auto addNode = [&](json::Value node) { sceneNodes.array.push_back(json::Value::makeNumber(g.push("nodes", std::move(node)))); };
    for (const BodyPart& p : model.parts) {
        json::Value lods = json::Value::makeArray();
        for (std::size_t l = 0; l < p.lods.size(); ++l)
            lods.array.push_back(json::Value::makeNumber(g.mesh(p.name + ".lod" + std::to_string(l), p.lods[l])));
        json::Value node = json::Value::makeObject();
        node.object["name"] = json::Value::makeString(p.name);
        if (!lods.array.empty()) node.object["mesh"] = lods.array[0];
        json::Value nx = json::Value::makeObject();
        nx.object["parent"] = json::Value::makeString(p.parent);
        nx.object["attach"] = vec3Value(p.attach);
        nx.object["deformable"] = json::Value::makeBool(p.deformable);
        nx.object["detachable"] = json::Value::makeBool(p.detachable);
        nx.object["collides"] = json::Value::makeBool(p.collides);
        nx.object["lods"] = lods;
        node.object["extras"] = nx;
        addNode(node);
    }
    for (const BodyHull& h : model.hulls) {
        BodyMesh m;
        for (const auto& p : h.points) m.positions.insert(m.positions.end(), p.begin(), p.end());
        json::Value mesh = json::Value::makeObject();
        mesh.object["name"] = json::Value::makeString(h.name);
        json::Value attrs = json::Value::makeObject();
        attrs.object["POSITION"] = json::Value::makeNumber(g.floats(m.positions, 3, true));
        json::Value prim = json::Value::makeObject();
        prim.object["attributes"] = attrs;
        prim.object["indices"] = json::Value::makeNumber(g.indices(h.indices.data(), h.indices.size()));
        json::Value prims = json::Value::makeArray();
        prims.array.push_back(prim);
        mesh.object["primitives"] = prims;
        json::Value node = json::Value::makeObject();
        node.object["name"] = json::Value::makeString(h.name);
        node.object["mesh"] = json::Value::makeNumber(g.push("meshes", mesh));
        addNode(node);
    }
    for (const BodySpare& s : model.spares) {
        json::Value node = json::Value::makeObject();
        node.object["name"] = json::Value::makeString("SPARE_" + s.name);
        json::Value nx = json::Value::makeObject();
        nx.object["parent"] = json::Value::makeString(s.parent);
        nx.object["seat"] = vec3Value(s.seat);
        nx.object["outward"] = vec3Value(s.outward);
        node.object["extras"] = nx;
        node.object["translation"] = vec3Value(s.seat);
        addNode(node);
    }
    json::Value scene = json::Value::makeObject();
    scene.object["nodes"] = sceneNodes;
    g.root.object["scenes"] = json::Value::makeArray();
    g.root.object["scenes"].array.push_back(scene);
    g.root.object["scene"] = json::Value::makeNumber(0);
    return g.write(path, error);
}

float hullConvexityError(const BodyHull& hull) {
    // A flat or tiny piece has no volume to collide with.
    float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    for (const auto& p : hull.points)
        for (int k = 0; k < 3; ++k) lo[k] = std::min(lo[k], p[std::size_t(k)]), hi[k] = std::max(hi[k], p[std::size_t(k)]);
    for (int k = 0; k < 3; ++k)
        if (hull.points.size() < 4 || hi[k] - lo[k] < 0.01f) return 1e9f;
    const HullPlanes planes(hull);
    if (planes.planes.size() < 4) return 1e9f;
    // Convex: every point lies on the hull of all points. A dent shows as points below the hull surface.
    float worst = 0.0f;
    for (const auto& p : hull.points) worst = std::max(worst, -planes.distance(p.data()));
    return worst;
}

BodyValidation validateBodyModel(const BodyModel& model, const std::vector<VehicleConfig>& configs, float clearance) {
    BodyValidation r;
    std::set<std::string> names;
    for (const BodyPart& p : model.parts) {
        if (!names.insert(p.name).second) r.errors.push_back("duplicate part '" + p.name + "'");
        if (bodyNameIsReserved(p.name))
            r.errors.push_back("part '" + p.name + "': wheels, tires, rims and suspension are drawn by the game, not the body");
        if (p.lods.empty() || p.lods[0].triangles() == 0) r.errors.push_back("part '" + p.name + "' has no triangles");
    }
    for (const BodyPart& p : model.parts)
        if (!p.parent.empty() && !names.count(p.parent))
            r.errors.push_back("part '" + p.name + "': unknown parent '" + p.parent + "'");
    for (const BodySpare& s : model.spares)
        if (!s.parent.empty() && !names.count(s.parent))
            r.errors.push_back("spare '" + s.name + "': unknown parent '" + s.parent + "'");
    for (const BodyMaterial& m : model.materials) {
        const bool light = m.role >= BodyRole::LightHead && m.role <= BodyRole::LightIndicator;
        if (light && std::max({m.emissive[0], m.emissive[1], m.emissive[2], m.color[0], m.color[1], m.color[2]}) < 0.05f)
            r.warnings.push_back("light material '" + m.name + "' is black (the lamp colour is its emission or base colour)");
    }
    if (model.hulls.empty()) r.errors.push_back("no collision pieces (COL_* nodes)");
    for (const BodyHull& h : model.hulls) {
        const float e = hullConvexityError(h);
        if (e > kBodyConvexTolerance) {
            char buf[128];
            if (e > 1e8f) std::snprintf(buf, sizeof buf, "%s is degenerate (needs a closed convex volume)", h.name.c_str());
            else std::snprintf(buf, sizeof buf, "%s is not convex: a point lies %.1f mm inside its hull", h.name.c_str(), 1000.0f * e);
            r.errors.push_back(buf);
        }
    }

    // Wheel sweeps: every triangle's corners, edge midpoints and centroid, so large triangles cannot span a sweep.
    for (const VehicleConfig& c : configs) {
        const auto sweeps = wheelSweeps(c);
        float reachMax = 0.0f;
        for (const WheelSweep& s : sweeps) reachMax = std::max(reachMax, std::hypot(s.radius, s.halfWidth) + 0.05f);
        auto samples = [&](const std::vector<float>& P, const std::vector<std::uint32_t>& I, auto&& visit) {
            for (std::size_t t = 0; t + 2 < I.size(); t += 3) {
                const float* a = &P[I[t] * 3];
                const float* b = &P[I[t + 1] * 3];
                const float* d = &P[I[t + 2] * 3];
                float s[7][3];
                for (int k = 0; k < 3; ++k) {
                    s[0][k] = a[k], s[1][k] = b[k], s[2][k] = d[k];
                    s[3][k] = 0.5f * (a[k] + b[k]), s[4][k] = 0.5f * (b[k] + d[k]), s[5][k] = 0.5f * (d[k] + a[k]);
                    s[6][k] = (a[k] + b[k] + d[k]) / 3.0f;
                }
                for (const auto& p : s) {
                    bool near = false;
                    for (const WheelSweep& w : sweeps)
                        near = near || (std::abs(p[0] - w.x) < reachMax + 0.3f && std::abs(p[2] - w.z) < reachMax + 0.3f);
                    if (near) visit(p);
                }
            }
        };
        for (std::size_t l = 0; l < std::size_t(kBodyLodCount); ++l)
            for (const BodyPart& p : model.parts) {
                if (l >= p.lods.size()) continue;
                const BodyMesh& m = p.lods[l];
                int hits = 0;
                std::string first;
                samples(m.positions, m.indices, [&](const float* q) {
                    const float cl = wheelSweepClearance(sweeps, q[0], q[1], q[2]);
                    if (l == 0 && cl < r.minClearance) {
                        r.minClearance = cl;
                        r.clearanceWhere = p.name + " " + point(q);
                    }
                    if (cl < clearance && hits++ == 0) first = point(q);
                });
                if (hits > 0) {
                    char buf[256];
                    std::snprintf(buf, sizeof buf, "part '%s' LOD%zu: %d points within %.0f mm of the wheel sweep (first %s) for config '%s'",
                                  p.name.c_str(), l, hits, 1000.0f * clearance, first.c_str(), c.id.c_str());
                    (l == 0 ? r.errors : r.warnings).push_back(buf);
                }
            }
        for (const BodyHull& h : model.hulls) {
            std::vector<float> P;
            for (const auto& q : h.points) P.insert(P.end(), q.begin(), q.end());
            int hits = 0;
            samples(P, h.indices, [&](const float* q) {
                const float cl = wheelSweepClearance(sweeps, q[0], q[1], q[2]);
                r.hullClearance = std::min(r.hullClearance, cl);
                if (cl < 0.0f) ++hits;
            });
            if (hits > 0)
                r.errors.push_back(h.name + ": " + std::to_string(hits) + " points inside the wheel sweep for config '" + c.id + "'");
        }
    }

    // Visual vs collision: bounds per face (parts flagged collides = false, e.g. mirrors, are left out) and the share
    // of render vertices close to a piece.
    if (!model.hulls.empty()) {
        float vlo[3] = {1e9f, 1e9f, 1e9f}, vhi[3] = {-1e9f, -1e9f, -1e9f}, clo[3] = {1e9f, 1e9f, 1e9f}, chi[3] = {-1e9f, -1e9f, -1e9f};
        std::vector<HullPlanes> planes;
        for (const BodyHull& h : model.hulls) {
            planes.emplace_back(h);
            for (const auto& p : h.points)
                for (int k = 0; k < 3; ++k) clo[k] = std::min(clo[k], p[std::size_t(k)]), chi[k] = std::max(chi[k], p[std::size_t(k)]);
        }
        std::size_t total = 0, close = 0;
        for (const BodyPart& p : model.parts) {
            if (!p.collides || p.lods.empty()) continue;
            const std::vector<float>& P = p.lods[0].positions;
            for (std::size_t i = 0; i + 2 < P.size(); i += 3) {
                for (int k = 0; k < 3; ++k) vlo[k] = std::min(vlo[k], P[i + std::size_t(k)]), vhi[k] = std::max(vhi[k], P[i + std::size_t(k)]);
                float d = 1e9f;
                for (const HullPlanes& h : planes) d = std::min(d, h.distance(&P[i]));
                ++total;
                if (d <= 0.10f) ++close;
            }
        }
        if (total > 0) {
            for (int k = 0; k < 3; ++k)
                r.boundsDeviation = std::max({r.boundsDeviation, std::abs(vlo[k] - clo[k]), std::abs(vhi[k] - chi[k])});
            r.hullCoverage = float(close) / float(total);
            char buf[160];
            if (r.boundsDeviation > kBodyBoundsTolerance) {
                std::snprintf(buf, sizeof buf, "visual and collision bounds differ by %.0f mm (limit %.0f mm)",
                              1000.0f * r.boundsDeviation, 1000.0f * kBodyBoundsTolerance);
                r.errors.push_back(buf);
            }
            if (r.hullCoverage < 0.9f) {
                std::snprintf(buf, sizeof buf, "only %.0f%% of the body vertices are within 100 mm of a collision piece (need 90%%)",
                              100.0f * r.hullCoverage);
                r.errors.push_back(buf);
            }
        }
    }
    return r;
}

std::vector<VehicleConfig> bodyCheckConfigs(const VehicleDefinition& def) {
    std::vector<VehicleConfig> bases{def.config};
    for (const VehicleKit& k : def.kits) {
        VehicleConfig c;
        std::vector<std::string> errors;
        if (configWithKit(def, k.id, c, errors)) bases.push_back(c);
    }
    std::vector<VehicleConfig> out;
    const SetupParamInfo* ride = findSetupParam("suspension.rideHeight");
    for (std::size_t i = 0; i < bases.size(); ++i) {
        const VehicleConfig& base = bases[i];
        out.push_back(base);
        if (!ride) continue;
        const VehicleKit* kit = i == 0 ? nullptr : &def.kits[i - 1];
        const ParamRange range = setupParamRange(*ride, def, kit, base);
        for (float v : {range.min, range.max}) {
            VehicleConfig c = base;
            ride->set(c, base, v);
            if (!(c == base)) out.push_back(c);
        }
    }
    return out;
}

} // namespace tf
