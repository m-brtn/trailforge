// trailforge_modelc: converts one CC0 glTF model (Poly Haven) into a game-ready glb with mesh LODs, an optional
// collision mesh and an optional impostor atlas, plus the material textures as BC7 KTX2 (same encodings as the
// terrain: <mat>_albedo.ktx2 = sRGB albedo + alpha, <mat>_nra.ktx2 = normal x / y (DirectX), roughness, AO).
//
//   trailforge_modelc --spec spec.json
//
// spec.json (written by tools/models/fetch_models.py):
//   { "id": "a", "gltf": ".../fir_tree_01.gltf", "out": "assets/models/fir_tree_01", "kind": "tree",
//     "nodes": ["fir_tree_01_a_LOD0"], "lods": [24000, 6000, 1500], "collisionLod": -1, "impostor": [128, 256],
//     "origin": "base" | "center", "writeTextures": true, "textureSize": 1024, "normalSize": 512,
//     "materials": { "fir_tree_01_twig": { "name": "fir_tree_01_twig", "albedo": "...diff.jpg", "alpha": "...alpha.png",
//                                          "normal": "...nor_gl.jpg", "normalGl": true, "arm": "...arm.jpg",
//                                          "write": true } } }
//
// LODs: opaque parts (bark, rock) are simplified with meshoptimizer (positions + normals + UVs). Foliage parts
// (alpha-tested cards and strips) are first reduced per card (each connected strip to about two triangles), then
// thinned by dropping whole cards; the kept cards grow about their end nearest the trunk so the crown keeps its
// coverage. Trees with far more foliage than the budget (Poly Haven conifers: millions of needle triangles) get clump
// cards instead: one quad per spatial cluster of source foliage, textured from a per-model atlas of baked clusters
// (<id>_foliage_albedo / _nra.ktx2). Foliage normals are bent toward the crown ellipsoid (soft, volumetric lighting).
// Per-vertex wind weights (_WIND: x trunk bend, y foliage flutter) are written for the render-only wind sway.
// Impostor: LOD0 rasterized on the CPU (orthographic, alpha-tested, 2 x 2 supersampled) from N azimuths into one
// atlas row; albedo + coverage and the view-space normal + AO. Output is deterministic for the same inputs.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#include <stb_image.h>
#undef STB_IMAGE_IMPLEMENTATION

#define CGLTF_IMPLEMENTATION
#include <cgltf.h>
#include <meshoptimizer.h>

static const char* const kToolName = "trailforge_modelc";
#include "TexCore.h"

#include "core/Json.h"

#include <array>
#include <cstdint>
#include <numeric>
#include <set>
#include <sstream>

using namespace texcore;
namespace json = tf::json;

namespace {

struct V3 {
    float x = 0, y = 0, z = 0;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float len(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 norm(V3 a) {
    const float l = len(a);
    return l > 1e-12f ? a * (1.0f / l) : V3{0, 1, 0};
}

// Vertex layout while processing (matches the glb attributes).
struct Vert {
    float p[3];
    float n[3];
    float uv[2];
    float wind[2];
};

struct Part {
    std::string material;  // source material name
    bool foliage = false;
    std::vector<Vert> verts;
    std::vector<std::uint32_t> idx;
    std::size_t tris() const { return idx.size() / 3; }
};

struct MaterialSpec {
    std::string name, albedo, alpha, normal, arm, rough; // arm: AO / roughness / metal; rough: roughness only
    bool normalGl = false; // OpenGL convention source (+Y up): converted to DirectX like the terrain maps
    bool write = true;
};

struct Lod {
    std::vector<Part> parts;
    double error = 0.0; // m, largest simplification error of the opaque parts
    std::size_t tris() const {
        std::size_t n = 0;
        for (const Part& p : parts) n += p.tris();
        return n;
    }
};

// ---------------------------------------------------------------- loading

void mat4mul(const float* m, const float* p, float w, float* out) {
    for (int r = 0; r < 3; ++r) out[r] = m[r] * p[0] + m[4 + r] * p[1] + m[8 + r] * p[2] + m[12 + r] * w;
}

void collectNode(const cgltf_node* node, const std::vector<std::string>& only, bool selected, std::vector<Part>& parts) {
    const bool sel = selected || only.empty() ||
                     (node->name && std::find(only.begin(), only.end(), std::string(node->name)) != only.end());
    if (sel && node->mesh) {
        float world[16];
        cgltf_node_transform_world(node, world);
        // Normal matrix: inverse transpose of the upper 3x3 (uniform scales in practice: the 3x3 itself, renormalized).
        for (cgltf_size pi = 0; pi < node->mesh->primitives_count; ++pi) {
            const cgltf_primitive& prim = node->mesh->primitives[pi];
            if (prim.type != cgltf_primitive_type_triangles) continue;
            const cgltf_material* mat = prim.material;
            const std::string mname = mat && mat->name ? mat->name : "default";
            int uvSet = 0;
            const cgltf_texture_view* tv = mat ? &mat->pbr_metallic_roughness.base_color_texture : nullptr;
            if (tv && tv->texture) uvSet = tv->texcoord;
            if (tv && tv->has_transform && tv->transform.has_texcoord) uvSet = tv->transform.texcoord;
            const cgltf_accessor *pos = nullptr, *nrm = nullptr, *uv = nullptr;
            for (cgltf_size a = 0; a < prim.attributes_count; ++a) {
                const cgltf_attribute& at = prim.attributes[a];
                if (at.type == cgltf_attribute_type_position) pos = at.data;
                else if (at.type == cgltf_attribute_type_normal) nrm = at.data;
                else if (at.type == cgltf_attribute_type_texcoord && at.index == uvSet) uv = at.data;
            }
            if (!pos) continue;
            const std::size_t n = pos->count;
            std::vector<float> P(n * 3), N(n * 3, 0.0f), UV(n * 2, 0.0f);
            cgltf_accessor_unpack_floats(pos, P.data(), P.size());
            if (nrm) cgltf_accessor_unpack_floats(nrm, N.data(), N.size());
            if (uv) cgltf_accessor_unpack_floats(uv, UV.data(), UV.size());
            // KHR_texture_transform of the base colour (the other maps of these assets use the same transform).
            float off[2] = {0, 0}, sc[2] = {1, 1}, rot = 0;
            if (tv && tv->has_transform) {
                off[0] = tv->transform.offset[0];
                off[1] = tv->transform.offset[1];
                sc[0] = tv->transform.scale[0];
                sc[1] = tv->transform.scale[1];
                rot = tv->transform.rotation;
            }
            Part* part = nullptr;
            for (Part& p : parts)
                if (p.material == mname) part = &p;
            if (!part) {
                parts.push_back({});
                part = &parts.back();
                part->material = mname;
                part->foliage = mat && (mat->alpha_mode == cgltf_alpha_mode_blend || mat->alpha_mode == cgltf_alpha_mode_mask);
            }
            const std::uint32_t base = std::uint32_t(part->verts.size());
            for (std::size_t i = 0; i < n; ++i) {
                Vert v{};
                mat4mul(world, &P[i * 3], 1.0f, v.p);
                float nn[3];
                mat4mul(world, &N[i * 3], 0.0f, nn);
                const V3 nv = norm({nn[0], nn[1], nn[2]});
                v.n[0] = nv.x;
                v.n[1] = nv.y;
                v.n[2] = nv.z;
                const float u0 = UV[i * 2] * sc[0], v0 = UV[i * 2 + 1] * sc[1];
                const float c = std::cos(rot), s = std::sin(rot);
                v.uv[0] = c * u0 + s * v0 + off[0];
                v.uv[1] = -s * u0 + c * v0 + off[1];
                part->verts.push_back(v);
            }
            if (prim.indices) {
                for (cgltf_size k = 0; k < prim.indices->count; ++k)
                    part->idx.push_back(base + std::uint32_t(cgltf_accessor_read_index(prim.indices, k)));
            } else {
                for (std::size_t k = 0; k < n; ++k) part->idx.push_back(base + std::uint32_t(k));
            }
        }
    }
    for (cgltf_size c = 0; c < node->children_count; ++c) collectNode(node->children[c], only, sel, parts);
}

// Welds identical vertices (position, normal, uv) and drops degenerate triangles.
void weld(Part& p) {
    std::vector<unsigned int> remap(p.verts.size());
    const std::size_t unique = meshopt_generateVertexRemap(remap.data(), p.idx.data(), p.idx.size(), p.verts.data(),
                                                           p.verts.size(), sizeof(Vert));
    std::vector<Vert> v(unique);
    meshopt_remapVertexBuffer(v.data(), p.verts.data(), p.verts.size(), sizeof(Vert), remap.data());
    std::vector<std::uint32_t> idx;
    idx.reserve(p.idx.size());
    for (std::size_t t = 0; t + 2 < p.idx.size(); t += 3) {
        const std::uint32_t a = remap[p.idx[t]], b = remap[p.idx[t + 1]], c = remap[p.idx[t + 2]];
        if (a == b || b == c || a == c) continue;
        idx.push_back(a);
        idx.push_back(b);
        idx.push_back(c);
    }
    p.verts.swap(v);
    p.idx.swap(idx);
}

// Removes unreferenced vertices and optimizes for the vertex cache.
void compact(Part& p) {
    if (p.idx.empty()) {
        p.verts.clear();
        return;
    }
    meshopt_optimizeVertexCache(p.idx.data(), p.idx.data(), p.idx.size(), p.verts.size());
    std::vector<Vert> v(p.verts.size());
    const std::size_t n = meshopt_optimizeVertexFetch(v.data(), p.idx.data(), p.idx.size(), p.verts.data(),
                                                      p.verts.size(), sizeof(Vert));
    v.resize(n);
    p.verts.swap(v);
}

// ---------------------------------------------------------------- simplification

float simplifyScale(const Part& p) { return meshopt_simplifyScale(&p.verts[0].p[0], p.verts.size(), sizeof(Vert)); }

// Opaque part to about `target` triangles; returns the absolute error (m).
double simplifyOpaque(Part& p, std::size_t target) {
    if (p.tris() <= target || p.idx.empty()) return 0.0;
    const float weights[5] = {0.5f, 0.5f, 0.5f, 0.2f, 0.2f}; // normal xyz, uv
    std::vector<std::uint32_t> out(p.idx.size());
    float err = 0.0f;
    std::size_t n = meshopt_simplifyWithAttributes(out.data(), p.idx.data(), p.idx.size(), &p.verts[0].p[0],
                                                   p.verts.size(), sizeof(Vert), &p.verts[0].n[0], sizeof(Vert),
                                                   weights, 5, nullptr, target * 3, 1.0f, meshopt_SimplifyPermissive,
                                                   &err);
    if (n > target * 3 * 3) {
        // Topology (seams, many small parts) keeps the count far above the target: the sloppy simplifier ignores
        // topology (may open small holes; reported).
        std::printf("  '%s': %zu triangles after the topology-preserving pass, sloppy simplification\n",
                    p.material.c_str(), n / 3);
        n = meshopt_simplifySloppy(out.data(), p.idx.data(), p.idx.size(), &p.verts[0].p[0], p.verts.size(),
                                   sizeof(Vert), nullptr, target * 3, 1.0f, &err);
    }
    out.resize(n);
    p.idx.swap(out);
    return double(err) * double(simplifyScale(p));
}

struct UnionFind {
    std::vector<std::uint32_t> parent;
    explicit UnionFind(std::size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), 0u); }
    std::uint32_t find(std::uint32_t a) {
        while (parent[a] != a) a = parent[a] = parent[parent[a]];
        return a;
    }
    void join(std::uint32_t a, std::uint32_t b) {
        a = find(a);
        b = find(b);
        if (a != b) parent[std::max(a, b)] = std::min(a, b);
    }
};

std::uint32_t hashU(std::uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

// Cards of a foliage part: triangles grouped by connected vertices.
struct Card {
    std::vector<std::uint32_t> tris; // first index of each triangle in Part::idx
    std::uint32_t key = 0;
};

std::vector<Card> cards(const Part& p) {
    UnionFind uf(p.verts.size());
    for (std::size_t t = 0; t + 2 < p.idx.size(); t += 3) {
        uf.join(p.idx[t], p.idx[t + 1]);
        uf.join(p.idx[t], p.idx[t + 2]);
    }
    std::map<std::uint32_t, std::size_t> slot;
    std::vector<Card> out;
    for (std::size_t t = 0; t + 2 < p.idx.size(); t += 3) {
        const std::uint32_t root = uf.find(p.idx[t]);
        auto it = slot.find(root);
        if (it == slot.end()) {
            it = slot.emplace(root, out.size()).first;
            out.push_back({});
            out.back().key = hashU(root * 2654435761u + 17u);
        }
        out[it->second].tris.push_back(std::uint32_t(t));
    }
    return out;
}

// Reduces every card to about two triangles (the simplifier never joins separate cards).
void reduceCards(Part& p, std::size_t budget) {
    const std::size_t n = cards(p).size();
    const std::size_t target = std::max(n * 2, budget);
    if (p.tris() <= target * 5 / 4) return;
    const float weights[2] = {1.0f, 1.0f}; // uv
    std::vector<std::uint32_t> out(p.idx.size());
    float err = 0.0f;
    const std::size_t k = meshopt_simplifyWithAttributes(out.data(), p.idx.data(), p.idx.size(), &p.verts[0].p[0],
                                                         p.verts.size(), sizeof(Vert), &p.verts[0].uv[0], sizeof(Vert),
                                                         weights, 2, nullptr, target * 3, 1.0f, 0, &err);
    out.resize(k);
    std::printf("  foliage '%s': %zu cards, %zu -> %zu triangles (card reduction)\n", p.material.c_str(), n, p.tris(),
                k / 3);
    p.idx.swap(out);
}

// Keeps about `target` triangles of whole cards (hash order) and grows the kept ones about their anchor.
void thinCards(Part& p, std::size_t target, float maxGrow) {
    if (p.tris() <= target) return;
    std::vector<Card> cs = cards(p);
    std::sort(cs.begin(), cs.end(), [](const Card& a, const Card& b) { return a.key < b.key; });
    std::vector<std::uint32_t> idx;
    std::vector<Vert> verts;
    std::size_t kept = 0;
    const std::size_t total = p.tris();
    std::vector<const Card*> keep;
    for (const Card& c : cs) {
        if (kept >= target) break;
        keep.push_back(&c);
        kept += c.tris.size();
    }
    const float grow = std::min(maxGrow, std::sqrt(float(total) / float(std::max<std::size_t>(kept, 1))));
    for (const Card* c : keep) {
        std::map<std::uint32_t, std::uint32_t> local;
        for (std::uint32_t t : c->tris)
            for (int k = 0; k < 3; ++k) local.emplace(p.idx[t + std::size_t(k)], 0u);
        // Anchor: the vertex nearest the trunk axis (cards hang off the branches outward).
        V3 anchor{};
        float best = 1e30f;
        for (auto& [vi, _] : local) {
            const Vert& v = p.verts[vi];
            const float d = v.p[0] * v.p[0] + v.p[2] * v.p[2];
            if (d < best) {
                best = d;
                anchor = {v.p[0], v.p[1], v.p[2]};
            }
        }
        for (auto& [vi, slotIdx] : local) {
            Vert v = p.verts[vi];
            const V3 q = anchor + (V3{v.p[0], v.p[1], v.p[2]} - anchor) * grow;
            v.p[0] = q.x;
            v.p[1] = q.y;
            v.p[2] = q.z;
            slotIdx = std::uint32_t(verts.size());
            verts.push_back(v);
        }
        for (std::uint32_t t : c->tris)
            for (int k = 0; k < 3; ++k) idx.push_back(local[p.idx[t + std::size_t(k)]]);
    }
    p.verts.swap(verts);
    p.idx.swap(idx);
}

// ---------------------------------------------------------------- textures

struct Image {
    int w = 0, h = 0;
    std::vector<float> px; // linear RGBA
    const float* at(int x, int y) const {
        x = ((x % w) + w) % w;
        y = ((y % h) + h) % h;
        return &px[(std::size_t(y) * std::size_t(w) + std::size_t(x)) * 4];
    }
    void sample(float u, float v, float* out) const { // bilinear, repeat
        const float x = u * float(w) - 0.5f, y = v * float(h) - 0.5f;
        const int x0 = int(std::floor(x)), y0 = int(std::floor(y));
        const float fx = x - float(x0), fy = y - float(y0);
        const float *a = at(x0, y0), *b = at(x0 + 1, y0), *c = at(x0, y0 + 1), *d = at(x0 + 1, y0 + 1);
        for (int k = 0; k < 4; ++k)
            out[k] = (a[k] * (1 - fx) + b[k] * fx) * (1 - fy) + (c[k] * (1 - fx) + d[k] * fx) * fy;
    }
};

Image toImage(const Plane& p) {
    Image im;
    im.w = p.w;
    im.h = p.h;
    im.px = p.px;
    return im;
}

// Share of texels with alpha >= 0.5 after scaling alpha by s.
float coverage(const Plane& p, float s) {
    std::size_t n = 0;
    for (std::size_t i = 3; i < p.px.size(); i += 4) n += p.px[i] * s >= 0.5f ? 1 : 0;
    return float(n) / float(std::max<std::size_t>(p.px.size() / 4, 1));
}

// Fills the colour of transparent texels from their opaque neighbours (no dark fringes in the mips).
void dilate(Plane& p, int passes) {
    for (int it = 0; it < passes; ++it) {
        Plane q = p;
        for (int y = 0; y < p.h; ++y)
            for (int x = 0; x < p.w; ++x) {
                float* o = q.at(x, y);
                if (p.at(x, y)[3] > 0.0f) continue;
                float acc[3] = {0, 0, 0};
                int cnt = 0;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int xx = x + dx, yy = y + dy;
                        if (xx < 0 || yy < 0 || xx >= p.w || yy >= p.h) continue;
                        const float* s = p.at(xx, yy);
                        if (s[3] <= 0.0f && !(s[0] + s[1] + s[2] > 0.0f)) continue;
                        acc[0] += s[0];
                        acc[1] += s[1];
                        acc[2] += s[2];
                        ++cnt;
                    }
                if (cnt > 0)
                    for (int k = 0; k < 3; ++k) o[k] = acc[k] / float(cnt);
            }
        p = q;
    }
}

// Encodes a linear RGBA plane (sRGB albedo + alpha, or normal / roughness / AO) with a full mip chain. Alpha-tested
// maps keep the level 0 coverage in every mip (alpha rescaled per level).
std::size_t writeKtx(Plane base, bool srgb, bool alphaTest, const std::string& path, double* psnrOut = nullptr) {
    tf::ktx2::Image image;
    image.vkFormat = srgb ? tf::ktx2::kVkFormatBc7Srgb : tf::ktx2::kVkFormatBc7Unorm;
    image.width = std::uint32_t(base.w);
    image.height = std::uint32_t(base.h);
    const float cov0 = alphaTest ? coverage(base, 1.0f) : 0.0f;
    double sq0 = 0;
    std::size_t n0 = 0;
    for (int level = 0;; ++level) {
        if (level > 0) {
            if (base.w == 1 && base.h == 1) break;
            base = half(base);
        }
        float scale = 1.0f;
        if (alphaTest && level > 0) {
            float lo = 0.5f, hi = 8.0f;
            for (int i = 0; i < 16; ++i) {
                const float mid = 0.5f * (lo + hi);
                (coverage(base, mid) < cov0 ? lo : hi) = mid;
            }
            scale = 0.5f * (lo + hi);
        }
        std::vector<std::uint8_t> rgba(std::size_t(base.w) * std::size_t(base.h) * 4);
        for (std::size_t i = 0; i < rgba.size(); i += 4) {
            const float* p = &base.px[i];
            for (int c = 0; c < 3; ++c) rgba[i + std::size_t(c)] = q8(srgb ? linearToSrgb(p[c]) : p[c]);
            rgba[i + 3] = q8(p[3] * scale);
        }
        Encoded e = encode(rgba, base.w, base.h, srgb);
        if (level == 0) {
            sq0 = e.sqErr;
            n0 = e.samples;
        }
        image.levels.push_back(std::move(e.level));
    }
    const std::vector<std::uint8_t> bytes = tf::ktx2::write(image, "trailforge_modelc (bc7enc_rdo b9438627)");
    writeFile(path, bytes);
    const double mse = sq0 / double(std::max<std::size_t>(n0, 1));
    const double psnr = mse > 0 ? 10.0 * std::log10(255.0 * 255.0 / mse) : 99.0;
    if (psnrOut) *psnrOut = psnr;
    std::printf("  %s: %dx%d, %zu levels, %zu bytes, PSNR %.1f dB\n", path.c_str(), int(image.width),
                int(image.height), image.levels.size(), bytes.size(), psnr);
    return bytes.size();
}

// Material maps: albedo (sRGB, alpha from the alpha map or opaque), nra (normal x / y DirectX, roughness, AO).
void writeMaterialTextures(const MaterialSpec& m, bool foliage, int size, int normalSize, const std::string& dir) {
    Plane a = load(m.albedo, size);
    for (std::size_t i = 0; i < a.px.size(); i += 4) {
        for (int c = 0; c < 3; ++c) a.px[i + std::size_t(c)] = srgbToLinear(a.px[i + std::size_t(c)]);
        a.px[i + 3] = 1.0f;
    }
    if (foliage && !m.alpha.empty()) {
        const Plane al = load(m.alpha, size);
        for (std::size_t i = 0; i < a.px.size(); i += 4) a.px[i + 3] = al.px[i];
        Plane d = a;
        for (std::size_t i = 3; i < d.px.size(); i += 4) d.px[i] = d.px[i] >= 0.5f ? 1.0f : 0.0f;
        dilate(d, 6);
        for (std::size_t i = 0; i < a.px.size(); i += 4)
            if (a.px[i + 3] < 0.5f)
                for (int c = 0; c < 3; ++c) a.px[i + std::size_t(c)] = d.px[i + std::size_t(c)];
    }
    writeKtx(a, true, foliage, dir + "/" + m.name + "_albedo.ktx2");

    Plane n = load(m.normal, normalSize);
    const bool hasArm = !m.arm.empty();
    const Plane arm = load(hasArm ? m.arm : m.rough, normalSize);
    for (std::size_t i = 0; i < n.px.size(); i += 4) {
        const float g = m.normalGl ? 1.0f - n.px[i + 1] : n.px[i + 1];
        V3 v = norm({n.px[i] * 2 - 1, g * 2 - 1, n.px[i + 2] * 2 - 1});
        n.px[i] = v.x * 0.5f + 0.5f;
        n.px[i + 1] = v.y * 0.5f + 0.5f;
        n.px[i + 2] = hasArm ? arm.px[i + 1] : arm.px[i]; // roughness
        n.px[i + 3] = hasArm ? arm.px[i] : 1.0f;          // AO
    }
    writeKtx(n, false, false, dir + "/" + m.name + "_nra.ktx2");
}

// ---------------------------------------------------------------- impostor

struct Bounds {
    V3 lo{1e30f, 1e30f, 1e30f}, hi{-1e30f, -1e30f, -1e30f};
    void add(const float* p) {
        lo = {std::min(lo.x, p[0]), std::min(lo.y, p[1]), std::min(lo.z, p[2])};
        hi = {std::max(hi.x, p[0]), std::max(hi.y, p[1]), std::max(hi.z, p[2])};
    }
};

// One atlas row of `views` tiles (tileW x tileH). View k sees the model from direction e = (sin a, 0, cos a),
// a = 2 pi k / views; tile x spans [-radius, radius] along r = (cos a, 0, -sin a), tile y spans [height, 0].
// Albedo: linear rgb + coverage. Normal plane: view-space normal (x along r, y up, z toward the viewer) * 0.5 + 0.5,
// roughness, AO (deeper parts of the crown darker).
void bakeImpostor(const Lod& lod, const std::map<std::string, Image>& albedos, const std::map<std::string, bool>& foliage,
                  float radius, float height, int views, int tileW, int tileH, Plane& outAlbedo, Plane& outNormal) {
    const int ss = 2;
    const int W = tileW * ss, H = tileH * ss;
    outAlbedo.w = outNormal.w = tileW * views;
    outAlbedo.h = outNormal.h = tileH;
    outAlbedo.px.assign(std::size_t(outAlbedo.w) * std::size_t(outAlbedo.h) * 4, 0.0f);
    outNormal.px.assign(outAlbedo.px.size(), 0.0f);
    for (int k = 0; k < views; ++k) {
        const float a = 2.0f * 3.14159265f * float(k) / float(views);
        const V3 e{std::sin(a), 0, std::cos(a)}, r{std::cos(a), 0, -std::sin(a)};
        std::vector<float> depth(std::size_t(W) * std::size_t(H), -1e30f);
        std::vector<float> col(std::size_t(W) * std::size_t(H) * 3, 0.0f), nrm(std::size_t(W) * std::size_t(H) * 3, 0.0f);
        std::vector<float> ao(std::size_t(W) * std::size_t(H), 1.0f);
        for (const Part& part : lod.parts) {
            const auto ai = albedos.find(part.material);
            const Image* tex = ai != albedos.end() ? &ai->second : nullptr;
            const bool fol = foliage.at(part.material);
            for (std::size_t t = 0; t + 2 < part.idx.size(); t += 3) {
                const Vert* v[3] = {&part.verts[part.idx[t]], &part.verts[part.idx[t + 1]], &part.verts[part.idx[t + 2]]};
                float sx[3], sy[3], sz[3];
                for (int i = 0; i < 3; ++i) {
                    const V3 p{v[i]->p[0], v[i]->p[1], v[i]->p[2]};
                    sx[i] = (dot(p, r) / radius * 0.5f + 0.5f) * float(W);
                    sy[i] = (1.0f - p.y / height) * float(H);
                    sz[i] = dot(p, e);
                }
                const float area = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
                if (std::abs(area) < 1e-9f) continue;
                const int x0 = std::max(0, int(std::floor(std::min({sx[0], sx[1], sx[2]})))),
                          x1 = std::min(W - 1, int(std::ceil(std::max({sx[0], sx[1], sx[2]}))));
                const int y0 = std::max(0, int(std::floor(std::min({sy[0], sy[1], sy[2]})))),
                          y1 = std::min(H - 1, int(std::ceil(std::max({sy[0], sy[1], sy[2]}))));
                for (int y = y0; y <= y1; ++y)
                    for (int x = x0; x <= x1; ++x) {
                        const float px = float(x) + 0.5f, py = float(y) + 0.5f;
                        float w0 = (sx[1] - px) * (sy[2] - py) - (sx[2] - px) * (sy[1] - py);
                        float w1 = (sx[2] - px) * (sy[0] - py) - (sx[0] - px) * (sy[2] - py);
                        float w2 = (sx[0] - px) * (sy[1] - py) - (sx[1] - px) * (sy[0] - py);
                        if (area < 0) {
                            w0 = -w0;
                            w1 = -w1;
                            w2 = -w2;
                        }
                        if (w0 < 0 || w1 < 0 || w2 < 0) continue;
                        const float inv = 1.0f / std::abs(area);
                        w0 *= inv;
                        w1 *= inv;
                        w2 *= inv;
                        const float z = w0 * sz[0] + w1 * sz[1] + w2 * sz[2];
                        const std::size_t o = std::size_t(y) * std::size_t(W) + std::size_t(x);
                        if (z <= depth[o]) continue;
                        float c[4] = {0.3f, 0.3f, 0.3f, 1.0f};
                        if (tex) {
                            const float u = w0 * v[0]->uv[0] + w1 * v[1]->uv[0] + w2 * v[2]->uv[0];
                            const float vv = w0 * v[0]->uv[1] + w1 * v[1]->uv[1] + w2 * v[2]->uv[1];
                            tex->sample(u, vv, c);
                        }
                        if (fol && c[3] < 0.5f) continue;
                        depth[o] = z;
                        V3 n = norm({w0 * v[0]->n[0] + w1 * v[1]->n[0] + w2 * v[2]->n[0],
                                     w0 * v[0]->n[1] + w1 * v[1]->n[1] + w2 * v[2]->n[1],
                                     w0 * v[0]->n[2] + w1 * v[1]->n[2] + w2 * v[2]->n[2]});
                        V3 vn{dot(n, r), n.y, dot(n, e)};
                        if (!fol && vn.z < 0) vn = vn * -1.0f;
                        col[o * 3] = c[0];
                        col[o * 3 + 1] = c[1];
                        col[o * 3 + 2] = c[2];
                        nrm[o * 3] = vn.x;
                        nrm[o * 3 + 1] = vn.y;
                        nrm[o * 3 + 2] = vn.z;
                        ao[o] = fol ? 0.55f + 0.45f * std::clamp((z / radius) * 0.5f + 0.5f, 0.0f, 1.0f) : 1.0f;
                    }
            }
        }
        // 2 x 2 box down into the tile; coverage = share of covered subsamples, colour = mean of the covered ones.
        for (int y = 0; y < tileH; ++y)
            for (int x = 0; x < tileW; ++x) {
                float c[3] = {0, 0, 0}, n[3] = {0, 0, 0}, a = 0;
                int cnt = 0;
                for (int j = 0; j < ss; ++j)
                    for (int i = 0; i < ss; ++i) {
                        const std::size_t o = std::size_t(y * ss + j) * std::size_t(W) + std::size_t(x * ss + i);
                        if (depth[o] <= -1e29f) continue;
                        for (int q = 0; q < 3; ++q) {
                            c[q] += col[o * 3 + std::size_t(q)];
                            n[q] += nrm[o * 3 + std::size_t(q)];
                        }
                        a += ao[o];
                        ++cnt;
                    }
                float* pa = outAlbedo.at(k * tileW + x, y);
                float* pn = outNormal.at(k * tileW + x, y);
                if (cnt == 0) continue;
                const V3 nn = norm({n[0], n[1], n[2]});
                for (int q = 0; q < 3; ++q) pa[q] = c[q] / float(cnt);
                pa[3] = float(cnt) / float(ss * ss);
                pn[0] = nn.x * 0.5f + 0.5f;
                pn[1] = nn.y * 0.5f + 0.5f;
                pn[2] = 0.85f;
                pn[3] = a / float(cnt);
            }
    }
}

// ---------------------------------------------------------------- foliage clump cards
// Trees whose crowns are hundreds of thousands of small needle cards cannot be thinned to a game budget without
// losing the crown. Instead the source foliage is split into spatial clusters (median splits), each cluster becomes
// one quad on its best-fit plane (tilted toward the outside of the crown), and the quads show baked clumps: per LOD a
// small library of real clusters rasterized onto their planes (albedo + coverage, normals in the card frame).

struct FoliageTri {
    V3 p[3], n[3];
    float uv[3][2];
    const Image* tex;
    V3 c;
};

struct Cluster {
    std::vector<std::uint32_t> tris;
    V3 c, u, vb, n; // centre, card axes (u right, vb down the tile), plane normal
    float w = 0, h = 0;
    std::uint32_t key = 0;
};

// Direction n minimizing n^T A n (best-fit plane normal of a covariance A): 1024 Fibonacci directions on the upper
// hemisphere, then three refinement rounds around the best one.
V3 planeNormal(const double a[3][3]) {
    auto energy = [&](V3 n) {
        const double v[3] = {n.x, n.y, n.z};
        double e = 0;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) e += v[r] * a[r][c] * v[c];
        return e;
    };
    V3 best{0, 1, 0};
    double bestE = energy(best);
    const int count = 1024;
    for (int i = 0; i < count; ++i) {
        const float y = 1.0f - (float(i) + 0.5f) / float(count);
        const float r = std::sqrt(std::max(0.0f, 1.0f - y * y)), phi = 2.39996323f * float(i);
        const V3 n{r * std::cos(phi), y, r * std::sin(phi)};
        const double e = energy(n);
        if (e < bestE) {
            bestE = e;
            best = n;
        }
    }
    float step = 0.06f;
    for (int round = 0; round < 3; ++round, step *= 0.3f) {
        V3 t1 = cross(best, std::abs(best.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0});
        t1 = norm(t1);
        const V3 t2 = cross(best, t1);
        const V3 centre = best;
        for (int j = -4; j <= 4; ++j)
            for (int i = -4; i <= 4; ++i) {
                const V3 n = norm(centre + t1 * (step * float(i)) + t2 * (step * float(j)));
                const double e = energy(n);
                if (e < bestE) {
                    bestE = e;
                    best = n;
                }
            }
    }
    return best;
}

std::vector<Cluster> clusterFoliage(const std::vector<FoliageTri>& tris, std::size_t count, float tilt) {
    std::vector<std::uint32_t> order(tris.size());
    std::iota(order.begin(), order.end(), 0u);
    std::vector<Cluster> out;
    struct Node {
        std::size_t a, b, k; // triangle range, clusters to make from it
    };
    std::vector<Node> stack{{0, order.size(), std::max<std::size_t>(1, std::min(count, tris.size()))}};
    while (!stack.empty()) {
        const auto [a, b, k] = stack.back();
        stack.pop_back();
        if (k <= 1 || b - a <= 1) {
            Cluster cl;
            cl.tris.assign(order.begin() + std::ptrdiff_t(a), order.begin() + std::ptrdiff_t(b));
            out.push_back(std::move(cl));
            continue;
        }
        V3 lo{1e30f, 1e30f, 1e30f}, hi{-1e30f, -1e30f, -1e30f};
        for (std::size_t i = a; i < b; ++i) {
            const V3& c = tris[order[i]].c;
            lo = {std::min(lo.x, c.x), std::min(lo.y, c.y), std::min(lo.z, c.z)};
            hi = {std::max(hi.x, c.x), std::max(hi.y, c.y), std::max(hi.z, c.z)};
        }
        const V3 ext = hi - lo;
        const int axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : (ext.y >= ext.z ? 1 : 2);
        auto coord = [&](std::uint32_t t) {
            const V3& c = tris[t].c;
            return axis == 0 ? c.x : axis == 1 ? c.y : c.z;
        };
        const std::size_t kl = k / 2;
        const std::size_t mid = a + std::size_t(double(b - a) * double(kl) / double(k));
        std::nth_element(order.begin() + std::ptrdiff_t(a), order.begin() + std::ptrdiff_t(mid),
                         order.begin() + std::ptrdiff_t(b),
                         [&](std::uint32_t x, std::uint32_t y) { return coord(x) < coord(y); });
        stack.push_back({mid, b, k - kl});
        stack.push_back({a, mid, kl});
    }
    for (std::size_t ci = 0; ci < out.size(); ++ci) {
        Cluster& cl = out[ci];
        double mean[3] = {0, 0, 0};
        std::size_t n = 0;
        for (std::uint32_t t : cl.tris)
            for (const V3& p : tris[t].p) {
                mean[0] += p.x;
                mean[1] += p.y;
                mean[2] += p.z;
                ++n;
            }
        for (double& m : mean) m /= double(std::max<std::size_t>(n, 1));
        double cov[3][3] = {};
        for (std::uint32_t t : cl.tris)
            for (const V3& p : tris[t].p) {
                const double d[3] = {p.x - mean[0], p.y - mean[1], p.z - mean[2]};
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c) cov[r][c] += d[r] * d[c];
            }
        const V3 c{float(mean[0]), float(mean[1]), float(mean[2])};
        V3 nrm = planeNormal(cov);
        V3 radial{c.x, 0, c.z};
        radial = len(radial) > 0.05f ? norm(radial) : V3{0, 0, 1};
        if (dot(nrm, radial) < 0) nrm = nrm * -1.0f;
        nrm = norm(nrm * (1.0f - tilt) + radial * tilt);
        V3 u = cross(V3{0, 1, 0}, nrm);
        u = len(u) > 0.1f ? norm(u) : V3{1, 0, 0};
        const V3 vb = norm(cross(u, nrm)); // down the tile for upright cards
        float s0 = 1e30f, s1 = -1e30f, t0 = 1e30f, t1 = -1e30f;
        for (std::uint32_t t : cl.tris)
            for (const V3& p : tris[t].p) {
                const float s = dot(p - c, u), tt = dot(p - c, vb);
                s0 = std::min(s0, s);
                s1 = std::max(s1, s);
                t0 = std::min(t0, tt);
                t1 = std::max(t1, tt);
            }
        cl.w = std::max(s1 - s0, 0.05f) * 1.04f;
        cl.h = std::max(t1 - t0, 0.05f) * 1.04f;
        cl.c = c + u * (0.5f * (s0 + s1)) + vb * (0.5f * (t0 + t1));
        cl.u = u;
        cl.vb = vb;
        cl.n = nrm;
        cl.key = hashU(std::uint32_t(ci) * 2654435761u + 7u);
    }
    return out;
}

// Rasterizes a cluster onto its card into a tile x tile region of the atlas planes (2 x 2 supersampled). Albedo:
// linear rgb + coverage; normal plane: source normal in the card frame (x along u, y along vb, z along n) * 0.5 + 0.5,
// roughness 0.8, AO 1.
void bakeClusterTile(const Cluster& cl, const std::vector<FoliageTri>& tris, int tile, int ox, int oy, Plane& albedo,
                     Plane& normal) {
    const int S = tile * 2;
    std::vector<float> depth(std::size_t(S) * std::size_t(S), -1e30f), col(std::size_t(S) * std::size_t(S) * 3, 0.0f),
        nrm(std::size_t(S) * std::size_t(S) * 3, 0.0f);
    for (std::uint32_t ti : cl.tris) {
        const FoliageTri& t = tris[ti];
        float sx[3], sy[3], sz[3];
        for (int i = 0; i < 3; ++i) {
            const V3 d = t.p[i] - cl.c;
            sx[i] = (dot(d, cl.u) / cl.w + 0.5f) * float(S);
            sy[i] = (dot(d, cl.vb) / cl.h + 0.5f) * float(S);
            sz[i] = dot(d, cl.n);
        }
        const float area = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
        if (std::abs(area) < 1e-9f) continue;
        const int x0 = std::max(0, int(std::floor(std::min({sx[0], sx[1], sx[2]})))),
                  x1 = std::min(S - 1, int(std::ceil(std::max({sx[0], sx[1], sx[2]}))));
        const int y0 = std::max(0, int(std::floor(std::min({sy[0], sy[1], sy[2]})))),
                  y1 = std::min(S - 1, int(std::ceil(std::max({sy[0], sy[1], sy[2]}))));
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                const float px = float(x) + 0.5f, py = float(y) + 0.5f;
                float w0 = (sx[1] - px) * (sy[2] - py) - (sx[2] - px) * (sy[1] - py);
                float w1 = (sx[2] - px) * (sy[0] - py) - (sx[0] - px) * (sy[2] - py);
                float w2 = (sx[0] - px) * (sy[1] - py) - (sx[1] - px) * (sy[0] - py);
                if (area < 0) {
                    w0 = -w0;
                    w1 = -w1;
                    w2 = -w2;
                }
                if (w0 < 0 || w1 < 0 || w2 < 0) continue;
                const float inv = 1.0f / std::abs(area);
                w0 *= inv;
                w1 *= inv;
                w2 *= inv;
                const float z = w0 * sz[0] + w1 * sz[1] + w2 * sz[2];
                const std::size_t o = std::size_t(y) * std::size_t(S) + std::size_t(x);
                if (z <= depth[o]) continue;
                float c[4];
                t.tex->sample(w0 * t.uv[0][0] + w1 * t.uv[1][0] + w2 * t.uv[2][0],
                              w0 * t.uv[0][1] + w1 * t.uv[1][1] + w2 * t.uv[2][1], c);
                if (c[3] < 0.5f) continue;
                depth[o] = z;
                const V3 n = norm(t.n[0] * w0 + t.n[1] * w1 + t.n[2] * w2);
                col[o * 3] = c[0];
                col[o * 3 + 1] = c[1];
                col[o * 3 + 2] = c[2];
                nrm[o * 3] = dot(n, cl.u);
                nrm[o * 3 + 1] = dot(n, cl.vb);
                nrm[o * 3 + 2] = dot(n, cl.n);
            }
    }
    for (int y = 0; y < tile; ++y)
        for (int x = 0; x < tile; ++x) {
            float c[3] = {0, 0, 0}, n[3] = {0, 0, 0};
            int cnt = 0;
            for (int j = 0; j < 2; ++j)
                for (int i = 0; i < 2; ++i) {
                    const std::size_t o = std::size_t(y * 2 + j) * std::size_t(S) + std::size_t(x * 2 + i);
                    if (depth[o] <= -1e29f) continue;
                    for (int q = 0; q < 3; ++q) {
                        c[q] += col[o * 3 + std::size_t(q)];
                        n[q] += nrm[o * 3 + std::size_t(q)];
                    }
                    ++cnt;
                }
            float* pa = albedo.at(ox + x, oy + y);
            float* pn = normal.at(ox + x, oy + y);
            pn[0] = 0.5f;
            pn[1] = 0.5f;
            pn[2] = 0.8f;
            pn[3] = 1.0f;
            if (cnt == 0) continue;
            V3 nn = norm({n[0], n[1], n[2]});
            if (nn.z < 0) nn = nn * -1.0f; // both faces of a card show the same side
            for (int q = 0; q < 3; ++q) pa[q] = c[q] / float(cnt);
            pa[3] = float(cnt) / 4.0f;
            pn[0] = nn.x * 0.5f + 0.5f;
            pn[1] = nn.y * 0.5f + 0.5f;
        }
}

// Library tiles for one LOD: `tiles` clusters from the middle of the size range, spread over the crown.
std::vector<std::size_t> pickLibrary(const std::vector<Cluster>& cls, std::size_t tiles) {
    std::vector<std::size_t> bySize(cls.size());
    std::iota(bySize.begin(), bySize.end(), std::size_t(0));
    std::sort(bySize.begin(), bySize.end(), [&](std::size_t a, std::size_t b) {
        const float sa = cls[a].w * cls[a].h, sb = cls[b].w * cls[b].h;
        return sa != sb ? sa < sb : cls[a].key < cls[b].key;
    });
    const std::size_t lo = bySize.size() / 5, hi = std::max(lo + 1, bySize.size() * 4 / 5);
    std::vector<std::size_t> mid(bySize.begin() + std::ptrdiff_t(lo), bySize.begin() + std::ptrdiff_t(hi));
    std::sort(mid.begin(), mid.end(), [&](std::size_t a, std::size_t b) { return cls[a].key < cls[b].key; });
    if (mid.size() > tiles) mid.resize(tiles);
    return mid;
}

// ---------------------------------------------------------------- glb writer

struct GlbWriter {
    std::vector<std::uint8_t> bin;
    json::Value views = json::Value::makeArray(), accessors = json::Value::makeArray();

    int addView(const void* data, std::size_t bytes, int target) {
        while (bin.size() % 4) bin.push_back(0);
        const std::size_t off = bin.size();
        bin.insert(bin.end(), static_cast<const std::uint8_t*>(data), static_cast<const std::uint8_t*>(data) + bytes);
        json::Value v = json::Value::makeObject();
        v.object["buffer"] = json::Value::makeNumber(0);
        v.object["byteOffset"] = json::Value::makeNumber(double(off));
        v.object["byteLength"] = json::Value::makeNumber(double(bytes));
        if (target) v.object["target"] = json::Value::makeNumber(target);
        views.array.push_back(v);
        return int(views.array.size()) - 1;
    }
    int addAccessor(int view, int componentType, std::size_t count, const char* type, const float* mn = nullptr,
                    const float* mx = nullptr, int comps = 0) {
        json::Value a = json::Value::makeObject();
        a.object["bufferView"] = json::Value::makeNumber(view);
        a.object["componentType"] = json::Value::makeNumber(componentType);
        a.object["count"] = json::Value::makeNumber(double(count));
        a.object["type"] = json::Value::makeString(type);
        if (mn && mx) {
            json::Value lo = json::Value::makeArray(), hi = json::Value::makeArray();
            for (int i = 0; i < comps; ++i) {
                lo.array.push_back(json::Value::makeNumber(mn[i]));
                hi.array.push_back(json::Value::makeNumber(mx[i]));
            }
            a.object["min"] = lo;
            a.object["max"] = hi;
        }
        accessors.array.push_back(a);
        return int(accessors.array.size()) - 1;
    }
};

json::Value meshJson(GlbWriter& w, const std::string& name, const std::vector<Part>& parts,
                     const std::map<std::string, int>& matIndex, bool positionsOnly) {
    json::Value mesh = json::Value::makeObject();
    mesh.object["name"] = json::Value::makeString(name);
    json::Value prims = json::Value::makeArray();
    for (const Part& p : parts) {
        if (p.idx.empty()) continue;
        const std::size_t n = p.verts.size();
        std::vector<float> P(n * 3), N(n * 3), UV(n * 2), WI(n * 2);
        float mn[3] = {1e30f, 1e30f, 1e30f}, mx[3] = {-1e30f, -1e30f, -1e30f};
        for (std::size_t i = 0; i < n; ++i) {
            const Vert& v = p.verts[i];
            for (int k = 0; k < 3; ++k) {
                P[i * 3 + std::size_t(k)] = v.p[k];
                N[i * 3 + std::size_t(k)] = v.n[k];
                mn[k] = std::min(mn[k], v.p[k]);
                mx[k] = std::max(mx[k], v.p[k]);
            }
            UV[i * 2] = v.uv[0];
            UV[i * 2 + 1] = v.uv[1];
            WI[i * 2] = v.wind[0];
            WI[i * 2 + 1] = v.wind[1];
        }
        json::Value prim = json::Value::makeObject();
        json::Value attrs = json::Value::makeObject();
        attrs.object["POSITION"] = json::Value::makeNumber(
            w.addAccessor(w.addView(P.data(), P.size() * 4, 34962), 5126, n, "VEC3", mn, mx, 3));
        if (!positionsOnly) {
            attrs.object["NORMAL"] = json::Value::makeNumber(w.addAccessor(w.addView(N.data(), N.size() * 4, 34962), 5126, n, "VEC3"));
            attrs.object["TEXCOORD_0"] = json::Value::makeNumber(w.addAccessor(w.addView(UV.data(), UV.size() * 4, 34962), 5126, n, "VEC2"));
            attrs.object["_WIND"] = json::Value::makeNumber(w.addAccessor(w.addView(WI.data(), WI.size() * 4, 34962), 5126, n, "VEC2"));
        }
        prim.object["attributes"] = attrs;
        if (n < 65536) {
            std::vector<std::uint16_t> idx(p.idx.begin(), p.idx.end());
            prim.object["indices"] = json::Value::makeNumber(w.addAccessor(w.addView(idx.data(), idx.size() * 2, 34963), 5123, idx.size(), "SCALAR"));
        } else {
            prim.object["indices"] = json::Value::makeNumber(w.addAccessor(w.addView(p.idx.data(), p.idx.size() * 4, 34963), 5125, p.idx.size(), "SCALAR"));
        }
        if (!positionsOnly) prim.object["material"] = json::Value::makeNumber(matIndex.at(p.material));
        prims.array.push_back(prim);
    }
    mesh.object["primitives"] = prims;
    return mesh;
}

void writeGlb(const std::string& path, const json::Value& doc, const std::vector<std::uint8_t>& binIn) {
    std::string text = json::write(doc);
    while (text.size() % 4) text.push_back(' ');
    std::vector<std::uint8_t> bin = binIn;
    while (bin.size() % 4) bin.push_back(0);
    std::vector<std::uint8_t> out;
    auto u32 = [&](std::uint32_t v) {
        for (int i = 0; i < 4; ++i) out.push_back(std::uint8_t(v >> (8 * i)));
    };
    u32(0x46546C67u); // "glTF"
    u32(2);
    u32(std::uint32_t(12 + 8 + text.size() + 8 + bin.size()));
    u32(std::uint32_t(text.size()));
    u32(0x4E4F534Au); // "JSON"
    out.insert(out.end(), text.begin(), text.end());
    u32(std::uint32_t(bin.size()));
    u32(0x004E4942u); // "BIN"
    out.insert(out.end(), bin.begin(), bin.end());
    writeFile(path, out);
}

json::Value num(double v) { return json::Value::makeNumber(v); }
json::Value str(const std::string& s) { return json::Value::makeString(s); }

} // namespace

int main(int argc, char** argv) {
    if (argc != 3 || std::strcmp(argv[1], "--spec") != 0) {
        std::fprintf(stderr, "usage: trailforge_modelc --spec spec.json\n");
        return 2;
    }
    std::ifstream in(argv[2]);
    if (!in) die(std::string("cannot read ") + argv[2]);
    std::stringstream ss;
    ss << in.rdbuf();
    json::Value spec;
    std::string error;
    if (!json::parse(ss.str(), spec, &error)) die(std::string(argv[2]) + ": " + error);
    const std::string id = spec["id"].stringOr("model"), kind = spec["kind"].stringOr("rock");
    const std::string outDir = spec["out"].stringOr(".");
    const std::string gltfPath = spec["gltf"].stringOr("");
    std::vector<std::string> nodes;
    for (const json::Value& v : spec["nodes"].array) nodes.push_back(v.string);
    std::vector<std::size_t> budgets;
    for (const json::Value& v : spec["lods"].array) budgets.push_back(std::size_t(v.number));
    if (budgets.empty()) budgets = {4000};
    const int collisionLod = int(spec["collisionLod"].numberOr(-1));
    const bool atBase = spec["origin"].stringOr(kind == "rock" || kind == "cliff" ? "center" : "base") == "base";
    const float scale = float(spec["scale"].numberOr(1.0));
    bc7enc_compress_block_init();

    cgltf_options opts{};
    cgltf_data* data = nullptr;
    if (cgltf_parse_file(&opts, gltfPath.c_str(), &data) != cgltf_result_success) die("cannot parse " + gltfPath);
    if (cgltf_load_buffers(&opts, data, gltfPath.c_str()) != cgltf_result_success) die("cannot load buffers of " + gltfPath);
    std::vector<Part> parts;
    const cgltf_scene* scene = data->scene ? data->scene : (data->scenes_count ? &data->scenes[0] : nullptr);
    if (scene) {
        for (cgltf_size i = 0; i < scene->nodes_count; ++i) collectNode(scene->nodes[i], nodes, false, parts);
    } else {
        for (cgltf_size i = 0; i < data->nodes_count; ++i)
            if (!data->nodes[i].parent) collectNode(&data->nodes[i], nodes, false, parts);
    }
    cgltf_free(data);
    if (parts.empty()) die("no triangles in the selected nodes");

    // Materials; a part is alpha-tested foliage when its material has an alpha map.
    std::map<std::string, MaterialSpec> mats;
    for (const auto& [src, v] : spec["materials"].object) {
        MaterialSpec m;
        m.name = v["name"].stringOr(src);
        m.albedo = v["albedo"].stringOr("");
        m.alpha = v["alpha"].stringOr("");
        m.normal = v["normal"].stringOr("");
        m.arm = v["arm"].stringOr("");
        m.rough = v["rough"].stringOr("");
        m.normalGl = v["normalGl"].boolOr(false);
        m.write = v["write"].boolOr(true);
        mats[src] = m;
    }
    for (Part& p : parts) {
        const auto it = mats.find(p.material);
        if (it == mats.end()) die("spec has no textures for material '" + p.material + "'");
        p.foliage = !it->second.alpha.empty();
    }

    // Cliff faces are open scanned shells: turn them so the area-weighted mean normal of the face is +Z (a vertical
    // wall facing +Z; the level places them with a yaw only).
    if (kind == "cliff") {
        V3 mean{};
        for (const Part& p : parts)
            for (std::size_t t = 0; t + 2 < p.idx.size(); t += 3) {
                const float *a = p.verts[p.idx[t]].p, *b = p.verts[p.idx[t + 1]].p, *c = p.verts[p.idx[t + 2]].p;
                mean = mean + cross(V3{b[0] - a[0], b[1] - a[1], b[2] - a[2]}, V3{c[0] - a[0], c[1] - a[1], c[2] - a[2]});
            }
        const V3 n = norm(mean), z{0, 0, 1};
        const V3 axisRaw = cross(n, z);
        const float s = len(axisRaw), c = dot(n, z);
        if (s > 1e-4f) {
            const V3 k = axisRaw * (1.0f / s);
            auto rot = [&](float* v) { // Rodrigues
                const V3 p{v[0], v[1], v[2]};
                const V3 r = p * c + cross(k, p) * s + k * (dot(k, p) * (1.0f - c));
                v[0] = r.x;
                v[1] = r.y;
                v[2] = r.z;
            };
            for (Part& p : parts)
                for (Vert& v : p.verts) {
                    rot(v.p);
                    rot(v.n);
                }
        }
        std::printf("  cliff: mean face normal (%.2f, %.2f, %.2f) turned to +Z\n", n.x, n.y, n.z);
    }

    // Origin and scale: trees, bushes, logs stand on y = 0 at the centre of their lowest metre; rocks are centred.
    Bounds all;
    for (const Part& p : parts)
        for (const Vert& v : p.verts) all.add(v.p);
    V3 origin{0.5f * (all.lo.x + all.hi.x), 0.5f * (all.lo.y + all.hi.y), 0.5f * (all.lo.z + all.hi.z)};
    if (atBase) {
        Bounds foot;
        for (const Part& p : parts)
            if (!p.foliage)
                for (const Vert& v : p.verts)
                    if (v.p[1] < all.lo.y + 0.3f) foot.add(v.p);
        if (foot.hi.x >= foot.lo.x) origin = {0.5f * (foot.lo.x + foot.hi.x), all.lo.y, 0.5f * (foot.lo.z + foot.hi.z)};
        else origin.y = all.lo.y;
    }
    Bounds b;
    for (Part& p : parts) {
        for (Vert& v : p.verts) {
            v.p[0] = (v.p[0] - origin.x) * scale;
            v.p[1] = (v.p[1] - origin.y) * scale;
            v.p[2] = (v.p[2] - origin.z) * scale;
            b.add(v.p);
        }
        weld(p);
    }
    const float height = b.hi.y - b.lo.y;
    float radius = 0.0f;
    for (const Part& p : parts)
        for (const Vert& v : p.verts) radius = std::max(radius, std::hypot(v.p[0], v.p[2]));

    // Trunk (trees): the bark vertices of a slice around 1.3 m, re-centred three times on their centroid within
    // 2.5 x the current radius; radius = median distance from that centre. The axis is then followed upward in 0.5 m
    // slices (centroid of the bark vertices within 2.5 x the radius of the previous centre) to 0.3 x the height.
    float trunkRadius = 0.0f;
    V3 trunkTop{0, 0, 0};
    if (kind == "tree") {
        auto slice = [&](float y, float half, float cx, float cz, float search, float& ox, float& oz, float* median) {
            double sx = 0, sz = 0;
            std::vector<float> d;
            std::size_t n = 0;
            for (const Part& p : parts)
                if (!p.foliage)
                    for (const Vert& v : p.verts)
                        if (std::abs(v.p[1] - y) < half && std::hypot(v.p[0] - cx, v.p[2] - cz) < search) {
                            sx += v.p[0];
                            sz += v.p[2];
                            ++n;
                        }
            if (n < 8) return false;
            ox = float(sx / double(n));
            oz = float(sz / double(n));
            if (median) {
                for (const Part& p : parts)
                    if (!p.foliage)
                        for (const Vert& v : p.verts)
                            if (std::abs(v.p[1] - y) < half && std::hypot(v.p[0] - ox, v.p[2] - oz) < search)
                                d.push_back(std::hypot(v.p[0] - ox, v.p[2] - oz));
                std::sort(d.begin(), d.end());
                *median = d.empty() ? 0.0f : d[d.size() / 2];
            }
            return true;
        };
        float cx = 0, cz = 0, r = 0.6f;
        for (int it = 0; it < 3; ++it) {
            float m = 0;
            if (!slice(1.3f, 0.3f, cx, cz, std::max(2.5f * r, 0.3f), cx, cz, &m)) break;
            r = m;
        }
        trunkRadius = r;
        const float top = 0.3f * height;
        float ax = 0, az = 0;
        for (float y = 0.5f; y <= top + 1e-3f; y += 0.5f) {
            if (!slice(y, 0.25f, ax, az, std::max(2.5f * trunkRadius, 0.3f), ax, az, nullptr)) break;
            trunkTop = {ax, y, az};
        }
    }

    std::size_t sourceTris = 0, sourceFoliage = 0;
    for (const Part& p : parts) {
        sourceTris += p.tris();
        if (p.foliage) sourceFoliage += p.tris();
    }
    // Foliage: bend the normals toward the crown ellipsoid and reduce every card to about two triangles (or to the
    // LOD0 foliage budget when that is larger). Trees with more than four times their LOD0 foliage budget get
    // clump cards instead.
    const bool clumps = kind == "tree" && double(sourceFoliage) > 4.0 * 0.78 * double(budgets[0]);
    const V3 crown{0, b.lo.y + 0.58f * height, 0};
    for (Part& p : parts) {
        if (!p.foliage) continue;
        for (Vert& v : p.verts) {
            const V3 q{v.p[0] / std::max(radius, 0.1f), (v.p[1] - crown.y) / std::max(0.5f * height, 0.1f), v.p[2] / std::max(radius, 0.1f)};
            const V3 radial = norm(q);
            const V3 n = norm(V3{v.n[0], v.n[1], v.n[2]} * 0.35f + radial * 0.65f);
            v.n[0] = n.x;
            v.n[1] = n.y;
            v.n[2] = n.z;
        }
        if (clumps) continue;
        const double share = sourceFoliage == sourceTris ? 1.0 : 0.78;
        reduceCards(p, std::size_t(double(budgets[0]) * share * double(p.tris()) / double(std::max<std::size_t>(sourceFoliage, 1))));
    }
    // Wind weights: x trunk / branch bend (grows with height squared), y flutter of the cards (outer cards more).
    for (Part& p : parts)
        for (Vert& v : p.verts) {
            const float h = std::clamp(v.p[1] / std::max(height, 0.1f), 0.0f, 1.0f);
            const float r = std::clamp(std::hypot(v.p[0], v.p[2]) / std::max(radius, 0.1f), 0.0f, 1.0f);
            v.wind[0] = kind == "rock" || kind == "cliff" || kind == "log" ? 0.0f : h * h;
            v.wind[1] = p.foliage ? std::pow(h, 0.7f) * (0.35f + 0.65f * r) : (kind == "bush" ? 0.5f * h : 0.0f);
        }

    std::map<std::string, bool> foliage;
    for (const Part& p : parts) {
        foliage[p.material] = p.foliage;
    }
    // Source albedo + alpha (linear) for the impostor and clump bakes.
    const json::Value& imp = spec["impostor"];
    const bool wantImpostor = imp.isArray() && imp.array.size() == 2;
    std::map<std::string, Image> albedos;
    if (wantImpostor || clumps)
        for (const auto& [src, m] : mats) {
            if (!foliage.count(src)) continue;
            Plane a = load(m.albedo, 512);
            for (std::size_t i = 0; i < a.px.size(); i += 4)
                for (int c = 0; c < 3; ++c) a.px[i + std::size_t(c)] = srgbToLinear(a.px[i + std::size_t(c)]);
            if (foliage[src] && !m.alpha.empty()) {
                const Plane al = load(m.alpha, 512);
                for (std::size_t i = 0; i < a.px.size(); i += 4) a.px[i + 3] = al.px[i];
            } else {
                for (std::size_t i = 3; i < a.px.size(); i += 4) a.px[i] = 1.0f;
            }
            albedos[src] = toImage(a);
        }

    // Clump cards: all source foliage triangles, one atlas of 8 x 8 tiles (128 px) shared by the LODs.
    const std::string clumpMaterial = id + "_foliage";
    std::vector<FoliageTri> ftris;
    const int clumpTile = 128, clumpGrid = 8;
    Plane clumpAlbedo, clumpNormal;
    int clumpNextTile = 0;
    if (clumps) {
        for (const Part& p : parts) {
            if (!p.foliage) continue;
            const Image* tex = &albedos.at(p.material);
            for (std::size_t t = 0; t + 2 < p.idx.size(); t += 3) {
                FoliageTri ft;
                ft.tex = tex;
                for (int k = 0; k < 3; ++k) {
                    const Vert& v = p.verts[p.idx[t + std::size_t(k)]];
                    ft.p[k] = {v.p[0], v.p[1], v.p[2]};
                    ft.n[k] = {v.n[0], v.n[1], v.n[2]};
                    ft.uv[k][0] = v.uv[0];
                    ft.uv[k][1] = v.uv[1];
                }
                ft.c = (ft.p[0] + ft.p[1] + ft.p[2]) * (1.0f / 3.0f);
                ftris.push_back(ft);
            }
        }
        clumpAlbedo.w = clumpNormal.w = clumpAlbedo.h = clumpNormal.h = clumpTile * clumpGrid;
        clumpAlbedo.px.assign(std::size_t(clumpAlbedo.w) * std::size_t(clumpAlbedo.h) * 4, 0.0f);
        clumpNormal.px.assign(clumpAlbedo.px.size(), 0.0f);
        foliage[clumpMaterial] = true;
    }
    auto clumpPart = [&](std::size_t cardCount, std::size_t li) {
        const int tilesLeft = clumpGrid * clumpGrid - clumpNextTile;
        const int tiles = std::max(1, std::min(tilesLeft / int(budgets.size() - li), 24));
        std::vector<Cluster> cls = clusterFoliage(ftris, cardCount, 0.45f);
        const std::vector<std::size_t> lib = pickLibrary(cls, std::size_t(tiles));
        std::vector<int> libSlot;
        for (std::size_t j = 0; j < lib.size(); ++j) {
            const int slot = clumpNextTile++;
            bakeClusterTile(cls[lib[j]], ftris, clumpTile, (slot % clumpGrid) * clumpTile, (slot / clumpGrid) * clumpTile,
                            clumpAlbedo, clumpNormal);
            libSlot.push_back(slot);
        }
        Part p;
        p.material = clumpMaterial;
        p.foliage = true;
        const float inset = 2.0f / float(clumpTile * clumpGrid), cell = 1.0f / float(clumpGrid);
        for (const Cluster& cl : cls) {
            // Library tile with the nearest aspect ratio; ties within 0.25 (log) broken by the card hash.
            const float aspect = std::log(cl.w / cl.h);
            float best = 1e30f;
            for (std::size_t j : lib) best = std::min(best, std::abs(std::log(cls[j].w / cls[j].h) - aspect));
            std::vector<std::size_t> near;
            for (std::size_t j = 0; j < lib.size(); ++j)
                if (std::abs(std::log(cls[lib[j]].w / cls[lib[j]].h) - aspect) <= best + 0.25f) near.push_back(j);
            const std::size_t pick = near[cl.key % near.size()];
            const int slot = libSlot[pick];
            float u0 = float(slot % clumpGrid) * cell + inset, u1 = u0 + cell - 2.0f * inset;
            const float v0 = float(slot / clumpGrid) * cell + inset, v1 = v0 + cell - 2.0f * inset;
            if ((cl.key >> 8) & 1u) std::swap(u0, u1);
            const std::uint32_t base = std::uint32_t(p.verts.size());
            const float su[4] = {-0.5f, 0.5f, 0.5f, -0.5f}, sv[4] = {-0.5f, -0.5f, 0.5f, 0.5f};
            for (int k = 0; k < 4; ++k) {
                const V3 q = cl.c + cl.u * (su[k] * cl.w) + cl.vb * (sv[k] * cl.h);
                Vert v{};
                v.p[0] = q.x;
                v.p[1] = q.y;
                v.p[2] = q.z;
                v.n[0] = cl.n.x;
                v.n[1] = cl.n.y;
                v.n[2] = cl.n.z;
                v.uv[0] = su[k] < 0 ? u0 : u1;
                v.uv[1] = sv[k] < 0 ? v0 : v1;
                const float h = std::clamp(q.y / std::max(height, 0.1f), 0.0f, 1.0f);
                const float r = std::clamp(std::hypot(q.x, q.z) / std::max(radius, 0.1f), 0.0f, 1.0f);
                v.wind[0] = h * h;
                v.wind[1] = std::pow(h, 0.7f) * (0.35f + 0.65f * r);
                p.verts.push_back(v);
            }
            for (std::uint32_t k : {0u, 1u, 2u, 0u, 2u, 3u}) p.idx.push_back(base + k);
        }
        std::printf("  lod%zu clump cards: %zu cards from %zu source triangles, %zu library tiles\n", li, cls.size(),
                    ftris.size(), lib.size());
        return p;
    };

    // LODs.
    std::vector<Lod> lods;
    for (std::size_t li = 0; li < budgets.size(); ++li) {
        Lod lod;
        const std::size_t budget = budgets[li];
        std::size_t foliageTris = 0, opaqueTris = 0;
        for (const Part& p : parts) (p.foliage ? foliageTris : opaqueTris) += p.tris();
        // Trees spend most of the budget on the crown.
        const double opaqueShare = foliageTris == 0 ? 1.0 : 0.22;
        if (clumps) {
            const std::size_t cards = std::max<std::size_t>(16, std::size_t(double(budget) * (1.0 - opaqueShare) / 2.0));
            lod.parts.push_back(clumpPart(cards, li));
        }
        for (const Part& src : parts) {
            if (clumps && src.foliage) continue;
            Part p = src;
            if (p.foliage) {
                const std::size_t target = std::size_t(double(budget) * (1.0 - opaqueShare) * double(p.tris()) / double(std::max<std::size_t>(foliageTris, 1)));
                thinCards(p, std::max<std::size_t>(target, 8), 3.0f);
            } else {
                const std::size_t target = std::size_t(double(budget) * opaqueShare * double(p.tris()) / double(std::max<std::size_t>(opaqueTris, 1)));
                lod.error = std::max(lod.error, simplifyOpaque(p, std::max<std::size_t>(target, 12)));
            }
            compact(p);
            if (!p.idx.empty()) lod.parts.push_back(std::move(p));
        }
        std::printf("  lod%zu: %zu triangles (budget %zu), error %.3f m\n", li, lod.tris(), budget, lod.error);
        lods.push_back(std::move(lod));
    }

    // Materials used by the LODs (clump trees no longer use their source twig materials).
    std::set<std::string> used;
    for (const Lod& l : lods)
        for (const Part& p : l.parts) used.insert(p.material);
    const int texSize = int(spec["textureSize"].numberOr(1024)), normalSize = int(spec["normalSize"].numberOr(512));
    if (spec["writeTextures"].boolOr(true))
        for (const auto& [src, m] : mats)
            if (used.count(src) && m.write) writeMaterialTextures(m, foliage[src], texSize, normalSize, outDir);
    if (clumps) {
        Plane mask = clumpAlbedo;
        for (std::size_t i = 3; i < mask.px.size(); i += 4) mask.px[i] = mask.px[i] >= 0.5f ? 1.0f : 0.0f;
        dilate(mask, 8);
        for (std::size_t i = 0; i < clumpAlbedo.px.size(); i += 4)
            if (clumpAlbedo.px[i + 3] < 0.5f)
                for (int c = 0; c < 3; ++c) clumpAlbedo.px[i + std::size_t(c)] = mask.px[i + std::size_t(c)];
        writeKtx(clumpAlbedo, true, true, outDir + "/" + clumpMaterial + "_albedo.ktx2");
        writeKtx(half(clumpNormal), false, false, outDir + "/" + clumpMaterial + "_nra.ktx2");
        MaterialSpec cm;
        cm.name = clumpMaterial;
        mats[clumpMaterial] = cm;
    }

    // Impostor atlas (clump trees: from the full source foliage).
    json::Value extras = json::Value::makeObject();
    if (wantImpostor) {
        const int tileW = int(imp.array[0].number), tileH = int(imp.array[1].number), views = 8;
        Lod bakeLod;
        for (const Part& p : lods[0].parts)
            if (!clumps || p.material != clumpMaterial) bakeLod.parts.push_back(p);
        if (clumps)
            for (const Part& p : parts)
                if (p.foliage) bakeLod.parts.push_back(p);
        Plane ia, in2;
        bakeImpostor(bakeLod, albedos, foliage, radius, height, views, tileW, tileH, ia, in2);
        Plane mask = ia;
        for (std::size_t i = 3; i < mask.px.size(); i += 4) mask.px[i] = mask.px[i] >= 0.5f ? 1.0f : 0.0f;
        dilate(mask, 8);
        Plane nmask = in2;
        for (std::size_t i = 0; i < ia.px.size(); i += 4)
            if (ia.px[i + 3] < 0.5f) {
                for (int c = 0; c < 3; ++c) ia.px[i + std::size_t(c)] = mask.px[i + std::size_t(c)];
                nmask.px[i] = 0.5f;
                nmask.px[i + 1] = 0.5f;
                nmask.px[i + 2] = 0.85f;
                nmask.px[i + 3] = 1.0f;
            }
        writeKtx(ia, true, true, outDir + "/" + id + "_impostor_albedo.ktx2");
        writeKtx(nmask, false, false, outDir + "/" + id + "_impostor_nra.ktx2");
        json::Value ij = json::Value::makeObject();
        ij.object["albedo"] = str(id + "_impostor_albedo.ktx2");
        ij.object["nra"] = str(id + "_impostor_nra.ktx2");
        ij.object["views"] = num(views);
        extras.object["impostor"] = ij;
    }

    // glb.
    GlbWriter w;
    json::Value doc = json::Value::makeObject();
    json::Value asset = json::Value::makeObject();
    asset.object["version"] = str("2.0");
    asset.object["generator"] = str("trailforge_modelc");
    doc.object["asset"] = asset;
    json::Value materials = json::Value::makeArray(), textures = json::Value::makeArray(), images = json::Value::makeArray();
    std::map<std::string, int> matIndex;
    for (const auto& [src, m] : mats) {
        if (!used.count(src)) continue;
        const int ia = int(images.array.size());
        json::Value im = json::Value::makeObject();
        im.object["uri"] = str(m.name + "_albedo.ktx2");
        images.array.push_back(im);
        im.object["uri"] = str(m.name + "_nra.ktx2");
        images.array.push_back(im);
        json::Value t = json::Value::makeObject();
        t.object["source"] = num(ia);
        textures.array.push_back(t);
        t.object["source"] = num(ia + 1);
        textures.array.push_back(t);
        json::Value mj = json::Value::makeObject();
        mj.object["name"] = str(m.name);
        json::Value pbr = json::Value::makeObject();
        json::Value bt = json::Value::makeObject();
        bt.object["index"] = num(ia);
        pbr.object["baseColorTexture"] = bt;
        pbr.object["metallicFactor"] = num(0);
        mj.object["pbrMetallicRoughness"] = pbr;
        json::Value nt = json::Value::makeObject();
        nt.object["index"] = num(ia + 1);
        mj.object["normalTexture"] = nt;
        if (foliage[src]) {
            mj.object["alphaMode"] = str("MASK");
            mj.object["alphaCutoff"] = num(0.5);
            mj.object["doubleSided"] = json::Value::makeBool(true);
            json::Value me = json::Value::makeObject();
            me.object["foliage"] = json::Value::makeBool(true);
            mj.object["extras"] = me;
        }
        matIndex[src] = int(materials.array.size());
        materials.array.push_back(mj);
    }
    json::Value meshes = json::Value::makeArray(), nodesJ = json::Value::makeArray();
    json::Value lodTris = json::Value::makeArray(), lodErr = json::Value::makeArray();
    for (std::size_t li = 0; li < lods.size(); ++li) {
        meshes.array.push_back(meshJson(w, "lod" + std::to_string(li), lods[li].parts, matIndex, false));
        lodTris.array.push_back(num(double(lods[li].tris())));
        lodErr.array.push_back(num(std::round(lods[li].error * 1000.0) / 1000.0));
    }
    std::size_t collisionTris = 0;
    if (collisionLod >= 0 && std::size_t(collisionLod) < lods.size()) {
        meshes.array.push_back(meshJson(w, "collision", lods[std::size_t(collisionLod)].parts, matIndex, true));
        collisionTris = lods[std::size_t(collisionLod)].tris();
    }
    json::Value node = json::Value::makeObject();
    node.object["name"] = str(id);
    node.object["mesh"] = num(0);
    nodesJ.array.push_back(node);
    json::Value sceneJ = json::Value::makeObject();
    json::Value sceneNodes = json::Value::makeArray();
    sceneNodes.array.push_back(num(0));
    sceneJ.object["nodes"] = sceneNodes;
    json::Value scenes = json::Value::makeArray();
    scenes.array.push_back(sceneJ);
    doc.object["scenes"] = scenes;
    doc.object["scene"] = num(0);
    doc.object["nodes"] = nodesJ;
    doc.object["meshes"] = meshes;
    doc.object["materials"] = materials;
    doc.object["textures"] = textures;
    doc.object["images"] = images;
    doc.object["accessors"] = w.accessors;
    doc.object["bufferViews"] = w.views;
    json::Value buffers = json::Value::makeArray();
    json::Value buf = json::Value::makeObject();
    buf.object["byteLength"] = num(double((w.bin.size() + 3) / 4 * 4));
    buffers.array.push_back(buf);
    doc.object["buffers"] = buffers;
    auto r3 = [](double v) { return num(std::round(v * 1000.0) / 1000.0); };
    extras.object["kind"] = str(kind);
    extras.object["height"] = r3(height);
    extras.object["radius"] = r3(radius);
    json::Value bmin = json::Value::makeArray(), bmax = json::Value::makeArray();
    for (float v : {b.lo.x, b.lo.y, b.lo.z}) bmin.array.push_back(r3(v));
    for (float v : {b.hi.x, b.hi.y, b.hi.z}) bmax.array.push_back(r3(v));
    extras.object["boundsMin"] = bmin;
    extras.object["boundsMax"] = bmax;
    if (kind == "tree") {
        extras.object["trunkRadius"] = r3(trunkRadius);
        json::Value tt = json::Value::makeArray();
        for (float v : {trunkTop.x, trunkTop.y, trunkTop.z}) tt.array.push_back(r3(v));
        extras.object["trunkTop"] = tt;
    }
    extras.object["lodTriangles"] = lodTris;
    extras.object["lodError"] = lodErr;
    if (collisionLod >= 0) extras.object["collisionLod"] = num(collisionLod);
    extras.object["sourceTriangles"] = num(double(sourceTris));
    doc.object["extras"] = extras;
    writeGlb(outDir + "/" + id + ".glb", doc, w.bin);

    json::Value stats = extras;
    stats.object["id"] = str(id);
    stats.object["collisionTriangles"] = num(double(collisionTris));
    stats.object["sourceFoliageTriangles"] = num(double(sourceFoliage));
    std::string line = json::write(stats);
    line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
    std::printf("STATS %s\n", line.c_str());
    return 0;
}
