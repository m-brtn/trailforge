#pragma once

#include "core/ModelAsset.h"

#include <Jolt/Jolt.h>

#include <cstdint>
#include <vector>

namespace tf {

// Modular procedural building kit: houses, the fuel station, garage, lighthouse, radio mast, castle ruins, quarry
// machinery, docks, tents, fences, gardens, power poles and small structures, generated from a few parameters.
//
// One generator call produces both sides from the same parts: the render LODs (core/ModelAsset, drawn instanced by
// the level model renderer) and the collision as convex pieces. Every solid part (wall block, roof slab, post,
// beam, deck, plinth) is one convex collision piece and is also drawn, unchanged, in the far LOD. The near LODs
// replace the flat wall faces with the same planes cut by window and door recesses (at most kKitRecess deep) and add
// trims that stand at most kKitTrim proud of the solid parts (frames, sills, shutters, flower boxes, barge boards,
// balusters, gutters). Thin parts out of the vehicle's reach (power line wires, mast antennas above 6 m) have no
// collision. Building space: origin on the floor level at the footprint center, +Y up, +Z the front (door side).

// Kit materials: a layer of assets/textures/buildings/buildings.json (nullptr: untextured, the renderer's white
// albedo), a linear albedo tint and a constant roughness (< 0: the layer's roughness map).
enum class KitMat : std::uint8_t {
    Plaster, PlasterWhite, PlasterRed, PlasterOchre, Stone, Castle, Siding, SidingDark, Wood, WoodDark, PaintGreen,
    PaintRed, PaintWhite, RoofTiles, Shingles, Corrugated, CorrugatedGrey, CorrugatedRed, Metal, MetalRed,
    MetalWhite, MetalDark, Rust, Concrete, Glass, Shadow, Soil, Leaves, Flowers, NylonOrange, NylonGreen, Lamp, Count
};
constexpr int kKitMatCount = int(KitMat::Count);

struct KitMaterial {
    const char* name;
    const char* layer; // buildings.json layer id, nullptr: untextured
    float tile;        // m per texture repeat (buildings.json "tile" of the layer)
    float tint[3];
    float roughness;   // >= 0: constant (untextured materials), < 0: from the layer's map
};
const KitMaterial& kitMaterial(KitMat m);

constexpr float kKitRecess = 0.16f; // m: deepest window / door recess behind a solid wall face (near LODs)
constexpr float kKitTrim = 0.30f;   // m: farthest a near-LOD trim stands proud of the solid parts (flower boxes)

enum class BuildingKind : std::uint8_t {
    House,          // w x d footprint, floors, variant bits (kHouse*)
    FuelCanopy,     // w x d canopy over a concrete pad with two pump islands
    FuelShop,       // w x d flat-roofed shop with a glass front
    Pylon,          // h m sign pylon
    Garage,         // w x d corrugated workshop hall with a roller door; variant 1: rusty quarry shed
    Lighthouse,     // h m tower, gallery and lantern
    RadioMast,      // h m lattice mast with antennas
    RadioHut,       // w x d concrete equipment hut
    RuinWall,       // w m long, h m high crumbling curtain wall, variant: seed
    RuinTower,      // w m diameter, h m broken round tower, variant: seed
    RuinGate,       // w m wide gateway with two pillars
    Crusher,        // quarry crusher on legs with a hopper, h m high
    Conveyor,       // w m long belt rising h m
    Silo,           // w m diameter, h m high silo on legs
    Dock,           // w m long, d m wide plank deck on posts reaching h m below it
    Tent,           // variant 0: orange ridge tent, 1: green dome
    Fence,          // w m long, variant 0: picket, 1: two-rail, 2: metal posts and rails
    Garden,         // w x d picket-fenced garden with raised beds
    PowerPole,      // h m wooden pole with a cross arm
    Wire,           // render only: three conductors over w m, dropping h m (power line span)
    Woodpile,       // w m long stacked logs under a small roof
    Cairn,          // stone cairn with a summit cross
    Signpost,       // post with two arrow boards
    Well,           // village well with a roof
    Shelter,        // w x d open timber shelter (lean-to roof on posts)
    Bench,          // w m long park bench facing +Z
    Count
};

// House variant bits.
constexpr int kHouseStoneBase = 1;     // ground floor in rubble stone (else plaster)
constexpr int kHouseUpperPlaster = 2;  // upper floors plastered (else timber siding)
constexpr int kHouseShingles = 4;      // wooden shingles (else clay tiles)
constexpr int kHouseShutterShift = 3;  // 2 bits: shutters green, red, dark wood, none
constexpr int kHouseBalcony = 32;      // balcony along the front gable
constexpr int kHouseTimber = 64;       // all walls timber siding (cabin)
constexpr int kHouseSideBalcony = 128; // second balcony along the +X side
constexpr int kHouseOchre = 256;       // ochre plaster (else white)

struct BuildingSpec {
    BuildingKind kind = BuildingKind::House;
    float w = 8.0f, d = 8.0f, h = 0.0f; // meaning per kind (see BuildingKind)
    int floors = 2;
    int variant = 0;
    bool operator==(const BuildingSpec& o) const = default;
};

struct BuildingGeometry {
    std::vector<JPH::Float3> collision;     // 3 positions per triangle, building space, wound outward
    std::vector<std::uint32_t> pieceStarts; // first triangle of every convex piece
    JPH::Float3 boundsMin{0, 0, 0}, boundsMax{0, 0, 0}; // of the collision
    ModelAsset model;                       // render LODs 0..2 (empty unless built with render = true)
    float footprintX = 0.0f, footprintZ = 0.0f; // half extents of the walls (map footprint, ground sampling)
};

// Builds a spec. render = false skips the render meshes (the level bake needs only the collision).
BuildingGeometry buildBuilding(const BuildingSpec& spec, bool render);
// Placement role: true for structures that keep 3 m from roads and out of water; false for small props (1.5 m).
bool buildingIsStructure(BuildingKind k);
// Ground plinth depth below the floor (m): the solid base reaches this far down to cover sloping ground.
float buildingPlinthDepth(BuildingKind k);
const char* buildingKindName(BuildingKind k);

} // namespace tf
