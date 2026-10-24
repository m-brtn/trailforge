#pragma once

// Coordinate system and units (shared by the whole project)
// -----------------------------------------------------
// * Physics units: meters, kilograms, seconds; angles in radians (degrees in the UI).
// * World: right-handed coordinate system, up is +Y, gravity (0, -9.81, 0) m/s^2.
// * Vehicle local axes (as in the Jolt VehicleConstraintTest sample):
//     +Z forward, +Y up, +X left (right-handed triple: X = Y x Z).
//   Wheels 0/1 are front left/right, 2/3 are rear left/right.
// * Jolt Mat44 and glm::mat4 are both column-major; a Jolt matrix is copied into glm
//   column by column without transposing (see render/MathConvert.h).
// * Jolt Quat stores (x, y, z, w); the glm::quat constructor takes (w, x, y, z).
// * Rendering: glm::lookAt (right-handed, the camera looks down view -Z),
//   glm::perspective with GLM_FORCE_DEPTH_ZERO_TO_ONE (NDC depth in [0, 1], as in SDL GPU).

#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <memory>

namespace tf {

namespace Layers {
inline constexpr JPH::ObjectLayer Static = 0;
inline constexpr JPH::ObjectLayer Moving = 1;
inline constexpr JPH::uint Count = 2;
} // namespace Layers

namespace BroadPhaseLayers {
inline constexpr JPH::BroadPhaseLayer Static{0};
inline constexpr JPH::BroadPhaseLayer Moving{1};
inline constexpr JPH::uint Count = 2;
} // namespace BroadPhaseLayers

class BroadPhaseLayerMap final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::Count; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        return layer == Layers::Static ? BroadPhaseLayers::Static : BroadPhaseLayers::Moving;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        return layer == BroadPhaseLayers::Static ? "Static" : "Moving";
    }
#endif
};

class ObjectVsBroadPhaseFilter final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer bp) const override {
        return layer == Layers::Moving || bp == BroadPhaseLayers::Moving;
    }
};

class ObjectPairFilter final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        return a == Layers::Moving || b == Layers::Moving;
    }
};

// RAII counter for global Jolt registration (allocator, Factory, types): registration
// happens with the first instance and is removed when the last one is destroyed.
// Required before creating any Jolt objects (including PhysicsMaterial).
class JoltRuntime {
public:
    JoltRuntime();
    ~JoltRuntime();
    JoltRuntime(const JoltRuntime&) = delete;
    JoltRuntime& operator=(const JoltRuntime&) = delete;
};

// Owns the PhysicsSystem, the temp allocator and the Jolt thread pool.
class PhysicsWorld {
public:
    struct Settings {
        JPH::uint maxBodies = 2048;
        JPH::uint maxBodyPairs = 4096;
        JPH::uint maxContactConstraints = 4096;
        int workerThreads = -1; // -1: min(hardware - 1, 4)
    };

    PhysicsWorld();
    explicit PhysicsWorld(const Settings& settings);
    ~PhysicsWorld();
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    JPH::PhysicsSystem& system() { return *m_system; }
    const JPH::PhysicsSystem& system() const { return *m_system; }
    JPH::BodyInterface& bodies() { return m_system->GetBodyInterface(); }

    // One fixed step. Returns Jolt errors (EPhysicsUpdateError).
    JPH::EPhysicsUpdateError step(float dt, int collisionSteps = 1);

    void optimizeBroadPhase() { m_system->OptimizeBroadPhase(); }

private:
    JoltRuntime m_runtime;
    BroadPhaseLayerMap m_bpMap;
    ObjectVsBroadPhaseFilter m_objVsBp;
    ObjectPairFilter m_objPair;
    std::unique_ptr<JPH::TempAllocatorImpl> m_tempAllocator;
    std::unique_ptr<JPH::JobSystemThreadPool> m_jobSystem;
    std::unique_ptr<JPH::PhysicsSystem> m_system;
};

} // namespace tf
