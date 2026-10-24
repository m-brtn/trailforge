#include "physics/PhysicsWorld.h"

#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <thread>

namespace tf {
namespace {

std::mutex g_runtimeMutex;
int g_runtimeRefs = 0;

void traceImpl(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    va_end(args);
    std::fputc('\n', stderr);
}

} // namespace

JoltRuntime::JoltRuntime() {
    std::lock_guard lock(g_runtimeMutex);
    if (g_runtimeRefs++ == 0) {
        JPH::RegisterDefaultAllocator();
        JPH::Trace = traceImpl;
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
}

JoltRuntime::~JoltRuntime() {
    std::lock_guard lock(g_runtimeMutex);
    if (--g_runtimeRefs == 0) {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

PhysicsWorld::PhysicsWorld() : PhysicsWorld(Settings{}) {}

PhysicsWorld::PhysicsWorld(const Settings& settings) {
    m_tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(32 * 1024 * 1024);
    int threads = settings.workerThreads;
    if (threads < 0) {
        const int hw = static_cast<int>(std::thread::hardware_concurrency());
        threads = std::clamp(hw - 1, 1, 4);
    }
    m_jobSystem = std::make_unique<JPH::JobSystemThreadPool>(
        JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, threads);

    m_system = std::make_unique<JPH::PhysicsSystem>();
    m_system->Init(settings.maxBodies, 0, settings.maxBodyPairs, settings.maxContactConstraints,
                   m_bpMap, m_objVsBp, m_objPair);
    m_system->SetGravity(JPH::Vec3(0.0f, -9.81f, 0.0f));
}

PhysicsWorld::~PhysicsWorld() {
    m_system.reset();
    m_jobSystem.reset();
    m_tempAllocator.reset();
}

JPH::EPhysicsUpdateError PhysicsWorld::step(float dt, int collisionSteps) {
    return m_system->Update(dt, collisionSteps, m_tempAllocator.get(), m_jobSystem.get());
}

} // namespace tf
