#pragma once

// Jolt -> GLM conversions. Both libraries: right-handed, column-major matrices.
// Jolt Quat stores (x, y, z, w); glm::quat is constructed as (w, x, y, z).

#include <Jolt/Jolt.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace tf {

inline glm::vec3 toGlm(JPH::Vec3Arg v) { return {v.GetX(), v.GetY(), v.GetZ()}; }
inline glm::quat toGlm(JPH::QuatArg q) { return {q.GetW(), q.GetX(), q.GetY(), q.GetZ()}; }

inline glm::mat4 toGlm(const JPH::Mat44& m) {
    glm::mat4 r;
    for (int c = 0; c < 4; ++c) {
        const JPH::Vec4 col = m.GetColumn4(JPH::uint(c));
        r[c] = glm::vec4(col.GetX(), col.GetY(), col.GetZ(), col.GetW());
    }
    return r;
}

inline glm::mat4 poseMatrix(const glm::vec3& position, const glm::quat& rotation) {
    glm::mat4 m = glm::mat4_cast(rotation);
    m[3] = glm::vec4(position, 1.0f);
    return m;
}

} // namespace tf
