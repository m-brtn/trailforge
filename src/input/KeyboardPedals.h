#pragma once

#include <algorithm>

namespace tf {

// Keyboard pedals. A key is on or off, but a driver does not stamp a pedal to the floor: holding W or S presses the
// pedal in two stages, quickly to a firm level, then progressively to full. Releasing the key releases the pedal at
// once. This models the driver's foot (an input filter, like an analog trigger held halfway); it applies no force and
// the simulation's own pedal smoothing still follows. Gamepad triggers are analog and do not go through it.
//
// Levels from `trailforge_probe rampsweep` / `brakesweep` on the shipped SUV (2H, ABS off, concrete): half throttle
// never spins the rear wheels, an instant full throttle spins them at ~4.8 m/s for ~2 s (~3.9 m/s peak on the
// 2.56 m wheelbase of v0.15, suv_marks_keyboard), a linear 0.5 s ramp still at 3.5 m/s; 0.5 at once and full after another 0.8 s stays below 1 m/s and reaches 30 km/h quickest. The rear
// wheels lock from a brake of ~0.45 at 50 km/h; 0.4 stops without locking.
struct KeyboardPedalSettings {
    float throttleFirm = 0.5f;      // reached within throttleFirmTime of the key press
    float throttleFirmTime = 0.15f; // s
    float throttleFullTime = 0.8f;  // s from firm to full
    float brakeFirm = 0.4f;
    float brakeFirmTime = 0.1f;     // s
    float brakeFullTime = 0.7f;     // s from firm to full
};

class KeyboardPedals {
public:
    // Once per frame with the held keys; returns the pedal positions 0..1.
    void update(bool throttleKey, bool brakeKey, float dt, const KeyboardPedalSettings& s = {}) {
        m_throttle = press(m_throttle, throttleKey, dt, s.throttleFirm, s.throttleFirmTime, s.throttleFullTime);
        m_brake = press(m_brake, brakeKey, dt, s.brakeFirm, s.brakeFirmTime, s.brakeFullTime);
    }
    float throttle() const { return m_throttle; }
    float brake() const { return m_brake; }
    void reset() { m_throttle = m_brake = 0.0f; }

private:
    static float press(float pedal, bool held, float dt, float firm, float firmTime, float fullTime) {
        if (!held) return 0.0f;
        const float rate = pedal < firm ? firm / std::max(firmTime, 1e-3f) : (1.0f - firm) / std::max(fullTime, 1e-3f);
        const float next = pedal + rate * std::max(dt, 0.0f);
        // Do not overshoot the knee within one long frame: the rest of the step continues at the slower rate.
        if (pedal < firm && next > firm) {
            const float rest = dt - (firm - pedal) / rate;
            return std::min(1.0f, firm + (1.0f - firm) / std::max(fullTime, 1e-3f) * rest);
        }
        return std::min(1.0f, next);
    }

    float m_throttle = 0.0f, m_brake = 0.0f;
};

} // namespace tf
